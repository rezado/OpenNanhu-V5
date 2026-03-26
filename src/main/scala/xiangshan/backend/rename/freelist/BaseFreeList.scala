/***************************************************************************************
* Copyright (c) 2020-2021 Institute of Computing Technology, Chinese Academy of Sciences
* Copyright (c) 2020-2021 Peng Cheng Laboratory
*
* XiangShan is licensed under Mulan PSL v2.
* You can use this software according to the terms and conditions of the Mulan PSL v2.
* You may obtain a copy of Mulan PSL v2 at:
*          http://license.coscl.org.cn/MulanPSL2
*
* THIS SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OF ANY KIND,
* EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO NON-INFRINGEMENT,
* MERCHANTABILITY OR FIT FOR A PARTICULAR PURPOSE.
*
* See the Mulan PSL v2 for more details.
***************************************************************************************/

package xiangshan.backend.rename.freelist

import org.chipsalliance.cde.config.Parameters
import chisel3._
import chisel3.util._
import xiangshan._
import xiangshan.backend.rename.SnapshotGenerator
import utils._
import xs.utils._
import xs.utils.perf._


abstract class BaseFreeList(size: Int, numLogicRegs:Int = 32)(implicit p: Parameters) extends XSModule with HasResizeCircularQueuePtrHelper {
  val io = IO(new Bundle {
    val redirect = Input(Bool())
    val walk = Input(Bool())
    val psize = Input(UInt(log2Up(size + 1).W))

    val allocateReq = Input(Vec(RenameWidth, Bool()))
    val walkReq = Input(Vec(RabCommitWidth, Bool()))
    val allocatePhyReg = Output(Vec(RenameWidth, UInt(PhyRegIdxWidth.W)))
    val canAllocate = Output(Bool())
    val doAllocate = Input(Bool())

    val freeReq = Input(Vec(RabCommitWidth, Bool()))
    val freePhyReg = Input(Vec(RabCommitWidth, UInt(PhyRegIdxWidth.W)))

    val commit = Input(new RabCommitIO)

    val snpt = Input(new SnapshotPort)

    val debug_rat = if(backendParams.debugEn) Some(Vec(numLogicRegs, Input(UInt(PhyRegIdxWidth.W)))) else None
  })

  class FreeListPtr extends ResizeCircularQueuePtr[FreeListPtr](size)

  object FreeListPtr {
    def apply(f: Boolean, v: Int): FreeListPtr = {
      val ptr = Wire(new FreeListPtr)
      ptr.flag := f.B
      ptr.value := v.U
      ptr.psize.valid := false.B
      ptr.psize.bits := size.U
      ptr
    }
  }

  protected def withPSize(ptr: FreeListPtr): FreeListPtr = {
    val sizedPtr = WireInit(ptr)
    sizedPtr.psize.valid := true.B
    sizedPtr.psize.bits := io.psize
    sizedPtr
  }

  protected def ptrToOH(ptr: FreeListPtr): UInt = withPSize(ptr).toOH

  protected def resizedTailPtr(flag: Boolean): FreeListPtr = {
    val ptr = Wire(new FreeListPtr)
    ptr.flag := flag.B
    ptr.value := Mux(io.psize === 0.U, 0.U, io.psize - 1.U)
    ptr.psize.valid := true.B
    ptr.psize.bits := io.psize
    ptr
  }

  val lastPSize = RegNext(io.psize, size.U(log2Up(size + 1).W))
  val psizeChanged = lastPSize =/= io.psize
  val redirectOrResize = io.redirect || psizeChanged

  XSError(io.psize === 0.U, "FreeList logical size should not be zero\n")
  XSError(io.psize > size.U, p"FreeList logical size should not exceed physical size ${size.U}\n")

  val lastCycleRedirect = GatedValidRegNext(redirectOrResize, false.B)
  val lastCycleSnpt = RegNext(io.snpt, 0.U.asTypeOf(io.snpt))

  val headPtr = RegInit(FreeListPtr(false, 0))
  val headPtrOH = RegInit(1.U(size.W))
  val archHeadPtr = RegInit(FreeListPtr(false, 0))
  XSError(ptrToOH(headPtr) =/= headPtrOH, p"wrong one-hot reg between $headPtr and $headPtrOH")

  val snapshots = SnapshotGenerator(headPtr, io.snpt.snptEnq, io.snpt.snptDeq, redirectOrResize, io.snpt.flushVec)

  val redirectedHeadPtr = Mux(
    lastCycleSnpt.useSnpt,
    withPSize(snapshots(lastCycleSnpt.snptSelect)) + PopCount(io.walkReq),
    withPSize(archHeadPtr) + PopCount(io.walkReq)
  )
  val redirectedHeadPtrOH = ptrToOH(redirectedHeadPtr)
}
