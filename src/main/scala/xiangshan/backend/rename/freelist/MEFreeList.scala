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
import utils._
import xs.utils._
import xs.utils.perf._


class MEFreeList(size: Int)(implicit p: Parameters) extends BaseFreeList(size) with HasPerfEvents {
  private val freeListInit = Seq.tabulate(size - 1)(i => (i + 1).U(PhyRegIdxWidth.W)) :+ 0.U(PhyRegIdxWidth.W)
  val freeList = RegInit(VecInit(
    // originally {1, 2, ..., size - 1} are free. Register 0-31 are mapped to x0.
    freeListInit))

  val tailPtr = RegInit(FreeListPtr(false, size - 1))

  val doWalkRename = io.walk && io.doAllocate && !redirectOrResize
  val doNormalRename = io.canAllocate && io.doAllocate && !redirectOrResize
  val doRename = doWalkRename || doNormalRename
  val doCommit = io.commit.isCommit

  /**
    * Allocation: from freelist (same as StdFreelist)
    */
  val phyRegCandidates = VecInit.tabulate(RenameWidth + 1)(i => freeList((withPSize(headPtr) + i.U).value))
  for (i <- 0 until RenameWidth) {
    // enqueue instr, is move elimination
    io.allocatePhyReg(i) := phyRegCandidates(PopCount(io.allocateReq.take(i)))
  }
  // update arch head pointer
  val archAlloc = io.commit.commitValid zip io.commit.info map {
    case (valid, info) => valid && info.rfWen && !info.isMove
  }
  val numArchAllocate = PopCount(archAlloc)
  val archHeadPtrNew  = withPSize(archHeadPtr) + numArchAllocate
  val archHeadPtrNext = Mux(doCommit, archHeadPtrNew, archHeadPtr)

  // update head pointer
  val numAllocate = Mux(io.walk, PopCount(io.walkReq), PopCount(io.allocateReq))
  val headPtrNew   = Mux(lastCycleRedirect, redirectedHeadPtr, withPSize(headPtr) + numAllocate)
  val headPtrOHNew = Mux(lastCycleRedirect, redirectedHeadPtrOH, ptrToOH(headPtrNew))
  val headPtrNext   = Mux(doRename, headPtrNew, headPtr)
  val headPtrOHNext = Mux(doRename, headPtrOHNew, headPtrOH)

  /**
    * Deallocation: when refCounter becomes zero, the register can be released to freelist
    */
  for (i <- 0 until RabCommitWidth) {
    when (io.freeReq(i)) {
      val freePtr = withPSize(tailPtr) + PopCount(io.freeReq.take(i))
      freeList(freePtr.value) := io.freePhyReg(i)
    }
  }

  // update tail pointer
  val tailPtrNext = withPSize(tailPtr) + PopCount(io.freeReq)

  val freeRegCnt = Mux(doWalkRename && !lastCycleRedirect, distanceBetween(withPSize(tailPtrNext), withPSize(headPtr)) - PopCount(io.walkReq),
                   Mux(doNormalRename,                     distanceBetween(withPSize(tailPtrNext), withPSize(headPtr)) - PopCount(io.allocateReq),
                                                           distanceBetween(withPSize(tailPtrNext), withPSize(headPtr))))
  
  val freeRegCntReg = Mux(redirectOrResize, 0.U,RegNext(freeRegCnt))
  io.canAllocate := freeRegCntReg >= RenameWidth.U

  when (psizeChanged) {
    headPtr := FreeListPtr(false, 0)
    headPtrOH := 1.U(size.W)
    archHeadPtr := FreeListPtr(false, 0)
    tailPtr := resizedTailPtr(false)
    freeList.zip(freeListInit).foreach { case (entry, init) => entry := init }
  }.otherwise {
    archHeadPtr := archHeadPtrNext
    headPtr   := headPtrNext
    headPtrOH := headPtrOHNext
    tailPtr := tailPtrNext
  }

  if(backendParams.debugEn){
    // The integer freelist can be resized dynamically by DSE. When that happens we
    // reset the active freelist state immediately, while the debug arch RAT/head
    // shadow registers still lag by two cycles. Clear the delayed shadows on resize
    // and suppress the check until the debug view has fully caught up.
    def zeroDebugRat = VecInit(Seq.fill(32)(0.U(PhyRegIdxWidth.W)))
    val debugArchHeadPtr_d1 = RegInit(FreeListPtr(false, 0))
    val debugArchHeadPtr = RegInit(FreeListPtr(false, 0))
    val debugArchRAT_d1 = RegInit(zeroDebugRat)
    val debugArchRAT = RegInit(zeroDebugRat)
    when (psizeChanged) {
      debugArchHeadPtr_d1 := FreeListPtr(false, 0)
      debugArchHeadPtr := FreeListPtr(false, 0)
      debugArchRAT_d1 := zeroDebugRat
      debugArchRAT := zeroDebugRat
    }.otherwise {
      debugArchHeadPtr_d1 := archHeadPtr
      debugArchHeadPtr := debugArchHeadPtr_d1
      debugArchRAT_d1 := io.debug_rat.get
      debugArchRAT := debugArchRAT_d1
    }
    val resizeInFlight = psizeChanged || RegNext(psizeChanged, false.B) || RegNext(RegNext(psizeChanged, false.B), false.B)
    val debugUniqPR = Seq.tabulate(32)(i => i match {
      case 0 => true.B
      case _ => !debugArchRAT.take(i).map(_ === debugArchRAT(i)).reduce(_ || _)
    })
    XSError(!resizeInFlight && distanceBetween(withPSize(tailPtr), withPSize(debugArchHeadPtr)) +& PopCount(debugUniqPR) =/= io.psize, "Integer physical register should be in either arch RAT or arch free list\n")
  }

  QueuePerf(size = size, utilization = freeRegCntReg, full = freeRegCntReg === 0.U)

  XSPerfAccumulate("allocation_blocked_cycle", !io.canAllocate)
  XSPerfAccumulate("can_alloc_wrong", !io.canAllocate && freeRegCnt >= RenameWidth.U)

  val perfEvents = Seq(
    ("me_freelist_1_4_valid", freeRegCntReg <  (size / 4).U                                     ),
    ("me_freelist_2_4_valid", freeRegCntReg >= (size / 4).U && freeRegCntReg <= (size / 2).U    ),
    ("me_freelist_3_4_valid", freeRegCntReg >= (size / 2).U && freeRegCntReg <= (size * 3 / 4).U),
    ("me_freelist_4_4_valid", freeRegCntReg >= (size * 3 / 4).U                                 ),
  )
  generatePerfEvent()
}
