package yunsuan.vector.vpermfsm

import chisel3._
import chisel3.util._
import yunsuan.util._
import yunsuan.vector.vpermutil._

class ViqCtrl extends VPermBundle {
    val opcode = UInt(3.W)
    val sew = UInt(2.W)

    val vs1 = UInt(VLEN.W)
    val vs1_preg_idx = UInt(8.W)
    val vs2_preg_idx = UInt(8.W)
    val old_vd = UInt(VLEN.W)
    val old_vd_preg_idx = UInt(8.W)
    val vd_preg_idx = UInt(8.W)

    val uop_idx = UInt(3.W)
    val mask = UInt(VLEN.W)
    val vm = Bool()
    val ta = Bool()
    val ma = Bool()
    val vstart = UInt(7.W)
    val vl = UInt(8.W)
    val lmul_4 = Bool()

    val uop_vld = Bool()
    val uop_flush_vld = Bool()
    val rob_idx = UInt(9.W)
}

class FsmVrfIO extends VPermBundle {
    val block_fsm_rd_vld = Input(Bool())
    val fsm_rd_vld = Output(Vec(4, Bool()))
    val fsm_rd_preg_idx = Output(Vec(4, UInt(8.W)))
    val fsm_rd_data = Input(Vec(4, UInt(VLEN.W)))

    val fsm_wb_vld = Output(Bool())
    val fsm_wb_preg_idx = Output(UInt(8.W))
    val fsm_wb_data = Output(UInt(VLEN.W))
}

class FsmFlushIO extends VPermBundle {
    val fsm_busy  = Output(Bool())
    val fsm_rob_idx = Output(UInt(9.W))
    val fsm_lmul_4 = Output(Bool())

    val br_flush_vld = Input(Bool())
    val br_flush_rob_idx = Input(UInt(9.W))
    val rob_flush_vld = Input(Bool())
    val rob_commit_vld = Input(Bool())
    val rob_commit_rob_idx = Input(UInt(9.W))
}

// vperm_fsm io
class VPermFsmIO extends VPermBundle {
    val viq = Vec(2, Input(new ViqCtrl))
    val vrf = new FsmVrfIO
    val flush = new FsmFlushIO
}

class Vecinfo extends VPermBundle {
    val sew = UInt(2.W)
    val lmul = UInt(4.W)
    val elem_num_pow = UInt(3.W)

    val vstart = UInt(7.W)
    val vl = UInt(8.W)
    val vm = Bool()
    val ta = Bool()
    val ma = Bool()

    val lmul_4 = Bool()
    val opcode = UInt(3.W)
}

class Uopinfo extends VPermBundle {
    val vs1_preg_idx = UInt(8.W)
    val vs2_preg_idx = UInt(8.W)
    val old_vd_preg_idx = UInt(8.W)
    val vd_preg_idx = UInt(8.W)

    val rob_idx = UInt(9.W)
}

class Execinfo extends VPermBundle {
    val valid = Bool()
    val write_vrf = Bool()
    val write_temp = Bool()
    val uop_idx = UInt(3.W)

    // gather
    val first_op = Bool()
    val table_valid_hi = Bool()
    val table_idx_hi = UInt(3.W)
    val table_idx_lo = UInt(3.W)
}

object VPermFsmType {
    def VPermFsmTypeWidth: Int = 3

    def vslideup     = "b000".U(VPermFsmTypeWidth.W) // Slideup
    def vslidedown   = "b001".U(VPermFsmTypeWidth.W) // Slidedown
    def vrgathervv   = "b010".U(VPermFsmTypeWidth.W) // Register Gather
    def vrgathervxvi = "b011".U(VPermFsmTypeWidth.W) // Register Gather, index is from x[rs1][XLEN-1:0]/uimm[4:0]
    def vrgather16   = "b100".U(VPermFsmTypeWidth.W) // Register Gather, EEW of index is 16
    def vcompress    = "b101".U(VPermFsmTypeWidth.W) // Compress
}