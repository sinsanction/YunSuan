package yunsuan.vector

import chisel3._
import chisel3.util._
import yunsuan.util._
import yunsuan.vector.vpermutil._
import yunsuan.vector.vpermfsm._
import yunsuan.VectorElementFormat


class VPermFsmTop extends VPermModule {
    val io = IO(new VPermFsmIO)

    // --------------------------------
    // stage 01 recv_preg_idx
    // --------------------------------

    val viq0_valid = io.viq(0).uop_vld && !io.viq(0).uop_flush_vld
    val viq1_valid = io.viq(1).uop_vld && !io.viq(1).uop_flush_vld
    val viq_valid = viq0_valid || viq1_valid

    // fsm state
    val idle :: recv_preg :: working :: end :: Nil = Enum(4)
    val state = RegInit(idle)
    val state_next = WireInit(state)
    val uop_recv = RegInit(VecInit(Seq.fill(8)(false.B)))
    val uop_recv_next = WireInit(uop_recv)
    val is_lmul_4 = Wire(Bool())
    state := state_next
    uop_recv := uop_recv_next

    switch(state) {
        is( idle ) {
            when( viq_valid ) {
                state_next := recv_preg
            }
        }
        is( recv_preg ) {
            when( (uop_recv_next.asUInt === "b11111111".U(8.W) && !is_lmul_4) || (uop_recv_next.asUInt === "b00001111".U(8.W) && is_lmul_4) ) {
                state_next := working
            }
        }
    }


    // global reg
    val vecinfo = Reg(new Vecinfo)
    val uopinfo = Reg(Vec(8, new Uopinfo))
    val v0_mask = Reg(UInt(VLEN.W))

    val elem_num_pow_0 = LookupTree(io.viq(0).sew, VFormat.VFormatTable.map(p => (p._1, p._2._3)))
    val elem_num_pow_1 = LookupTree(io.viq(1).sew, VFormat.VFormatTable.map(p => (p._1, p._2._3)))

    when(state === idle && viq_valid) {
        vecinfo.sew := Mux(viq0_valid, io.viq(0).sew, io.viq(1).sew)
        vecinfo.lmul := Mux(Mux(viq0_valid, io.viq(0).lmul_4, io.viq(1).lmul_4), 4.U, 8.U)
        vecinfo.elem_num_pow := Mux(viq0_valid, elem_num_pow_0, elem_num_pow_1)

        vecinfo.vstart := Mux(viq0_valid, io.viq(0).vstart, io.viq(1).vstart)
        vecinfo.vl := Mux(viq0_valid, io.viq(0).vl, io.viq(1).vl)
        vecinfo.vm := Mux(viq0_valid, io.viq(0).vm, io.viq(1).vm)
        vecinfo.ta := Mux(viq0_valid, io.viq(0).ta, io.viq(1).ta)
        vecinfo.ma := Mux(viq0_valid, io.viq(0).ma, io.viq(1).ma)
        vecinfo.lmul_4 := Mux(viq0_valid, io.viq(0).lmul_4, io.viq(1).lmul_4)
        vecinfo.opcode := Mux(viq0_valid, io.viq(0).opcode, io.viq(1).opcode)
    }

    when((state === idle || state === recv_preg) && viq0_valid) {
        uopinfo(io.viq(0).uop_idx).vs1_preg_idx := io.viq(0).vs1_preg_idx
        uopinfo(io.viq(0).uop_idx).vs2_preg_idx := io.viq(0).vs2_preg_idx
        uopinfo(io.viq(0).uop_idx).old_vd_preg_idx := io.viq(0).old_vd_preg_idx
        uopinfo(io.viq(0).uop_idx).vd_preg_idx := io.viq(0).vd_preg_idx
        uopinfo(io.viq(0).uop_idx).rob_idx := io.viq(0).rob_idx
        uop_recv_next(io.viq(0).uop_idx) := true.B
    }
    when((state === idle || state === recv_preg) && viq1_valid) {
        uopinfo(io.viq(1).uop_idx).vs1_preg_idx := io.viq(1).vs1_preg_idx
        uopinfo(io.viq(1).uop_idx).vs2_preg_idx := io.viq(1).vs2_preg_idx
        uopinfo(io.viq(1).uop_idx).old_vd_preg_idx := io.viq(1).old_vd_preg_idx
        uopinfo(io.viq(1).uop_idx).vd_preg_idx := io.viq(1).vd_preg_idx
        uopinfo(io.viq(1).uop_idx).rob_idx := io.viq(1).rob_idx
        uop_recv_next(io.viq(1).uop_idx) := true.B
    }

    val viq0_is_first_uop = (io.viq(0).uop_idx === 0.U) && viq0_valid
    val viq1_is_first_uop = (io.viq(1).uop_idx === 0.U) && viq1_valid
    val is_first_uop = viq0_is_first_uop || viq1_is_first_uop
    when((state === idle || state === recv_preg) && viq0_valid && viq0_is_first_uop) {
        v0_mask := Mux(io.viq(0).opcode === VPermFsmType.vcompress, io.viq(0).vs1, io.viq(0).mask)
    }
    when((state === idle || state === recv_preg) && viq1_valid && viq1_is_first_uop) {
        v0_mask := Mux(io.viq(1).opcode === VPermFsmType.vcompress, io.viq(1).vs1, io.viq(1).mask)
    }

    is_lmul_4 := vecinfo.lmul_4

    // slide reg
    val slide = Reg(UInt(4.W))
    val src_idx = Reg(Vec(7, UInt(3.W))) // slideup : src_idx_hi[1-7] ; slidedown : src_idx_lo[0-6]
    val exceed_limit = Reg(Vec(8, Bool())) // slideup : exceed_limit_hi[0-7] ; slidedown : exceed_limit_lo[0-7]

    val rs1 = Mux(viq0_is_first_uop, io.viq(0).vs1(XLEN-1, 0), io.viq(1).vs1(XLEN-1, 0))
    val sew_first_uop = Mux(viq0_is_first_uop, io.viq(0).sew, io.viq(1).sew)
    val rs1_div_elem_num = rs1 >> Mux(viq0_is_first_uop, elem_num_pow_0, elem_num_pow_1)
    val lmul_first_uop = Mux(Mux(viq0_is_first_uop, io.viq(0).lmul_4, io.viq(1).lmul_4), 4.U, 8.U)
    val opcode_first_uop = Mux(viq0_is_first_uop, io.viq(0).opcode, io.viq(1).opcode)

    when((state === idle || state === recv_preg) && is_first_uop) {
        slide := LookupTree(sew_first_uop, List(
            VectorElementFormat.b -> rs1(3, 0),
            VectorElementFormat.h -> ZeroExt(rs1(2, 0), 4),
            VectorElementFormat.w -> ZeroExt(rs1(1, 0), 4),
            VectorElementFormat.d -> ZeroExt(rs1(0), 4)
        ))
    }

    val sldup_src_idx = Wire(Vec(7, UInt(3.W))) // src_idx_hi[1-7]
    val sldup_exceed_limit = Wire(Vec(8, Bool())) // exceed_limit_hi[0-7]

    for(i <- 0 until 7) { sldup_src_idx(i) := (i+1).U - rs1_div_elem_num }
    for(i <- 0 until 8) { sldup_exceed_limit(i) := rs1_div_elem_num >= (i+1).U }

    val slddw_src_idx = Wire(Vec(7, UInt(3.W))) // src_idx_lo[0-6]
    val slddw_exceed_limit = Wire(Vec(8, Bool())) // exceed_limit_lo[0-7]

    for(i <- 0 until 7) { slddw_src_idx(i) := rs1_div_elem_num + i.U }
    for(i <- 0 until 8) { slddw_exceed_limit(i) := rs1_div_elem_num >= (lmul_first_uop - i.U) }

    when((state === idle || state === recv_preg) && is_first_uop) {
        for(i <- 0 until 7) { src_idx(i) := Mux(opcode_first_uop === VPermFsmType.vslideup, sldup_src_idx(i), slddw_src_idx(i)) }
        for(i <- 0 until 8) { exceed_limit(i) := Mux(opcode_first_uop === VPermFsmType.vslideup, sldup_exceed_limit(i), slddw_exceed_limit(i)) }
    }


    io.vfr.fsm_wb_preg_idx := 0.U
    io.vfr.fsm_wb_vld := false.B

    io.vfr.fsm_rd_vld := DontCare
    io.vfr.fsm_rd_preg_idx := DontCare

    io.flush.fsm_busy := DontCare
    io.flush.fsm_rob_idx := 0.U
    io.flush.fsm_lmul_4 := true.B

    // --------------------------------
    // stage 02 working
    // --------------------------------
    // common
    val uop_idx = RegInit(0.U(3.W))
    val sew = vecinfo.sew
    val lmul = vecinfo.lmul
    val elem_num_pow = vecinfo.elem_num_pow
    val vstart = vecinfo.vstart
    val vl = vecinfo.vl
    val vm = vecinfo.vm
    val ta = vecinfo.ta
    val ma = vecinfo.ma
    val lmul_4 = vecinfo.lmul_4
    val opcode = vecinfo.opcode
    val mask_start_idx = uop_idx << elem_num_pow

    when(state === recv_preg && state_next === working) {
        uop_idx := 0.U
    }.elsewhen(state === working) {
        uop_idx := uop_idx + 1.U
    }

    // slide
    val sldup_src_idx_lo = Wire(Vec(8, UInt(3.W)))
    val sldup_src_idx_hi = Wire(Vec(8, UInt(3.W)))
    val sldup_exceed_limit_lo = Wire(Vec(8, Bool()))
    val sldup_exceed_limit_hi = Wire(Vec(8, Bool()))

    sldup_src_idx_hi(0) := 0.U
    for(i <- 1 until 8) { sldup_src_idx_hi(i) := src_idx(i-1) }
    sldup_src_idx_lo(0) := 0.U
    for(i <- 1 until 8) { sldup_src_idx_lo(i) := sldup_src_idx_hi(i-1) }

    for(i <- 0 until 8) { sldup_exceed_limit_hi(i) := exceed_limit(i) }
    sldup_exceed_limit_lo(0) := true.B
    for(i <- 1 until 8) { sldup_exceed_limit_lo(i) := sldup_exceed_limit_hi(i-1) }

    val slddw_src_idx_lo = Wire(Vec(8, UInt(3.W)))
    val slddw_src_idx_hi = Wire(Vec(8, UInt(3.W)))
    val slddw_exceed_limit_lo = Wire(Vec(8, Bool()))
    val slddw_exceed_limit_hi = Wire(Vec(8, Bool()))

    slddw_src_idx_lo(7) := 7.U
    for(i <- 0 until 7) { slddw_src_idx_lo(i) := src_idx(i) }
    slddw_src_idx_hi(7) := 7.U
    for(i <- 0 until 7) { slddw_src_idx_hi(i) := slddw_src_idx_lo(i+1) }
    when(lmul_4) {
        slddw_src_idx_lo(3) := 3.U
        slddw_src_idx_hi(3) := 3.U
    }

    for(i <- 0 until 8) { slddw_exceed_limit_lo(i) := exceed_limit(i) }
    slddw_exceed_limit_hi(7) := true.B
    for(i <- 0 until 7) { slddw_exceed_limit_hi(i) := slddw_exceed_limit_lo(i+1) }

    // slide up
    val vslideup_fsm = Module(new SlideUpFsmModule)
    val vslideup_in = Wire(new SlideFsmIO)

    vslideup_in.mask_start_idx := mask_start_idx
    vslideup_in.slide := slide
    vslideup_in.sew := sew
    vslideup_in.vstart := vstart
    vslideup_in.vl := vl
    vslideup_in.vm := vm
    vslideup_in.ta := ta
    vslideup_in.ma := ma

    vslideup_in.mask := (v0_mask >> mask_start_idx)(15, 0)
    vslideup_in.exceed_limit_hi := sldup_exceed_limit_hi(uop_idx)
    vslideup_in.exceed_limit_lo := sldup_exceed_limit_lo(uop_idx)
    vslideup_in.src_data_hi := io.vfr.fsm_rd_data(2)
    vslideup_in.src_data_lo := io.vfr.fsm_rd_data(1)
    vslideup_in.prev_data := io.vfr.fsm_rd_data(3)

    vslideup_fsm.io.slidefsm := vslideup_in

    // slide down
    val vslidedown_fsm = Module(new SlideDownFsmModule)
    val vslidedown_in = Wire(new SlideFsmIO)

    vslidedown_in.mask_start_idx := mask_start_idx
    vslidedown_in.slide := slide
    vslidedown_in.sew := sew
    vslidedown_in.vstart := vstart
    vslidedown_in.vl := vl
    vslidedown_in.vm := vm
    vslidedown_in.ta := ta
    vslidedown_in.ma := ma

    vslidedown_in.mask := (v0_mask >> mask_start_idx)(15, 0)
    vslidedown_in.exceed_limit_hi := slddw_exceed_limit_hi(uop_idx)
    vslidedown_in.exceed_limit_lo := slddw_exceed_limit_lo(uop_idx)
    vslidedown_in.src_data_hi := io.vfr.fsm_rd_data(2)
    vslidedown_in.src_data_lo := io.vfr.fsm_rd_data(1)
    vslidedown_in.prev_data := io.vfr.fsm_rd_data(3)

    vslidedown_fsm.io.slidefsm := vslidedown_in

    io.vfr.fsm_wb_data := vslideup_fsm.io.res_data
}
