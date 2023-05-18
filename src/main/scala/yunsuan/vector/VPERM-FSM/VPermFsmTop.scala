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
    val idle :: recv_preg :: working :: ending :: Nil = Enum(4)
    val state = RegInit(idle)
    val state_next = WireInit(state)
    val uop_recv = RegInit(0.U(8.W))
    val uop_recv_next = WireInit(uop_recv)
    val is_uop_8 = Wire(Bool())
    val working_done = Wire(Bool())
    val ending_done = Wire(Bool())

    val flush_valid = io.flush.br_flush_vld || io.flush.rob_flush_vld
    when(flush_valid) {
        state := idle
        uop_recv := 0.U(8.W)
    }.otherwise {
        state := state_next
        uop_recv := uop_recv_next
    }

    switch(state) {
        is( idle ) {
            when( viq_valid ) {
                state_next := recv_preg
            }
        }
        is( recv_preg ) {
            when( (uop_recv_next.asUInt === "b11111111".U(8.W) && is_uop_8) || (uop_recv_next.asUInt === "b00001111".U(8.W) && !is_uop_8) ) {
                state_next := working
            }
        }
        is( working ) {
            when( working_done ) {
                state_next := ending
            }
        }
        is( ending ) {
            when( ending_done ) {
                state_next := idle
            }
        }
    }

    when((state === idle || state === recv_preg) && viq0_valid && viq1_valid) {
        uop_recv_next := uop_recv | UIntToOH(io.viq(0).uop_idx, 8) | UIntToOH(io.viq(1).uop_idx, 8)
    }.elsewhen((state === idle || state === recv_preg) && viq0_valid) {
        uop_recv_next := uop_recv | UIntToOH(io.viq(0).uop_idx, 8)
    }.elsewhen((state === idle || state === recv_preg) && viq1_valid) {
        uop_recv_next := uop_recv | UIntToOH(io.viq(1).uop_idx, 8)
    }.elsewhen(state === working) {
        uop_recv_next := 0.U(8.W)
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
    }
    when((state === idle || state === recv_preg) && viq1_valid) {
        uopinfo(io.viq(1).uop_idx).vs1_preg_idx := io.viq(1).vs1_preg_idx
        uopinfo(io.viq(1).uop_idx).vs2_preg_idx := io.viq(1).vs2_preg_idx
        uopinfo(io.viq(1).uop_idx).old_vd_preg_idx := io.viq(1).old_vd_preg_idx
        uopinfo(io.viq(1).uop_idx).vd_preg_idx := io.viq(1).vd_preg_idx
        uopinfo(io.viq(1).uop_idx).rob_idx := io.viq(1).rob_idx
    }

    val viq0_is_first_uop = (io.viq(0).uop_idx === 0.U) && viq0_valid
    val viq1_is_first_uop = (io.viq(1).uop_idx === 0.U) && viq1_valid
    val is_first_uop = viq0_is_first_uop || viq1_is_first_uop
    when((state === idle || state === recv_preg) && viq0_is_first_uop) {
        v0_mask := Mux(io.viq(0).opcode === VPermFsmType.vcompress, io.viq(0).vs1, io.viq(0).mask)
    }.elsewhen((state === idle || state === recv_preg) && viq1_is_first_uop) {
        v0_mask := Mux(io.viq(1).opcode === VPermFsmType.vcompress, io.viq(1).vs1, io.viq(1).mask)
    }

    is_uop_8 := !vecinfo.lmul_4 || vecinfo.lmul_4 && (vecinfo.opcode === VPermFsmType.vrgather16) && (vecinfo.sew === VectorElementFormat.b)

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

    // gather reg
    val idx_rs1 = Reg(UInt(XLEN.W))
    val access_table = Reg(Vec(8, UInt(8.W)))

    when((state === idle || state === recv_preg) && is_first_uop) {
        idx_rs1 := rs1
    }

    val lmul_0 = Mux(io.viq(0).lmul_4, 4.U, 8.U)
    val lmul_1 = Mux(io.viq(1).lmul_4, 4.U, 8.U)

    val access_table_viq_0 = Module(new AccessTableModule)
    val access_table_viq_1 = Module(new AccessTableModule)
    access_table_viq_0.io.sew          := io.viq(0).sew
    access_table_viq_0.io.elem_num_pow := elem_num_pow_0
    access_table_viq_0.io.lmul         := lmul_0
    access_table_viq_0.io.opcode       := io.viq(0).opcode
    access_table_viq_0.io.vs1_data     := io.viq(0).vs1
    access_table_viq_1.io.sew          := io.viq(1).sew
    access_table_viq_1.io.elem_num_pow := elem_num_pow_1
    access_table_viq_1.io.lmul         := lmul_1
    access_table_viq_1.io.opcode       := io.viq(1).opcode
    access_table_viq_1.io.vs1_data     := io.viq(1).vs1

    when((state === idle || state === recv_preg) && viq0_valid) {
        access_table(io.viq(0).uop_idx) := access_table_viq_0.io.res_data
    }
    when((state === idle || state === recv_preg) && viq1_valid) {
        access_table(io.viq(1).uop_idx) := access_table_viq_1.io.res_data
    }

    when(io.viq(0).lmul_4) {
        assert(access_table_viq_0.io.res_data <= 15.U, "access table 0 has valid bit over 4 when lmul is 4")
    }
    when(io.viq(1).lmul_4) {
        assert(access_table_viq_1.io.res_data <= 15.U, "access table 1 has valid bit over 4 when lmul is 4")
    }

    // gather16
    val access_table_16 = Reg(Vec(8, UInt(8.W)))

    val access_table_16_viq_0 = Module(new AccessTable16Module)
    val access_table_16_viq_1 = Module(new AccessTable16Module)
    access_table_16_viq_0.io.sew          := io.viq(0).sew
    access_table_16_viq_0.io.elem_num_pow := elem_num_pow_0
    access_table_16_viq_0.io.lmul         := lmul_0
    access_table_16_viq_0.io.uop_idx      := io.viq(0).uop_idx
    access_table_16_viq_0.io.vs1_data     := io.viq(0).vs1
    access_table_16_viq_1.io.sew          := io.viq(1).sew
    access_table_16_viq_1.io.elem_num_pow := elem_num_pow_1
    access_table_16_viq_1.io.lmul         := lmul_1
    access_table_16_viq_1.io.uop_idx      := io.viq(1).uop_idx
    access_table_16_viq_1.io.vs1_data     := io.viq(1).vs1

    when((state === idle || state === recv_preg) && viq0_valid) {
        access_table_16(io.viq(0).uop_idx) := access_table_16_viq_0.io.res_data
    }
    when((state === idle || state === recv_preg) && viq1_valid) {
        access_table_16(io.viq(1).uop_idx) := access_table_16_viq_1.io.res_data
    }

    // --------------------------------
    // stage 02 read_vrf
    // --------------------------------
    // common
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

    val execinfo = Wire(new Execinfo)
    val uop_idx_vrf = Reg(UInt(3.W))
    val uop_idx_update = Wire(Bool())
    val vrf_request_succ = Wire(Bool())
    val first_op = RegInit(false.B)

    when(state === recv_preg && state_next === working) {
        uop_idx_vrf := 0.U
    }.elsewhen(state === working && uop_idx_update) {
        uop_idx_vrf := uop_idx_vrf + 1.U
    }

    when(state === recv_preg && state_next === working) {
        first_op := true.B
    }.elsewhen(state === working) {
        when(vrf_request_succ && !uop_idx_update) {
            first_op := false.B
        }.elsewhen(uop_idx_update) {
            first_op := true.B
        }
    }.otherwise {
        first_op := false.B
    }

    working_done := Mux(is_uop_8, (uop_idx_vrf === 7.U), (uop_idx_vrf === 3.U)) && uop_idx_update

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

    // gather
    val current_access_lo = SetOnlyFirst(access_table(uop_idx_vrf))
    val current_access_hi = SetOnlyFirst(~current_access_lo & access_table(uop_idx_vrf))
    assert(PopCount(current_access_lo) <= 1.U, "current_access_lo is not one-hot")
    assert(PopCount(current_access_hi) <= 1.U, "current_access_hi is not one-hot")
    val access_table_next = ~(current_access_lo | current_access_hi) & access_table(uop_idx_vrf)
    when(state === working && vrf_request_succ) {
        access_table(uop_idx_vrf) := access_table_next
    }

    val current_is_last = PopCount(access_table(uop_idx_vrf)) <= 2.U
    val current_table_valid_hi = PopCount(access_table(uop_idx_vrf)) >= 2.U
    val current_table_valid_lo = PopCount(access_table(uop_idx_vrf)) >= 1.U

    val current_table_idx_hi = (0 until 8).map{ i => Fill(3, current_access_hi(i)) & i.U(3.W) }.reduce(_ | _)
    val current_table_idx_lo = (0 until 8).map{ i => Fill(3, current_access_lo(i)) & i.U(3.W) }.reduce(_ | _)
    val current_table_preg_idx_hi = (0 until 8).map{ i => Fill(8, current_access_hi(i)) & uopinfo(i).vs2_preg_idx }.reduce(_ | _)
    val current_table_preg_idx_lo = (0 until 8).map{ i => Fill(8, current_access_lo(i)) & uopinfo(i).vs2_preg_idx }.reduce(_ | _)

    // gather16
    val is_lmul4_sew8 = lmul_4 && (opcode === VPermFsmType.vrgather16) && (sew === VectorElementFormat.b)
    val current_access_table = access_table_16(uop_idx_vrf)
    val current_access_16_lo = SetOnlyFirst(current_access_table)
    val current_access_16_hi = SetOnlyFirst(~current_access_16_lo & current_access_table)
    assert(PopCount(current_access_16_lo) <= 1.U, "current_access_16_lo is not one-hot")
    assert(PopCount(current_access_16_hi) <= 1.U, "current_access_16_hi is not one-hot")
    val access_table_16_next = ~(current_access_16_lo | current_access_16_hi) & current_access_table
    when(state === working && vrf_request_succ) {
        access_table_16(uop_idx_vrf) := access_table_16_next
    }

    val current_16_is_last = PopCount(current_access_table) <= 2.U
    val current_16_table_valid_hi = PopCount(current_access_table) >= 2.U
    val current_16_table_valid_lo = PopCount(current_access_table) >= 1.U

    val current_16_table_idx_hi = (0 until 8).map{ i => Fill(3, current_access_16_hi(i)) & i.U(3.W) }.reduce(_ | _)
    val current_16_table_idx_lo = (0 until 8).map{ i => Fill(3, current_access_16_lo(i)) & i.U(3.W) }.reduce(_ | _)
    val current_16_table_preg_idx_hi = (0 until 8).map{ i => Fill(8, current_access_16_hi(i)) & uopinfo(i).vs2_preg_idx }.reduce(_ | _)
    val current_16_table_preg_idx_lo = (0 until 8).map{ i => Fill(8, current_access_16_lo(i)) & uopinfo(i).vs2_preg_idx }.reduce(_ | _)

    val lmul4_sew8_table_preg_idx_hi = LookupTreeDefault(current_access_16_hi(3,0), 0.U(8.W), List(
        "b0001".U -> uopinfo(0).vs2_preg_idx,
        "b0010".U -> uopinfo(2).vs2_preg_idx,
        "b0100".U -> uopinfo(4).vs2_preg_idx,
        "b1000".U -> uopinfo(6).vs2_preg_idx
    ))
    val lmul4_sew8_table_preg_idx_lo = LookupTreeDefault(current_access_16_lo(3,0), 0.U(8.W), List(
        "b0001".U -> uopinfo(0).vs2_preg_idx,
        "b0010".U -> uopinfo(2).vs2_preg_idx,
        "b0100".U -> uopinfo(4).vs2_preg_idx,
        "b1000".U -> uopinfo(6).vs2_preg_idx
    ))
    val half_op = uop_idx_vrf(0).asBool

    // compress
    val src_reg_idx = Reg(UInt(3.W))
    val prev_ones_sum = Reg(UInt(8.W))

    val select_mask = (v0_mask >> (src_reg_idx << elem_num_pow))(15, 0)
    val vs1_mask = LookupTree(sew, List(
        VectorElementFormat.b -> select_mask,
        VectorElementFormat.h -> ZeroExt(select_mask(7,0), 16),
        VectorElementFormat.w -> ZeroExt(select_mask(3,0), 16),
        VectorElementFormat.d -> ZeroExt(select_mask(1,0), 16)
    ))
    val curr_ones_sum = prev_ones_sum + PopCount(vs1_mask)

    val current_vd_finished = Mux(lmul_4, (src_reg_idx === 3.U), (src_reg_idx === 7.U)) || (curr_ones_sum >= ((uop_idx_vrf +& 1.U) << elem_num_pow))

    when(state === recv_preg && state_next === working) {
        src_reg_idx := 0.U
        prev_ones_sum := 0.U
    }.elsewhen((state === working) && vrf_request_succ) {
        when(!current_vd_finished) {
            prev_ones_sum := curr_ones_sum
        }
        when(Mux(lmul_4, (src_reg_idx =/= 3.U), (src_reg_idx =/= 7.U)) && (curr_ones_sum <= ((uop_idx_vrf +& 1.U) << elem_num_pow))) {
            src_reg_idx := src_reg_idx + 1.U
        }
    }

    // to vrf
    val rd_preg_idx = Wire(Vec(4, UInt(8.W)))
    val rd_vld = Wire(Vec(4, Bool()))
    val wb_vld = Wire(Bool())
    vrf_request_succ := !io.vrf.block_fsm_rd_vld

    rd_preg_idx := 0.U.asTypeOf(Vec(4, UInt(8.W)))
    rd_vld := 0.U.asTypeOf(Vec(4, Bool()))
    wb_vld := false.B
    execinfo := 0.U.asTypeOf(new Execinfo)
    uop_idx_update := false.B

    rd_preg_idx(3) := uopinfo(uop_idx_vrf).old_vd_preg_idx
    execinfo.valid := vrf_request_succ && (state === working)
    execinfo.uop_idx := uop_idx_vrf

    when(opcode === VPermFsmType.vslideup) {
        rd_preg_idx(1) := uopinfo(sldup_src_idx_lo(uop_idx_vrf)).vs2_preg_idx
        rd_preg_idx(2) := uopinfo(sldup_src_idx_hi(uop_idx_vrf)).vs2_preg_idx
        rd_vld(1) := !sldup_exceed_limit_lo(uop_idx_vrf)
        rd_vld(2) := !sldup_exceed_limit_hi(uop_idx_vrf)
        rd_vld(3) := true.B
        wb_vld := true.B
        execinfo.write_vrf := true.B
        execinfo.write_temp := false.B
        uop_idx_update := vrf_request_succ
    }.elsewhen(opcode === VPermFsmType.vslidedown) {
        rd_preg_idx(1) := uopinfo(slddw_src_idx_lo(uop_idx_vrf)).vs2_preg_idx
        rd_preg_idx(2) := uopinfo(slddw_src_idx_hi(uop_idx_vrf)).vs2_preg_idx
        rd_vld(1) := !slddw_exceed_limit_lo(uop_idx_vrf)
        rd_vld(2) := !slddw_exceed_limit_hi(uop_idx_vrf)
        rd_vld(3) := true.B
        wb_vld := true.B
        execinfo.write_vrf := true.B
        execinfo.write_temp := false.B
        uop_idx_update := vrf_request_succ
    }.elsewhen(opcode === VPermFsmType.vrgathervv || opcode === VPermFsmType.vrgathervxvi) {
        rd_preg_idx(0) := uopinfo(uop_idx_vrf).vs1_preg_idx
        rd_preg_idx(1) := current_table_preg_idx_lo
        rd_preg_idx(2) := current_table_preg_idx_hi
        rd_vld(0) := opcode === VPermFsmType.vrgathervv
        rd_vld(1) := current_table_valid_lo
        rd_vld(2) := current_table_valid_hi
        rd_vld(3) := first_op
        wb_vld := current_is_last
        execinfo.write_vrf := current_is_last
        execinfo.write_temp := !current_is_last
        execinfo.first_op := first_op
        execinfo.table_valid_hi := current_table_valid_hi
        execinfo.table_idx_hi := current_table_idx_hi
        execinfo.table_idx_lo := current_table_idx_lo
        uop_idx_update := current_is_last && vrf_request_succ
    }.elsewhen(opcode === VPermFsmType.vrgather16) {
        rd_preg_idx(0) := uopinfo(uop_idx_vrf).vs1_preg_idx
        rd_preg_idx(1) := Mux(is_lmul4_sew8, lmul4_sew8_table_preg_idx_lo, current_16_table_preg_idx_lo)
        rd_preg_idx(2) := Mux(is_lmul4_sew8, lmul4_sew8_table_preg_idx_hi, current_16_table_preg_idx_hi)
        rd_vld(0) := true.B
        rd_vld(1) := current_16_table_valid_lo
        rd_vld(2) := current_16_table_valid_hi
        rd_vld(3) := Mux(is_lmul4_sew8, first_op && !half_op, first_op)
        wb_vld := Mux(is_lmul4_sew8, current_16_is_last && half_op, current_16_is_last)
        execinfo.write_vrf := Mux(is_lmul4_sew8, current_16_is_last && half_op, current_16_is_last)
        execinfo.write_temp := Mux(is_lmul4_sew8, !(current_16_is_last && half_op), !current_16_is_last)
        execinfo.first_op := first_op
        execinfo.table_valid_hi := current_16_table_valid_hi
        execinfo.table_idx_hi := current_16_table_idx_hi
        execinfo.table_idx_lo := current_16_table_idx_lo
        uop_idx_update := current_16_is_last && vrf_request_succ
    }.elsewhen(opcode === VPermFsmType.vcompress) {
        rd_preg_idx(0) := uopinfo(src_reg_idx).vs2_preg_idx
        rd_vld(0) := true.B
        rd_vld(3) := first_op
        wb_vld := current_vd_finished
        execinfo.write_vrf := current_vd_finished
        execinfo.write_temp := !current_vd_finished
        execinfo.first_op := first_op
        execinfo.pmos := prev_ones_sum
        execinfo.vs1_mask := vs1_mask
        uop_idx_update := current_vd_finished && vrf_request_succ
    }

    io.vrf.fsm_wb_vld := wb_vld && vrf_request_succ && (state === working)
    io.vrf.fsm_rd_vld(0) := rd_vld(0) && vrf_request_succ && (state === working)
    io.vrf.fsm_rd_vld(1) := rd_vld(1) && vrf_request_succ && (state === working)
    io.vrf.fsm_rd_vld(2) := rd_vld(2) && vrf_request_succ && (state === working)
    io.vrf.fsm_rd_vld(3) := rd_vld(3) && vrf_request_succ && (state === working)

    io.vrf.fsm_wb_preg_idx := uopinfo(uop_idx_vrf).vd_preg_idx
    io.vrf.fsm_rd_preg_idx(0) := rd_preg_idx(0)
    io.vrf.fsm_rd_preg_idx(1) := rd_preg_idx(1)
    io.vrf.fsm_rd_preg_idx(2) := rd_preg_idx(2)
    io.vrf.fsm_rd_preg_idx(3) := rd_preg_idx(3)

    io.flush.fsm_busy := (state =/= idle)
    io.flush.fsm_rob_idx := uopinfo(0).rob_idx
    io.flush.fsm_lmul_4 := vecinfo.lmul_4

    // --------------------------------
    // stage 03 exec
    // --------------------------------
    // common
    val execinfo_reg = DelayN(execinfo, 4)
    val exec_temp = Reg(UInt(VLEN.W))
    val uop_idx = execinfo_reg.uop_idx
    val exec_valid = execinfo_reg.valid
    val write_vrf = execinfo_reg.write_vrf
    val write_temp = execinfo_reg.write_temp
    val exec_first_op = execinfo_reg.first_op
    val mask_start_idx = uop_idx << elem_num_pow
    val mask_selected = (v0_mask >> mask_start_idx)(15, 0)

    // gather
    val table_valid_hi = execinfo_reg.table_valid_hi
    val min_hi = execinfo_reg.table_idx_hi << elem_num_pow
    val max_hi = (execinfo_reg.table_idx_hi +& 1.U) << elem_num_pow
    val min_lo = execinfo_reg.table_idx_lo << elem_num_pow
    val max_lo = (execinfo_reg.table_idx_lo +& 1.U) << elem_num_pow

    // gather16
    val half_gather16 = uop_idx(0).asBool
    val mask_start_idx_lmul4_sew8 = (uop_idx(2,1) << elem_num_pow) + Mux(half_gather16, 8.U, 0.U)
    val mask_start_idx_gather16 = Mux(is_lmul4_sew8, mask_start_idx_lmul4_sew8, mask_start_idx)
    val mask_selected_lmul4_sew8 = (v0_mask >> mask_start_idx_lmul4_sew8)(7, 0)
    val mask_selected_gather16 = Mux(is_lmul4_sew8, mask_selected_lmul4_sew8, mask_selected(7, 0))

    val idx_data = io.vrf.fsm_rd_data(0)
    val idx_data_sew32 = Mux(uop_idx(0), idx_data(127, 64), idx_data(63, 0))
    val idx_data_sew64 = LookupTree(uop_idx(1,0), List(
        "b00".U -> idx_data(31, 0),
        "b01".U -> idx_data(63, 32),
        "b10".U -> idx_data(95, 64),
        "b11".U -> idx_data(127, 96)
    ))
    val idx_data_gather16 = LookupTree(sew, List(
        VectorElementFormat.b -> idx_data,
        VectorElementFormat.h -> idx_data,
        VectorElementFormat.w -> ZeroExt(idx_data_sew32, VLEN),
        VectorElementFormat.d -> ZeroExt(idx_data_sew64, VLEN)
    ))

    // compress
    val ones_sum_base = mask_start_idx
    val pmos = execinfo_reg.pmos
    val compress_mask = execinfo_reg.vs1_mask

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

    vslideup_in.mask := mask_selected
    vslideup_in.exceed_limit_hi := sldup_exceed_limit_hi(uop_idx)
    vslideup_in.exceed_limit_lo := sldup_exceed_limit_lo(uop_idx)
    vslideup_in.src_data_hi := io.vrf.fsm_rd_data(2)
    vslideup_in.src_data_lo := io.vrf.fsm_rd_data(1)
    vslideup_in.prev_data := io.vrf.fsm_rd_data(3)

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

    vslidedown_in.mask := mask_selected
    vslidedown_in.exceed_limit_hi := slddw_exceed_limit_hi(uop_idx)
    vslidedown_in.exceed_limit_lo := slddw_exceed_limit_lo(uop_idx)
    vslidedown_in.src_data_hi := io.vrf.fsm_rd_data(2)
    vslidedown_in.src_data_lo := io.vrf.fsm_rd_data(1)
    vslidedown_in.prev_data := io.vrf.fsm_rd_data(3)

    vslidedown_fsm.io.slidefsm := vslidedown_in

    // gather
    val vrgather_fsm = Module(new GatherFsmModule)
    val vrgather_in = Wire(new GatherFsmIO)

    vrgather_in.mask_start_idx := mask_start_idx
    vrgather_in.sew := sew
    vrgather_in.vstart := vstart
    vrgather_in.vl := vl
    vrgather_in.vm := vm
    vrgather_in.ta := ta
    vrgather_in.ma := ma

    vrgather_in.mask := mask_selected
    vrgather_in.table_valid_hi := table_valid_hi
    vrgather_in.first_op := exec_first_op
    vrgather_in.min_hi := min_hi
    vrgather_in.max_hi := max_hi
    vrgather_in.min_lo := min_lo
    vrgather_in.max_lo := max_lo
    vrgather_in.idx_rs1 := idx_rs1
    vrgather_in.idx_is_rs1 := (opcode === VPermFsmType.vrgathervxvi)

    vrgather_in.idx_data := io.vrf.fsm_rd_data(0)
    vrgather_in.table_data_hi := io.vrf.fsm_rd_data(2)
    vrgather_in.table_data_lo := io.vrf.fsm_rd_data(1)
    vrgather_in.prev_data := io.vrf.fsm_rd_data(3)
    vrgather_in.temp_data := exec_temp

    vrgather_fsm.io.gather := vrgather_in

    // gather16
    val vrgather16_fsm = Module(new Gather16FsmModule)
    val vrgather16_in = Wire(new Gather16FsmIO)

    vrgather16_in.mask_start_idx := mask_start_idx_gather16
    vrgather16_in.sew := sew
    vrgather16_in.vstart := vstart
    vrgather16_in.vl := vl
    vrgather16_in.vm := vm
    vrgather16_in.ta := ta
    vrgather16_in.ma := ma

    vrgather16_in.mask := mask_selected_gather16
    vrgather16_in.table_valid_hi := table_valid_hi
    vrgather16_in.first_op := exec_first_op
    vrgather16_in.half_op := half_gather16
    vrgather16_in.min_hi := min_hi
    vrgather16_in.max_hi := max_hi
    vrgather16_in.min_lo := min_lo
    vrgather16_in.max_lo := max_lo

    vrgather16_in.idx_data := idx_data_gather16
    vrgather16_in.table_data_hi := io.vrf.fsm_rd_data(2)
    vrgather16_in.table_data_lo := io.vrf.fsm_rd_data(1)
    vrgather16_in.prev_data := io.vrf.fsm_rd_data(3)
    vrgather16_in.temp_data := exec_temp

    vrgather16_fsm.io.gather := vrgather16_in

    // compress
    val vcompress_fsm = Module(new CompressFsmModule)
    val vcompress_in = Wire(new CompressFsmIO)

    vcompress_in.os_base := ones_sum_base
    vcompress_in.pmos := pmos
    vcompress_in.mask := compress_mask
    vcompress_in.first_op := exec_first_op

    vcompress_in.sew := sew
    vcompress_in.vl := vl
    vcompress_in.ta := ta

    vcompress_in.src_data := io.vrf.fsm_rd_data(0)
    vcompress_in.prev_data := io.vrf.fsm_rd_data(3)
    vcompress_in.temp_data := exec_temp

    vcompress_fsm.io.compress := vcompress_in

    // write vrf
    val execinfo_reg_w = RegNext(execinfo_reg)
    val wb_data = Reg(UInt(VLEN.W))

    ending_done := Mux(is_uop_8, (execinfo_reg_w.uop_idx === 7.U), (execinfo_reg_w.uop_idx === 3.U)) && execinfo_reg_w.valid && execinfo_reg_w.write_vrf

    when((state === working || state === ending) && execinfo_reg_w.valid && execinfo_reg_w.write_temp) {
        exec_temp := Mux(opcode === VPermFsmType.vcompress, vcompress_fsm.io.res_data, Mux(opcode === VPermFsmType.vrgather16, vrgather16_fsm.io.res_data, vrgather_fsm.io.res_data))
    }

    val res_data = LookupTreeDefault(opcode, 0.U(VLEN.W), List(
        VPermFsmType.vslideup     -> vslideup_fsm.io.res_data,
        VPermFsmType.vslidedown   -> vslidedown_fsm.io.res_data,
        VPermFsmType.vrgathervv   -> vrgather_fsm.io.res_data,
        VPermFsmType.vrgathervxvi -> vrgather_fsm.io.res_data,
        VPermFsmType.vrgather16   -> vrgather16_fsm.io.res_data,
        VPermFsmType.vcompress    -> vcompress_fsm.io.res_data,
    ))
    when((state === working || state === ending) && execinfo_reg_w.valid && execinfo_reg_w.write_vrf) {
        wb_data := res_data
    }
    io.vrf.fsm_wb_data := wb_data
}
