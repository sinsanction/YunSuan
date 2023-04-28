package yunsuan.vector

import chisel3._
import chisel3.util._
import yunsuan.util._
import yunsuan.vector.vpermutil._
import yunsuan.vector.vpermfsm._
import yunsuan.VectorElementFormat


class GatherFsmIO extends VPermBundle {
    val mask_start_idx  = UInt(7.W)

    val sew             = UInt(2.W)
    val vstart          = UInt(7.W)
    val vl              = UInt(8.W)
    val vm              = Bool()
    val ta              = Bool()
    val ma              = Bool()

    val mask            = UInt(16.W)
    val table_valid_hi  = Bool()
    val first_op        = Bool()
    val min_hi          = UInt(7.W)
    val max_hi          = UInt(8.W)
    val min_lo          = UInt(7.W)
    val max_lo          = UInt(8.W)

    val idx_data        = UInt(VLEN.W)
    val idx_rs1         = UInt(XLEN.W)
    val idx_is_rs1      = Bool()

    val table_data_hi   = UInt(VLEN.W)
    val table_data_lo   = UInt(VLEN.W)
    val prev_data       = UInt(VLEN.W)
    val temp_data       = UInt(VLEN.W)
}

// table_lookup_fsm
class TableLookupFsm(n: Int) extends VPermModule {
    val io = IO(new VPermBundle() {
        val gather   = Input(new GatherFsmIO)
        val res_data = Output(UInt(VLEN.W))
    })

    // stage-0
    val idx = Wire(Vec(n, UInt(log2Up(n).W)))
    val res_keep_old_vd = Wire(Vec(n, Bool()))
    val res_agnostic = Wire(Vec(n, Bool()))
    val idx_valid_hi = Wire(Vec(n, Bool()))
    val idx_valid_lo = Wire(Vec(n, Bool()))
    val table_data_hi_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val table_data_lo_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val idx_data_vec  = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        val elements_idx = io.gather.mask_start_idx + i.U
        idx_data_vec(i) := io.gather.idx_data((VLEN/n)*(i+1)-1, (VLEN/n)*i)
        val index = Mux(io.gather.idx_is_rs1, io.gather.idx_rs1, idx_data_vec(i))

        res_keep_old_vd(i) := RegNext((!io.gather.vm && !io.gather.mask(i).asBool && !io.gather.ma) || (elements_idx < io.gather.vstart) || ((elements_idx >= io.gather.vl) && !io.gather.ta) || (io.gather.vstart >= io.gather.vl))
        res_agnostic(i) := RegNext(((elements_idx >= io.gather.vl) && io.gather.ta) || (!io.gather.vm && !io.gather.mask(i).asBool && io.gather.ma))
        idx_valid_lo(i) := RegNext((io.gather.min_lo <= index) && (index < io.gather.max_lo))
        idx_valid_hi(i) := RegNext((io.gather.min_hi <= index) && (index < io.gather.max_hi) && io.gather.table_valid_hi)

        table_data_hi_vec(i) := RegNext(io.gather.table_data_hi((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        table_data_lo_vec(i) := RegNext(io.gather.table_data_lo((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        idx(i) := RegNext(index(log2Up(n)-1, 0))
    }

    val prev_data = RegNext(io.gather.prev_data)
    val first_op = RegNext(io.gather.first_op)

    // stage-1
    val prev_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))
    val res_data_vec  = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        prev_data_vec(i) := Mux(first_op, prev_data((VLEN/n)*(i+1)-1, (VLEN/n)*i), io.gather.temp_data((VLEN/n)*(i+1)-1, (VLEN/n)*i))

        when( res_keep_old_vd(i) ) {
            res_data_vec(i) := prev_data_vec(i)
        }.elsewhen( res_agnostic(i) ) {
            res_data_vec(i) := Fill(VLEN/n, 1.U(1.W))
        }.elsewhen( !idx_valid_lo(i) && !idx_valid_hi(i) ) {
            res_data_vec(i) := Mux(first_op, 0.U((VLEN/n).W), prev_data_vec(i))
        }.elsewhen( idx_valid_hi(i) ) {
            res_data_vec(i) := table_data_hi_vec(idx(i))
        }.otherwise {
            res_data_vec(i) := table_data_lo_vec(idx(i))
        }
    }

    io.res_data := res_data_vec.reduce{ (a, b) => Cat(b, a) }
}

class GatherFsmModule extends VPermModule {
    val io = IO(new VPermBundle() {
        val gather   = Input(new GatherFsmIO)
        val res_data = Output(UInt(VLEN.W))
    })

    val gather_fsm_module_0 = Module(new TableLookupFsm(16)) //sew=8
    val gather_fsm_module_1 = Module(new TableLookupFsm(8))  //sew=16
    val gather_fsm_module_2 = Module(new TableLookupFsm(4))  //sew=32
    val gather_fsm_module_3 = Module(new TableLookupFsm(2))  //sew=64

    val gather_fsm_module = VecInit(Seq(gather_fsm_module_0.io, gather_fsm_module_1.io, gather_fsm_module_2.io, gather_fsm_module_3.io))
    for(i <- 0 until 4) {
        gather_fsm_module(i).gather := io.gather
    }

    io.res_data := LookupTree(io.gather.sew, List(
        VectorElementFormat.b -> gather_fsm_module_0.io.res_data,
        VectorElementFormat.h -> gather_fsm_module_1.io.res_data,
        VectorElementFormat.w -> gather_fsm_module_2.io.res_data,
        VectorElementFormat.d -> gather_fsm_module_3.io.res_data
    ))
}
