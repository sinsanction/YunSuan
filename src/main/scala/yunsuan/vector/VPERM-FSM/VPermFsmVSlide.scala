package yunsuan.vector

import chisel3._
import chisel3.util._
import yunsuan.util._
import yunsuan.vector.vpermutil._
import yunsuan.vector.vpermfsm._
import yunsuan.VectorElementFormat


class SlideFsmIO extends VPermBundle {
    val mask_start_idx  = UInt(7.W)
    val slide           = UInt(4.W)

    val sew             = UInt(2.W)
    val vstart          = UInt(7.W)
    val vl              = UInt(8.W)
    val vm              = Bool()
    val ta              = Bool()
    val ma              = Bool()

    val mask            = UInt(16.W)
    val exceed_limit_hi = Bool()
    val exceed_limit_lo = Bool()
    val src_data_hi     = UInt(VLEN.W)
    val src_data_lo     = UInt(VLEN.W)
    val prev_data       = UInt(VLEN.W)
}

// slideup_lookup_fsm
class SlideUpLookupFsm(n: Int) extends VPermModule {
    val io = IO(new VPermBundle() {
        val slidefsm  = Input(new SlideFsmIO)
        val res_data  = Output(UInt(VLEN.W))
    })

    // stage-0
    val index = Wire(Vec(n, UInt(log2Up(n).W)))
    val res_keep_old_vd = Wire(Vec(n, Bool()))
    val res_agnostic = Wire(Vec(n, Bool()))
    val res_update_hi = Wire(Vec(n, Bool()))
    val res_update_lo = Wire(Vec(n, Bool()))
    val src_data_hi_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val src_data_lo_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val prev_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        val elements_idx = io.slidefsm.mask_start_idx + i.U

        index(i) := RegNext(i.U + ~io.slidefsm.slide + 1.U)
        res_keep_old_vd(i) := RegNext((!io.slidefsm.vm && !io.slidefsm.mask(i).asBool && !io.slidefsm.ma) || (elements_idx < io.slidefsm.vstart) || ((elements_idx >= io.slidefsm.vl) && !io.slidefsm.ta) || (io.slidefsm.vstart >= io.slidefsm.vl))
        res_agnostic(i) := RegNext(((elements_idx >= io.slidefsm.vl) && io.slidefsm.ta) || (!io.slidefsm.vm && !io.slidefsm.mask(i).asBool && io.slidefsm.ma))
        res_update_hi(i) := RegNext(io.slidefsm.slide <= i.U)
        res_update_lo(i) := RegNext((i.U < io.slidefsm.slide) && !io.slidefsm.exceed_limit_lo)

        src_data_hi_vec(i) := RegNext(io.slidefsm.src_data_hi((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        src_data_lo_vec(i) := RegNext(io.slidefsm.src_data_lo((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        prev_data_vec(i)   := RegNext(io.slidefsm.prev_data((VLEN/n)*(i+1)-1, (VLEN/n)*i))
    }

    val exceed_limit_hi = RegNext(io.slidefsm.exceed_limit_hi)

    // stage-1
    val res_data_vec  = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        when( res_keep_old_vd(i) ) {
            res_data_vec(i) := prev_data_vec(i)
        }.elsewhen( res_agnostic(i) ) {
            res_data_vec(i) := Fill(VLEN/n, 1.U(1.W))
        }.elsewhen( exceed_limit_hi ) {
            res_data_vec(i) := prev_data_vec(i)
        }.elsewhen( res_update_hi(i) ) {
            res_data_vec(i) := src_data_hi_vec(index(i))
        }.elsewhen( res_update_lo(i) ) {
            res_data_vec(i) := src_data_lo_vec(index(i))
        }.otherwise {
            res_data_vec(i) := prev_data_vec(i)
        }
    }

    io.res_data := res_data_vec.reduce{ (a, b) => Cat(b, a) }
}

class SlideUpFsmModule extends VPermModule {
    val io = IO(new VPermBundle() {
        val slidefsm  = Input(new SlideFsmIO)
        val res_data  = Output(UInt(VLEN.W))
    })

    val slideup_fsm_module_0 = Module(new SlideUpLookupFsm(16)) //sew=8
    val slideup_fsm_module_1 = Module(new SlideUpLookupFsm(8))  //sew=16
    val slideup_fsm_module_2 = Module(new SlideUpLookupFsm(4))  //sew=32
    val slideup_fsm_module_3 = Module(new SlideUpLookupFsm(2))  //sew=64

    val slideup_fsm_module = VecInit(Seq(slideup_fsm_module_0.io, slideup_fsm_module_1.io, slideup_fsm_module_2.io, slideup_fsm_module_3.io))
    for(i <- 0 until 4) {
        slideup_fsm_module(i).slidefsm := io.slidefsm
    }

    io.res_data := LookupTree(io.slidefsm.sew, List(
        VectorElementFormat.b -> slideup_fsm_module_0.io.res_data,
        VectorElementFormat.h -> slideup_fsm_module_1.io.res_data,
        VectorElementFormat.w -> slideup_fsm_module_2.io.res_data,
        VectorElementFormat.d -> slideup_fsm_module_3.io.res_data
    ))
}

// slidedown_lookup_fsm
class SlideDownLookupFsm(n: Int) extends VPermModule {
    val io = IO(new VPermBundle() {
        val slidefsm  = Input(new SlideFsmIO)
        val res_data  = Output(UInt(VLEN.W))
    })

    // stage-0
    val index = Wire(Vec(n, UInt(log2Up(n).W)))
    val res_keep_old_vd = Wire(Vec(n, Bool()))
    val res_agnostic = Wire(Vec(n, Bool()))
    val res_update_hi = Wire(Vec(n, Bool()))
    val res_update_lo = Wire(Vec(n, Bool()))
    val src_data_hi_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val src_data_lo_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val prev_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        val elements_idx = io.slidefsm.mask_start_idx + i.U

        index(i) := RegNext(i.U + io.slidefsm.slide)
        res_keep_old_vd(i) := RegNext((!io.slidefsm.vm && !io.slidefsm.mask(i).asBool && !io.slidefsm.ma) || (elements_idx < io.slidefsm.vstart) || ((elements_idx >= io.slidefsm.vl) && !io.slidefsm.ta) || (io.slidefsm.vstart >= io.slidefsm.vl))
        res_agnostic(i) := RegNext(((elements_idx >= io.slidefsm.vl) && io.slidefsm.ta) || (!io.slidefsm.vm && !io.slidefsm.mask(i).asBool && io.slidefsm.ma))
        res_update_hi(i) := RegNext((n.U <= (i.U +& io.slidefsm.slide)) && !io.slidefsm.exceed_limit_hi)
        res_update_lo(i) := RegNext((i.U +& io.slidefsm.slide) < n.U)

        src_data_hi_vec(i) := RegNext(io.slidefsm.src_data_hi((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        src_data_lo_vec(i) := RegNext(io.slidefsm.src_data_lo((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        prev_data_vec(i)   := RegNext(io.slidefsm.prev_data((VLEN/n)*(i+1)-1, (VLEN/n)*i))
    }
    
    val exceed_limit_lo = RegNext(io.slidefsm.exceed_limit_lo)

    // stage-1
    val res_data_vec  = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        when( res_keep_old_vd(i) ) {
            res_data_vec(i) := prev_data_vec(i)
        }.elsewhen( res_agnostic(i) ) {
            res_data_vec(i) := Fill(VLEN/n, 1.U(1.W))
        }.elsewhen( exceed_limit_lo ){
            res_data_vec(i) := 0.U((VLEN/n).W)
        }.elsewhen( res_update_lo(i) ) {
            res_data_vec(i) := src_data_lo_vec(index(i))
        }.elsewhen( res_update_hi(i) ) {
            res_data_vec(i) := src_data_hi_vec(index(i))
        }.otherwise {
            res_data_vec(i) := 0.U((VLEN/n).W)
        }
    }

    io.res_data := res_data_vec.reduce{ (a, b) => Cat(b, a) }
}

class SlideDownFsmModule extends VPermModule {
    val io = IO(new VPermBundle() {
        val slidefsm  = Input(new SlideFsmIO)
        val res_data  = Output(UInt(VLEN.W))
    })

    val slidedown_fsm_module_0 = Module(new SlideDownLookupFsm(16)) //sew=8
    val slidedown_fsm_module_1 = Module(new SlideDownLookupFsm(8))  //sew=16
    val slidedown_fsm_module_2 = Module(new SlideDownLookupFsm(4))  //sew=32
    val slidedown_fsm_module_3 = Module(new SlideDownLookupFsm(2))  //sew=64

    val slidedown_fsm_module = VecInit(Seq(slidedown_fsm_module_0.io, slidedown_fsm_module_1.io, slidedown_fsm_module_2.io, slidedown_fsm_module_3.io))
    for(i <- 0 until 4) {
        slidedown_fsm_module(i).slidefsm := io.slidefsm
    }

    io.res_data := LookupTree(io.slidefsm.sew, List(
        VectorElementFormat.b -> slidedown_fsm_module_0.io.res_data,
        VectorElementFormat.h -> slidedown_fsm_module_1.io.res_data,
        VectorElementFormat.w -> slidedown_fsm_module_2.io.res_data,
        VectorElementFormat.d -> slidedown_fsm_module_3.io.res_data
    ))
}
