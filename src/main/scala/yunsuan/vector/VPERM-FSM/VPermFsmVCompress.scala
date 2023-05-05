package yunsuan.vector

import chisel3._
import chisel3.util._
import yunsuan.util._
import yunsuan.vector.vpermutil._
import yunsuan.vector.vpermfsm._
import yunsuan.VectorElementFormat


class CompressFsmIO extends VPermBundle {
    val os_base    = UInt(8.W) // ones_sum_base
    val pmos       = UInt(8.W) // previous_mask_ones_sum

    val sew        = UInt(2.W)
    val vl         = UInt(8.W)
    val ta         = Bool()

    val mask       = UInt(16.W)
    val first_op   = Bool()
    val src_data   = UInt(VLEN.W)
    val prev_data  = UInt(VLEN.W)
    val temp_data  = UInt(VLEN.W)
}

// compress_fsm
class CompressFsm(n: Int) extends VPermModule {
    val io = IO(new VPermBundle() {
        val compress  = Input(new CompressFsmIO)
        val res_data  = Output(UInt(VLEN.W))
    })

    // stage-0
    val src_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))
    val res_agnostic = Wire(Vec(n, Bool()))

    for(i <- 0 until n) {
        src_data_vec(i) := RegNext(io.compress.src_data((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        res_agnostic(i) := RegNext(((io.compress.os_base + i.U) >= io.compress.vl) && io.compress.ta)
    }

    val cmos_vec = Wire(Vec(n+1, UInt(8.W)))
    val cmos_vec_reg = Wire(Vec(n+1, UInt(8.W)))
    for(i <- 0 until n+1) {
        if (i == 0)
            cmos_vec(i) := io.compress.pmos
        else
            cmos_vec(i) := cmos_vec(i-1) + io.compress.mask(i-1)

        cmos_vec_reg(i) := RegNext(cmos_vec(i))
    }

    val os_base = RegNext(io.compress.os_base)
    val mask = RegNext(io.compress.mask)
    val first_op = RegNext(io.compress.first_op)
    val prev_data = RegNext(io.compress.prev_data)

    // stage-1
    val prev_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))
    val temp_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))
    val res_data_vec = Wire(Vec(n, UInt((VLEN/n).W)))
    val compressed_en_vec = Wire(Vec(n, UInt(n.W)))
    val compressed_data_vec = Wire(Vec(n, UInt(VLEN.W)))
    val inst_invalid = io.compress.vl === 0.U

    for(i <- 0 until n) {
        val res_idx = (cmos_vec_reg(i) - os_base)(3, 0)
        val compress_en = mask(i).asBool && (os_base <= cmos_vec_reg(i)) && (cmos_vec_reg(i) < (os_base +& n.U)) && (cmos_vec_reg(i) < io.compress.vl)

        compressed_en_vec(i) := ZeroExt(compress_en.asUInt << res_idx, n)
        compressed_data_vec(i) := Fill(VLEN, compress_en.asUInt) & ZeroExt((src_data_vec(i) << (res_idx << log2Up(VLEN/n))), VLEN)
    }

    val select_compressed = ParallelOR(compressed_en_vec)
    val compressed_data_merged = ParallelOR(compressed_data_vec)
    val compressed_data_merged_vec = Wire(Vec(n, UInt((VLEN/n).W)))

    for(i <- 0 until n) {
        compressed_data_merged_vec(i) := compressed_data_merged((VLEN/n)*(i+1)-1, (VLEN/n)*i)
        prev_data_vec(i) := Mux(first_op, prev_data((VLEN/n)*(i+1)-1, (VLEN/n)*i), io.compress.temp_data((VLEN/n)*(i+1)-1, (VLEN/n)*i))
        temp_data_vec(i) := Mux(inst_invalid, prev_data_vec(i), Mux(res_agnostic(i), Fill(VLEN/n, 1.U(1.W)), prev_data_vec(i)))

        res_data_vec(i) := Mux(select_compressed(i).asBool, compressed_data_merged_vec(i), temp_data_vec(i))
    }

    io.res_data := res_data_vec.reduce{ (a, b) => Cat(b, a) }
}

class CompressFsmModule extends VPermModule {
    val io = IO(new VPermBundle() {
        val compress  = Input(new CompressFsmIO)
        val res_data  = Output(UInt(VLEN.W))
    })

    val compress_fsm_module_0 = Module(new CompressFsm(16)) //sew=8
    val compress_fsm_module_1 = Module(new CompressFsm(8))  //sew=16
    val compress_fsm_module_2 = Module(new CompressFsm(4))  //sew=32
    val compress_fsm_module_3 = Module(new CompressFsm(2))  //sew=64

    val compress_fsm_module = VecInit(Seq(compress_fsm_module_0.io, compress_fsm_module_1.io, compress_fsm_module_2.io, compress_fsm_module_3.io))
    for(i <- 0 until 4) {
        compress_fsm_module(i).compress := io.compress
    }

    io.res_data := LookupTree(io.compress.sew, List(
        VectorElementFormat.b -> compress_fsm_module_0.io.res_data,
        VectorElementFormat.h -> compress_fsm_module_1.io.res_data,
        VectorElementFormat.w -> compress_fsm_module_2.io.res_data,
        VectorElementFormat.d -> compress_fsm_module_3.io.res_data
    ))
}
