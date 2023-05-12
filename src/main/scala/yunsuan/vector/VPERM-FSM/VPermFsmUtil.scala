package yunsuan.vector.vpermfsm

import chisel3._
import chisel3.util._
import yunsuan._
import yunsuan.util._
import yunsuan.vector.vpermutil._

class AccessTable(n: Int) extends VPermModule {
    val io = IO(new VPermBundle() {
        val elem_num_pow = Input(UInt(3.W))
        val lmul         = Input(UInt(4.W))
        val vs1_data     = Input(UInt(VLEN.W))
        val res_data     = Output(UInt(8.W))
    })

    val vs1_data_vec  = Wire(Vec(n, UInt((VLEN/n).W)))
    val res_data_vec  = Wire(Vec(n, UInt(8.W)))

    for(i <- 0 until n) {
        vs1_data_vec(i) := io.vs1_data((VLEN/n)*(i+1)-1, (VLEN/n)*i)
        val vs2_idx = (vs1_data_vec(i) >> io.elem_num_pow)
        when( vs2_idx >= io.lmul ) {
            res_data_vec(i) := 0.U(8.W)
        }.otherwise {
            res_data_vec(i) := UIntToOH(vs2_idx(2, 0), 8)
        }
    }

    io.res_data := res_data_vec.reduce(_ | _)
}

class AccessTableModule extends VPermModule {
    val io = IO(new VPermBundle() {
        val sew          = Input(UInt(2.W))
        val elem_num_pow = Input(UInt(3.W))
        val lmul         = Input(UInt(4.W))
        val opcode       = Input(UInt(3.W))
        val vs1_data     = Input(UInt(VLEN.W))
        val res_data     = Output(UInt(8.W))
    })

    // vrgather.vv
    val ac_table_module_0 = Module(new AccessTable(16)) //sew=8
    val ac_table_module_1 = Module(new AccessTable(8))  //sew=16
    val ac_table_module_2 = Module(new AccessTable(4))  //sew=32
    val ac_table_module_3 = Module(new AccessTable(2))  //sew=64

    val ac_table_module = VecInit(Seq(ac_table_module_0.io, ac_table_module_1.io, ac_table_module_2.io, ac_table_module_3.io))
    for(i <- 0 until 4) {
        ac_table_module(i).elem_num_pow := io.elem_num_pow
        ac_table_module(i).lmul         := io.lmul
        ac_table_module(i).vs1_data     := io.vs1_data
    }

    // vrgather.vx/vi
    val ac_table_vxvi = Wire(UInt(8.W))
    val vs2_idx = (io.vs1_data(XLEN-1, 0) >> io.elem_num_pow)
    when( vs2_idx >= io.lmul ) {
        ac_table_vxvi := 0.U(8.W)
    }.otherwise {
        ac_table_vxvi := UIntToOH(vs2_idx(2, 0), 8)
    }

    io.res_data := Mux(io.opcode === VPermFsmType.vrgathervxvi, ac_table_vxvi, LookupTree(io.sew, List(
        VectorElementFormat.b -> ac_table_module_0.io.res_data,
        VectorElementFormat.h -> ac_table_module_1.io.res_data,
        VectorElementFormat.w -> ac_table_module_2.io.res_data,
        VectorElementFormat.d -> ac_table_module_3.io.res_data
    )))
}

class AccessTable16(n: Int) extends VPermModule {
    val io = IO(new VPermBundle() {
        val elem_num_pow = Input(UInt(3.W))
        val lmul         = Input(UInt(4.W))
        val vs1_data     = Input(UInt((16*n).W))
        val res_data     = Output(UInt(8.W))
    })

    val vs1_data_vec  = Wire(Vec(n, UInt(16.W)))
    val res_data_vec  = Wire(Vec(n, UInt(8.W)))

    for(i <- 0 until n) {
        vs1_data_vec(i) := io.vs1_data(16*(i+1)-1, 16*i)
        val vs2_idx = (vs1_data_vec(i) >> io.elem_num_pow)
        when( vs2_idx >= io.lmul ) {
            res_data_vec(i) := 0.U(8.W)
        }.otherwise {
            res_data_vec(i) := UIntToOH(vs2_idx(2, 0), 8)
        }
    }

    io.res_data := res_data_vec.reduce(_ | _)
}

class AccessTable16Module extends VPermModule {
    val io = IO(new VPermBundle() {
        val sew          = Input(UInt(2.W))
        val elem_num_pow = Input(UInt(3.W))
        val lmul         = Input(UInt(4.W))
        val uop_idx      = Input(UInt(3.W))
        val vs1_data     = Input(UInt(VLEN.W))
        val res_data     = Output(UInt(8.W))
    })

    val vs1_data_32 = Mux(io.uop_idx(0), io.vs1_data(127, 64), io.vs1_data(63, 0))
    val vs1_data_64 = LookupTree(io.uop_idx(1,0), List(
        "b00".U -> io.vs1_data(31, 0),
        "b01".U -> io.vs1_data(63, 32),
        "b10".U -> io.vs1_data(95, 64),
        "b11".U -> io.vs1_data(127, 96)
    ))

    // vrgather.vv
    val ac_table_module_0 = Module(new AccessTable16(8))  //sew=8/16
    val ac_table_module_1 = Module(new AccessTable16(4))  //sew=32
    val ac_table_module_2 = Module(new AccessTable16(2))  //sew=64

    ac_table_module_0.io.vs1_data := io.vs1_data
    ac_table_module_0.io.elem_num_pow := io.elem_num_pow
    ac_table_module_0.io.lmul := io.lmul
    ac_table_module_1.io.vs1_data := vs1_data_32
    ac_table_module_1.io.elem_num_pow := io.elem_num_pow
    ac_table_module_1.io.lmul := io.lmul
    ac_table_module_2.io.vs1_data := vs1_data_64
    ac_table_module_2.io.elem_num_pow := io.elem_num_pow
    ac_table_module_2.io.lmul := io.lmul

    io.res_data := LookupTree(io.sew, List(
        VectorElementFormat.b -> ac_table_module_0.io.res_data,
        VectorElementFormat.h -> ac_table_module_0.io.res_data,
        VectorElementFormat.w -> ac_table_module_1.io.res_data,
        VectorElementFormat.d -> ac_table_module_2.io.res_data
    ))
}

object SetOnlyFirst {
  def apply(in: UInt) = {
    var in_or = Wire(Vec(in.getWidth, UInt(1.W)))
    var out = Wire(Vec(in.getWidth, UInt(1.W)))

    for (i <- 0 until in.getWidth) {
      if (i == 0) {
        out(i) := in(i)
        in_or(i) := in(i)
      }
      else {
        out(i) := in(i) & ~in_or(i-1)
        in_or(i) := in(i) | in_or(i-1)
      }
    }

    out.reduce{ (a, b) => Cat(b, a) }
  }
}
