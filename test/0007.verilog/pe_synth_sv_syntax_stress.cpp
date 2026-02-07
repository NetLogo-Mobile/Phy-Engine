#include <cstddef>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

int main()
{
    using namespace ::phy_engine::verilog::digital;

    // Same SV stress source as `sv_syntax_stress.cpp`, but exercised through PE synthesis.
    decltype(auto) src = u8R"SV(
`timescale 1ns/1ps

//============================================================
// 0) 基础宏/工具函数
//============================================================
`define STR(x) `"x`"
`define CAT(a,b) a``b

//============================================================
// 1) package：typedef/enum/struct/union/class/import/export
//============================================================
package sv_pkg;
  // SystemVerilog: 2-state types & integer types
  typedef bit        b1_t;
  typedef bit [1:0]  b2_t;
  typedef logic [1:0] l2_t;

  typedef byte       byte_t;
  typedef shortint   s16_t;
  typedef int        s32_t;
  typedef longint    s64_t;

  // enum
  typedef enum logic [1:0] {IDLE=2'b00, RUN=2'b01, WAIT=2'b10, DONE=2'b11} state_e;

  // struct / union packed
  typedef struct packed {
    logic [3:0] a;
    logic [3:0] b;
  } ab4_s;

  typedef union packed {
    logic [7:0] u8;
    ab4_s      ab;
  } u8_ab_u;

  // parameter type (SV)
  parameter type data_t = logic [7:0];

  // function with automatic + typed arguments/return
  function automatic int add_int(input int x, input int y);
    return x + y;
  endfunction

  // SV string
  function automatic string hello(input string name);
    return {"hello, ", name};
  endfunction

`ifdef ENABLE_OOP
  // class / rand / constraint / new / virtual
  class pkt;
    rand bit [7:0]  addr;
    rand bit [15:0] data;
    constraint c_addr { addr inside {[8'h10:8'h1F]}; }
    function new(); endfunction
    function string sprint();
      return $sformatf("pkt(addr=0x%0h, data=0x%0h)", addr, data);
    endfunction
  endclass
`endif
endpackage : sv_pkg


//============================================================
// 2) interface / modport / clocking block
//============================================================
interface bus_if #(parameter int AW=8, DW=16) (input logic clk);
  logic rst_n;

  logic          valid;
  logic          ready;
  logic [AW-1:0] addr;
  logic [DW-1:0] wdata;
  logic [DW-1:0] rdata;

  // clocking block (SV)
  clocking cb @(posedge clk);
    default input #1step output #1step;
    output valid, addr, wdata;
    input  ready, rdata;
  endclocking

  // modport (SV)
  modport master (clocking cb, output rst_n);
  modport slave  (input clk, input rst_n,
                  input valid, input addr, input wdata,
                  output ready, output rdata);
endinterface : bus_if


//============================================================
// 3) SystemVerilog Assertions (SVA): sequence/property/assert
//============================================================
`ifdef ENABLE_SVA
module sva_checker #(parameter int AW=8, DW=16) (
  input logic clk,
  input logic rst_n,
  input logic valid,
  input logic ready,
  input logic [AW-1:0] addr
);
  // sequence/property (SV)
  sequence s_handshake;
    valid ##1 ready;
  endsequence

  property p_eventually_ready;
    @(posedge clk) disable iff (!rst_n)
      valid |-> ##[1:3] ready;
  endproperty

  // concurrent assertion
  a_eventually_ready: assert property (p_eventually_ready)
    else $error("SVA fail: valid not followed by ready in 1..3 cycles, addr=%0h", addr);

  // immediate assertion (SV also支持)
  always_ff @(posedge clk) begin
    if (rst_n) begin
      assert (^addr !== 1'bx) else $fatal("addr has X at time %0t", $time);
    end
  end
endmodule
`endif


//============================================================
// 4) coverage: covergroup/coverpoint/bins
//============================================================
`ifdef ENABLE_COVER
module cov_checker(input logic clk, input logic rst_n, input logic [1:0] st);
  covergroup cg @(posedge clk);
    option.per_instance = 1;
    cp_st: coverpoint st iff (rst_n) {
      bins all[] = {[0:3]};
      illegal_bins bad = default; // “default” 在某些工具实现不一
    }
  endgroup
  cg cg_i = new();
endmodule
`endif


//============================================================
// 5) DPI（可选）：只测试语法，不要求链接成功
//============================================================
`ifdef ENABLE_DPI
import "DPI-C" function int c_add(input int a, input int b);
`endif


//============================================================
// 6) 设计模块：覆盖 declarations/arrays/always_* / generate / foreach / unique / priority / streaming / cast
//============================================================
module dut #(
  parameter int W = 2,
  parameter type T = logic [7:0]
)(
  input  logic clk,
  input  logic rst_n,
  bus_if.master m
);
  import sv_pkg::*;

  //-------------------------
  // 6.1 你关心的 reg[1:0] / [1:0] 声明覆盖
  //-------------------------
  // Verilog-2001:
  reg [1:0] v2001_reg_range;     // 经典写法（如果这都不支持，你的解析器很可疑）
  reg [0:1] v2001_reg_range_rev; // 反向范围（有些烂前端会挂）

  // SystemVerilog:
  logic [1:0] sv_logic_range;
  bit   [1:0] sv_bit_range;      // 2-state packed
  logic [W-1:0] param_range;     // 参数化宽度

  // packed + unpacked array:
  logic [1:0] packed2_unpacked [0:3]; // 4个元素，每个元素2bit（SV允许这种组合）
  logic [3:0][1:0] multi_packed;      // 多维 packed（SV）

  //-------------------------
  // 6.2 新类型：int/byte/string/enum/struct/union
  //-------------------------
  int      i32;
  byte     b8;
  string   s;
  state_e  st;
  ab4_s    ab;
  u8_ab_u  uu;

  //-------------------------
  // 6.3 动态数组/关联数组/队列 + foreach
  //-------------------------
  int dyn[];                 // dynamic array (SV)
  int assoc[string];         // associative array (SV)
  int q[$];                  // queue (SV)

  //-------------------------
  // 6.4 mailbox/semaphore/event（SV增强）
  //-------------------------
  mailbox #(int) mbx;
  semaphore sem;
  event ev;

  //-------------------------
  // 6.5 always_ff / always_comb / always_latch + unique/priority
  //-------------------------
  logic [7:0] a, b, y;
  logic latch_en;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      v2001_reg_range <= '0;
      sv_logic_range  <= '0;
      st              <= IDLE;
    end else begin
      v2001_reg_range <= v2001_reg_range + 2'd1;
      sv_logic_range  <= sv_logic_range ^ 2'b11;

      unique case (st) // unique (SV)
        IDLE: st <= RUN;
        RUN : st <= WAIT;
        WAIT: st <= DONE;
        default: st <= IDLE;
      endcase
    end
  end

  always_comb begin
    // priority if (SV)
    priority if (a[0]) y = a + b;
    else if (a[1])     y = a - b;
    else               y = a ^ b;
  end

  always_latch begin
    if (latch_en) begin
      b8 = byte'(y); // cast (SV)
    end
  end

  //-------------------------
  // 6.6 streaming operator (SV): {<<{}} / {>>{}}
  //-------------------------
  logic [15:0] stream_in;
  logic [15:0] stream_out;
  always_comb begin
    stream_out = {<<8{stream_in}}; // byte-stream (SV)
  end

  //-------------------------
  // 6.7 generate / genvar / for-generate / if-generate
  //-------------------------
  genvar g;
  for (g=0; g<4; g++) begin : GEN_BLK
    always_comb begin
      packed2_unpacked[g] = logic'(g[1:0]) ^ sv_logic_range;
    end
  end

  if (W == 2) begin : GEN_IF
    // localparam + typed param
    localparam int K = 7;
  end

  //-------------------------
  // 6.8 函数/任务 automatic + 默认参数 + ref 参数（SV）
  //-------------------------
  task automatic t_ref(input int n, ref int acc, input int step=1);
    int k;
    for (k=0; k<n; k++) acc += step;
  endtask

  function automatic T f_id(input T x);
    return x;
  endfunction

  //============================================================
  // 7) 初始块：让工具把语法都吃到（运行不重要）
  //============================================================
  initial begin
    // string + concat
    s = sv_pkg::hello("sv");
    s = {s, " / time=", $sformatf("%0t", $time)};

    // struct/union
    ab.a = 4'hA;
    ab.b = 4'h5;
    uu.ab = ab;
    stream_in = {uu.u8, uu.u8};

    // dynamic array
    dyn = new[3];
    dyn[0]=10; dyn[1]=20; dyn[2]=30;
    foreach (dyn[idx]) begin
      i32 += dyn[idx];
    end

    // assoc array
    assoc["one"] = 1;
    assoc["two"] = 2;
    foreach (assoc[k]) begin
      i32 += assoc[k];
    end

    // queue
    q.push_back(11);
    q.push_front(9);
    void'(q.pop_back());

    // mailbox/semaphore/event
    mbx = new();
    sem = new(1);
    -> ev;
    sem.get(1);
    mbx.put(123);
    sem.put(1);

    // ref task
    t_ref(3, i32, 2);

`ifdef ENABLE_DPI
    // DPI 语法测试（不保证链接）
    i32 = c_add(1, 2);
`endif

`ifdef ENABLE_OOP
    // class randomize
    pkt p = new();
    void'(p.randomize());
    s = {s, " / ", p.sprint()};
`endif

    // $bits / '0 / '1 / 'x / 'z (SV literals)
    v2001_reg_range = '0;
    sv_logic_range  = '1;
    sv_bit_range    = '0;

    // finish
    #1;
    $display("SV syntax smoke ran. s=%s i32=%0d", s, i32);
    $finish;
  end

  //============================================================
  // 8) interface 信号驱动示例（clocking block）
  //============================================================
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      m.rst_n     <= 1'b0;
      m.cb.valid  <= 1'b0;
      m.cb.addr   <= '0;
      m.cb.wdata  <= '0;
    end else begin
      m.rst_n     <= 1'b1;
      m.cb.valid  <= 1'b1;
      m.cb.addr   <= {6'd0, v2001_reg_range}; // 混合拼接
      m.cb.wdata  <= {14'd0, sv_logic_range};
    end
  end

endmodule : dut


//============================================================
// 9) 顶层：实例化 interface + dut + 可选 SVA/COV
//============================================================
module top;
  import sv_pkg::*;

  timeunit 1ns; timeprecision 1ps; // SV timeunit/timeprecision

  logic clk;
  logic rst_n;

  // interface instance
  bus_if #(8,16) bus (.*);

  // clock
  initial clk = 0;
  always #5 clk = ~clk;

  // reset
  initial begin
    rst_n = 0;
    bus.rst_n = 0;
    repeat (3) @(posedge clk);
    rst_n = 1;
    bus.rst_n = 1;
  end

  // dut
  dut #(.W(2), .T(logic [7:0])) u_dut (
    .clk (clk),
    .rst_n (rst_n),
    .m (bus)
  );

`ifdef ENABLE_SVA
  sva_checker #(.AW(8), .DW(16)) u_sva (
    .clk(clk), .rst_n(rst_n),
    .valid(bus.valid), .ready(bus.ready),
    .addr(bus.addr)
  );
`endif

`ifdef ENABLE_COVER
  cov_checker u_cov(.clk(clk), .rst_n(rst_n), .st(u_dut.st));
`endif

  // slave side simple response
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bus.ready <= 1'b0;
      bus.rdata <= '0;
    end else begin
      bus.ready <= 1'b1;
      bus.rdata <= {8'h55, bus.addr};
    end
  end
endmodule : top
)SV";

    auto cr = compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }

    auto design = build_design(::std::move(cr));
    auto const* top_mod = find_module(design, u8"top");
    if(top_mod == nullptr) { return 2; }

    auto top_inst = elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    ::phy_engine::circult c{};
    auto& nl = c.get_netlist();

    ::std::vector<::phy_engine::model::node_t*> ports{};  // top has no ports in this stress file

    ::phy_engine::verilog::digital::pe_synth_error err{};
    if(!synthesize_to_pe_netlist(nl, top_inst, ports, &err, {})) { return 4; }
    return 0;
}

