// RV32I (unprivileged) single-cycle core implemented in a restricted Verilog subset
// (no memories/arrays), intended for Verilog->PE netlist synthesis tests.
//
// - ISA: RV32I base integer (most common opcodes), plus SYSTEM treated as NOP except ECALL/EBREAK.
// - Memory: tiny internal data RAM implemented as 16x32-bit registers (word addressed by addr[5:2]).
// - Instruction ROM: hard-coded program via case(pc[6:2]) (no $readmemh).
// - Register file: x1..x31 explicit regs; x0 is hard-wired to 0 via decode mux.
//
// This is not a cycle-accurate micro-architectural model; it's a synthesizable functional core for logic synthesis.

`ifndef RESET_PC
`define RESET_PC 32'h00000000
`endif

`ifndef UART_MMIO_ADDR
`define UART_MMIO_ADDR 32'h10000000
`endif

module riscv_top(input clk, input rst_n, output done, output uart_valid, output [7:0] uart_data);
  // Program counter and fetched instruction.
  reg [31:0] pc;
  reg [31:0] instr;

  // done is driven by x10[0] (a0 bit0) for the test harness.
  // If the program writes x10=1, done becomes 1.
  wire done_w;
  assign done = done_w;

  // Memory-mapped UART write pulse.
  reg uart_valid_r;
  reg [7:0] uart_data_r;
  assign uart_valid = uart_valid_r;
  assign uart_data = uart_data_r;

  // Integer registers x1..x31 (x0 is always 0).
  reg [31:0] x1;
  reg [31:0] x2;
  reg [31:0] x3;
  reg [31:0] x4;
  reg [31:0] x5;
  reg [31:0] x6;
  reg [31:0] x7;
  reg [31:0] x8;
  reg [31:0] x9;
  reg [31:0] x10;
  reg [31:0] x11;
  reg [31:0] x12;
  reg [31:0] x13;
  reg [31:0] x14;
  reg [31:0] x15;
  reg [31:0] x16;
  reg [31:0] x17;
  reg [31:0] x18;
  reg [31:0] x19;
  reg [31:0] x20;
  reg [31:0] x21;
  reg [31:0] x22;
  reg [31:0] x23;
  reg [31:0] x24;
  reg [31:0] x25;
  reg [31:0] x26;
  reg [31:0] x27;
  reg [31:0] x28;
  reg [31:0] x29;
  reg [31:0] x30;
  reg [31:0] x31;

  assign done_w = x10[0];

  // Tiny internal data RAM: 16 words.
  reg [31:0] mem0;
  reg [31:0] mem1;
  reg [31:0] mem2;
  reg [31:0] mem3;
  reg [31:0] mem4;
  reg [31:0] mem5;
  reg [31:0] mem6;
  reg [31:0] mem7;
  reg [31:0] mem8;
  reg [31:0] mem9;
  reg [31:0] mem10;
  reg [31:0] mem11;
  reg [31:0] mem12;
  reg [31:0] mem13;
  reg [31:0] mem14;
  reg [31:0] mem15;

  // Instruction ROM (word-addressed by pc[6:2] in the default include).
  always @* begin
    instr = 32'h00000013; // NOP (addi x0,x0,0)
`include "riscv_rom.vh"
  end

  // Decode fields.
  wire [6:0] opcode;
  wire [4:0] rd;
  wire [2:0] funct3;
  wire [4:0] rs1;
  wire [4:0] rs2;
  wire [6:0] funct7;
  assign opcode = instr[6:0];
  assign rd = instr[11:7];
  assign funct3 = instr[14:12];
  assign rs1 = instr[19:15];
  assign rs2 = instr[24:20];
  assign funct7 = instr[31:25];

  // Immediates.
  wire [31:0] imm_i;
  wire [31:0] imm_s;
  wire [31:0] imm_b;
  wire [31:0] imm_u;
  wire [31:0] imm_j;

  assign imm_i = {{20{instr[31]}}, instr[31:20]};
  assign imm_s = {{20{instr[31]}}, instr[31:25], instr[11:7]};
  assign imm_b = {{19{instr[31]}}, instr[31], instr[7], instr[30:25], instr[11:8], 1'b0};
  assign imm_u = {instr[31:12], 12'b0};
  assign imm_j = {{11{instr[31]}}, instr[31], instr[19:12], instr[20], instr[30:21], 1'b0};

  // SYSTEM decode (CSR/traps).
  wire [11:0] csr_addr;
  assign csr_addr = instr[31:20];
  wire is_system;
  assign is_system = (opcode == 7'b1110011);
  wire is_ecall;
  wire is_ebreak;
  wire is_mret;
  assign is_ecall = (instr == 32'h00000073);
  assign is_ebreak = (instr == 32'h00100073);
  assign is_mret = (instr == 32'h30200073);

  // Register reads (combinational).
  reg [31:0] rs1_val;
  reg [31:0] rs2_val;

  always @* begin
    rs1_val = 32'h0;
    case(rs1)
      5'd0: rs1_val = 32'h0;
      5'd1: rs1_val = x1;
      5'd2: rs1_val = x2;
      5'd3: rs1_val = x3;
      5'd4: rs1_val = x4;
      5'd5: rs1_val = x5;
      5'd6: rs1_val = x6;
      5'd7: rs1_val = x7;
      5'd8: rs1_val = x8;
      5'd9: rs1_val = x9;
      5'd10: rs1_val = x10;
      5'd11: rs1_val = x11;
      5'd12: rs1_val = x12;
      5'd13: rs1_val = x13;
      5'd14: rs1_val = x14;
      5'd15: rs1_val = x15;
      5'd16: rs1_val = x16;
      5'd17: rs1_val = x17;
      5'd18: rs1_val = x18;
      5'd19: rs1_val = x19;
      5'd20: rs1_val = x20;
      5'd21: rs1_val = x21;
      5'd22: rs1_val = x22;
      5'd23: rs1_val = x23;
      5'd24: rs1_val = x24;
      5'd25: rs1_val = x25;
      5'd26: rs1_val = x26;
      5'd27: rs1_val = x27;
      5'd28: rs1_val = x28;
      5'd29: rs1_val = x29;
      5'd30: rs1_val = x30;
      5'd31: rs1_val = x31;
      default: rs1_val = 32'h0;
    endcase
  end

  always @* begin
    rs2_val = 32'h0;
    case(rs2)
      5'd0: rs2_val = 32'h0;
      5'd1: rs2_val = x1;
      5'd2: rs2_val = x2;
      5'd3: rs2_val = x3;
      5'd4: rs2_val = x4;
      5'd5: rs2_val = x5;
      5'd6: rs2_val = x6;
      5'd7: rs2_val = x7;
      5'd8: rs2_val = x8;
      5'd9: rs2_val = x9;
      5'd10: rs2_val = x10;
      5'd11: rs2_val = x11;
      5'd12: rs2_val = x12;
      5'd13: rs2_val = x13;
      5'd14: rs2_val = x14;
      5'd15: rs2_val = x15;
      5'd16: rs2_val = x16;
      5'd17: rs2_val = x17;
      5'd18: rs2_val = x18;
      5'd19: rs2_val = x19;
      5'd20: rs2_val = x20;
      5'd21: rs2_val = x21;
      5'd22: rs2_val = x22;
      5'd23: rs2_val = x23;
      5'd24: rs2_val = x24;
      5'd25: rs2_val = x25;
      5'd26: rs2_val = x26;
      5'd27: rs2_val = x27;
      5'd28: rs2_val = x28;
      5'd29: rs2_val = x29;
      5'd30: rs2_val = x30;
      5'd31: rs2_val = x31;
      default: rs2_val = 32'h0;
    endcase
  end

  // Signed views for signed comparisons.
  wire signed [31:0] rs1_s;
  wire signed [31:0] rs2_s;
  assign rs1_s = rs1_val;
  assign rs2_s = rs2_val;
  wire signed [31:0] imm_i_s;
  assign imm_i_s = imm_i;

  // M-extension microcode (MUL* uses multi-cycle shift-add to keep expressions small enough for the synthesizer).
  reg mul_busy;
  reg [5:0] mul_cnt;
  reg [63:0] mul_acc;
  reg [63:0] mul_a;
  reg [31:0] mul_b;
  reg [4:0] mul_rd;
  reg mul_take_high;
  reg mul_neg;
  reg start_mul;

  wire [63:0] mul_acc_add;
  wire [63:0] mul_acc_next;
  wire [63:0] mul_a_next;
  wire [31:0] mul_b_next;
  assign mul_acc_add = mul_acc + mul_a;
  assign mul_acc_next = mul_b[0] ? mul_acc_add : mul_acc;
  assign mul_a_next = mul_a << 1;
  assign mul_b_next = mul_b >> 1;

  wire [63:0] mul_prod_abs;
  wire [63:0] mul_prod_signed;
  assign mul_prod_abs = mul_acc_next;  // product after applying the final step
  assign mul_prod_signed = mul_neg ? ((~mul_prod_abs) + 64'd1) : mul_prod_abs;
  wire [31:0] mul_result_low;
  wire [31:0] mul_result_high;
  assign mul_result_low = mul_prod_signed[31:0];
  assign mul_result_high = mul_prod_signed[63:32];

  // Memory address and read data (combinational read).
  reg [31:0] mem_addr;
  reg [31:0] mem_rdata_word;

  always @* begin
    mem_rdata_word = 32'h0;
    case(mem_addr[5:2])
      4'd0: mem_rdata_word = mem0;
      4'd1: mem_rdata_word = mem1;
      4'd2: mem_rdata_word = mem2;
      4'd3: mem_rdata_word = mem3;
      4'd4: mem_rdata_word = mem4;
      4'd5: mem_rdata_word = mem5;
      4'd6: mem_rdata_word = mem6;
      4'd7: mem_rdata_word = mem7;
      4'd8: mem_rdata_word = mem8;
      4'd9: mem_rdata_word = mem9;
      4'd10: mem_rdata_word = mem10;
      4'd11: mem_rdata_word = mem11;
      4'd12: mem_rdata_word = mem12;
      4'd13: mem_rdata_word = mem13;
      4'd14: mem_rdata_word = mem14;
      4'd15: mem_rdata_word = mem15;
      default: mem_rdata_word = 32'h0;
    endcase
  end

  // CSR read mux.
  always @* begin
    csr_rdata = 32'h0;
    case(csr_addr)
      12'h300: csr_rdata = csr_mstatus;
      12'h305: csr_rdata = csr_mtvec;
      12'h341: csr_rdata = csr_mepc;
      12'h342: csr_rdata = csr_mcause;
      12'h343: csr_rdata = csr_mtval;
      12'hb00: csr_rdata = csr_mcycle;
      12'hb02: csr_rdata = csr_minstret;
      default: csr_rdata = 32'h0;
    endcase
  end

  // Combinational helpers for arithmetic shift right (SRA/SRAI) without using >>>.
  reg [31:0] sra_out;
  reg [31:0] sra_in;
  reg [4:0] sra_shamt;
  always @* begin
    sra_out = 32'h0;
    case(sra_shamt)
      5'd0: sra_out = sra_in;
      5'd1: sra_out = { {1{sra_in[31]}},  sra_in[31:1]  };
      5'd2: sra_out = { {2{sra_in[31]}},  sra_in[31:2]  };
      5'd3: sra_out = { {3{sra_in[31]}},  sra_in[31:3]  };
      5'd4: sra_out = { {4{sra_in[31]}},  sra_in[31:4]  };
      5'd5: sra_out = { {5{sra_in[31]}},  sra_in[31:5]  };
      5'd6: sra_out = { {6{sra_in[31]}},  sra_in[31:6]  };
      5'd7: sra_out = { {7{sra_in[31]}},  sra_in[31:7]  };
      5'd8: sra_out = { {8{sra_in[31]}},  sra_in[31:8]  };
      5'd9: sra_out = { {9{sra_in[31]}},  sra_in[31:9]  };
      5'd10: sra_out = { {10{sra_in[31]}}, sra_in[31:10] };
      5'd11: sra_out = { {11{sra_in[31]}}, sra_in[31:11] };
      5'd12: sra_out = { {12{sra_in[31]}}, sra_in[31:12] };
      5'd13: sra_out = { {13{sra_in[31]}}, sra_in[31:13] };
      5'd14: sra_out = { {14{sra_in[31]}}, sra_in[31:14] };
      5'd15: sra_out = { {15{sra_in[31]}}, sra_in[31:15] };
      5'd16: sra_out = { {16{sra_in[31]}}, sra_in[31:16] };
      5'd17: sra_out = { {17{sra_in[31]}}, sra_in[31:17] };
      5'd18: sra_out = { {18{sra_in[31]}}, sra_in[31:18] };
      5'd19: sra_out = { {19{sra_in[31]}}, sra_in[31:19] };
      5'd20: sra_out = { {20{sra_in[31]}}, sra_in[31:20] };
      5'd21: sra_out = { {21{sra_in[31]}}, sra_in[31:21] };
      5'd22: sra_out = { {22{sra_in[31]}}, sra_in[31:22] };
      5'd23: sra_out = { {23{sra_in[31]}}, sra_in[31:23] };
      5'd24: sra_out = { {24{sra_in[31]}}, sra_in[31:24] };
      5'd25: sra_out = { {25{sra_in[31]}}, sra_in[31:25] };
      5'd26: sra_out = { {26{sra_in[31]}}, sra_in[31:26] };
      5'd27: sra_out = { {27{sra_in[31]}}, sra_in[31:27] };
      5'd28: sra_out = { {28{sra_in[31]}}, sra_in[31:28] };
      5'd29: sra_out = { {29{sra_in[31]}}, sra_in[31:29] };
      5'd30: sra_out = { {30{sra_in[31]}}, sra_in[31:30] };
      5'd31: sra_out = { {31{sra_in[31]}}, sra_in[31:31] };
      default: sra_out = sra_in;
    endcase
  end

  // Load unit (from mem_rdata_word + mem_addr).
  reg [31:0] load_data;
  reg [7:0] load_b;
  reg [15:0] load_h;
  always @* begin
    load_data = mem_rdata_word;
    load_b = 8'h00;
    load_h = 16'h0000;
    case(mem_addr[1:0])
      2'd0: load_b = mem_rdata_word[7:0];
      2'd1: load_b = mem_rdata_word[15:8];
      2'd2: load_b = mem_rdata_word[23:16];
      2'd3: load_b = mem_rdata_word[31:24];
      default: load_b = mem_rdata_word[7:0];
    endcase
    case(mem_addr[1])
      1'b0: load_h = mem_rdata_word[15:0];
      1'b1: load_h = mem_rdata_word[31:16];
      default: load_h = mem_rdata_word[15:0];
    endcase
  end

  // Next-state control.
  reg [31:0] next_pc;
  reg reg_write_en;
  reg [31:0] reg_write_data;

  reg mem_write_en;
  reg [31:0] mem_write_mask;
  reg [31:0] mem_write_value;
  reg [4:0] byte_shift;
  reg [4:0] half_shift;

  // CSR state (minimal machine-mode subset).
  reg [31:0] csr_mstatus;
  reg [31:0] csr_mtvec;
  reg [31:0] csr_mepc;
  reg [31:0] csr_mcause;
  reg [31:0] csr_mtval;
  reg [31:0] csr_mcycle;
  reg [31:0] csr_minstret;
  reg [31:0] csr_rdata;
  reg csr_we;
  reg [11:0] csr_waddr;
  reg [31:0] csr_wdata;

  // Trap/return control.
  reg trap_req;
  reg [31:0] trap_cause;
  reg [31:0] trap_tval;
  reg mret_req;

  // UART pulse from stores to UART_MMIO_ADDR.
  reg uart_pulse;
  reg [7:0] uart_pulse_data;

  // Divider microcode (for DIV/DIVU/REM/REMU).
  reg div_busy;
  reg [5:0] div_cnt;
  reg [31:0] div_dvd;
  reg [31:0] div_dvs;
  reg [31:0] div_rem;
  reg [31:0] div_quot;
  reg [4:0] div_rd;
  reg div_is_signed;
  reg div_is_rem;
  reg div_neg_q;
  reg div_neg_r;
  reg start_div;

  wire [31:0] div_rem_shift;
  wire [31:0] div_dvd_shift;
  wire div_ge;
  wire [31:0] div_rem_sub;
  wire [31:0] div_quot_shift;
  assign div_rem_shift = {div_rem[30:0], div_dvd[31]};
  assign div_dvd_shift = {div_dvd[30:0], 1'b0};
  assign div_ge = (div_rem_shift >= div_dvs);
  assign div_rem_sub = div_rem_shift - div_dvs;
  assign div_quot_shift = {div_quot[30:0], div_ge};

  wire [31:0] div_step_rem_next;
  wire [31:0] div_step_quot_next;
  assign div_step_rem_next = div_ge ? div_rem_sub : div_rem_shift;
  assign div_step_quot_next = div_quot_shift;

  wire [31:0] div_finish_q_final;
  wire [31:0] div_finish_r_final;
  assign div_finish_q_final = (div_is_signed && div_neg_q) ? ((~div_step_quot_next) + 32'd1) : div_step_quot_next;
  assign div_finish_r_final = (div_is_signed && div_neg_r) ? ((~div_step_rem_next) + 32'd1) : div_step_rem_next;

  wire div_start_signed;
  wire div_start_rem;
  wire div_start_byzero;
  wire div_start_overflow;
  wire div_start_dvd_neg;
  wire div_start_dvs_neg;
  wire [31:0] div_start_dvd_abs;
  wire [31:0] div_start_dvs_abs;
  wire div_start_neg_q;
  wire div_start_neg_r;
  assign div_start_signed = (funct3 == 3'b100) || (funct3 == 3'b110);
  assign div_start_rem = (funct3 == 3'b110) || (funct3 == 3'b111);
  assign div_start_byzero = (rs2_val == 32'h0);
  assign div_start_overflow = div_start_signed && (rs1_val == 32'h80000000) && (rs2_val == 32'hffffffff);
  assign div_start_dvd_neg = div_start_signed && rs1_val[31];
  assign div_start_dvs_neg = div_start_signed && rs2_val[31];
  assign div_start_dvd_abs = div_start_dvd_neg ? ((~rs1_val) + 32'd1) : rs1_val;
  assign div_start_dvs_abs = div_start_dvs_neg ? ((~rs2_val) + 32'd1) : rs2_val;
  assign div_start_neg_q = div_start_signed && (rs1_val[31] ^ rs2_val[31]);
  assign div_start_neg_r = div_start_signed && rs1_val[31];

  // Decode + execute (combinational).
  always @* begin
    // Defaults
    next_pc = pc + 32'd4;
    reg_write_en = 1'b0;
    reg_write_data = 32'h0;
    mem_write_en = 1'b0;
    mem_write_mask = 32'h0;
    mem_write_value = 32'h0;
    csr_we = 1'b0;
    csr_waddr = 12'h000;
    csr_wdata = 32'h0;
    trap_req = 1'b0;
    trap_cause = 32'h0;
    trap_tval = 32'h0;
    mret_req = 1'b0;
    uart_pulse = 1'b0;
    uart_pulse_data = 8'h00;
    start_mul = 1'b0;
    start_div = 1'b0;

    mem_addr = rs1_val + imm_i;
    sra_in = rs1_val;
    sra_shamt = 5'd0;
    byte_shift = {mem_addr[1:0], 3'b000};  // 0,8,16,24
    half_shift = {mem_addr[1], 4'b0000};   // 0,16

    // While a MUL/DIV microcode operation is running, stall fetch/execute.
    if(div_busy || mul_busy) begin
      next_pc = pc;
      reg_write_en = 1'b0;
      mem_write_en = 1'b0;
    end else begin
    // RV32I/RV32M opcodes
    // LUI (0110111)
    if(opcode == 7'b0110111) begin
      reg_write_en = 1'b1;
      reg_write_data = imm_u;
    end
    // AUIPC (0010111)
    else if(opcode == 7'b0010111) begin
      reg_write_en = 1'b1;
      reg_write_data = pc + imm_u;
    end
    // JAL (1101111)
    else if(opcode == 7'b1101111) begin
      reg_write_en = 1'b1;
      reg_write_data = pc + 32'd4;
      next_pc = pc + imm_j;
    end
    // JALR (1100111)
    else if(opcode == 7'b1100111 && funct3 == 3'b000) begin
      reg_write_en = 1'b1;
      reg_write_data = pc + 32'd4;
      next_pc = (rs1_val + imm_i) & 32'hfffffffe;
    end
    // BRANCH (1100011)
    else if(opcode == 7'b1100011) begin
      // default not taken
      if(funct3 == 3'b000) begin
        if(rs1_val == rs2_val) next_pc = pc + imm_b; // BEQ
      end else if(funct3 == 3'b001) begin
        if(rs1_val != rs2_val) next_pc = pc + imm_b; // BNE
      end else if(funct3 == 3'b100) begin
        if(rs1_s < rs2_s) next_pc = pc + imm_b; // BLT
      end else if(funct3 == 3'b101) begin
        if(rs1_s >= rs2_s) next_pc = pc + imm_b; // BGE
      end else if(funct3 == 3'b110) begin
        if(rs1_val < rs2_val) next_pc = pc + imm_b; // BLTU
      end else if(funct3 == 3'b111) begin
        if(rs1_val >= rs2_val) next_pc = pc + imm_b; // BGEU
      end
    end
    // LOAD (0000011)
    else if(opcode == 7'b0000011) begin
      // address already in mem_addr = rs1 + imm_i
      reg_write_en = 1'b1;
      if(funct3 == 3'b000) begin
        // LB
        reg_write_data = {{24{load_b[7]}}, load_b};
      end else if(funct3 == 3'b001) begin
        // LH
        reg_write_data = {{16{load_h[15]}}, load_h};
      end else if(funct3 == 3'b010) begin
        // LW
        reg_write_data = mem_rdata_word;
      end else if(funct3 == 3'b100) begin
        // LBU
        reg_write_data = {24'h0, load_b};
      end else if(funct3 == 3'b101) begin
        // LHU
        reg_write_data = {16'h0, load_h};
      end else begin
        reg_write_data = 32'h0;
      end
    end
    // STORE (0100011)
    else if(opcode == 7'b0100011) begin
      mem_addr = rs1_val + imm_s;
      mem_write_en = 1'b1;
      byte_shift = {mem_addr[1:0], 3'b000};
      half_shift = {mem_addr[1], 4'b0000};
      if(funct3 == 3'b000) begin
        // SB
        mem_write_mask = (32'h000000ff << byte_shift);
        mem_write_value = (rs2_val[7:0] << byte_shift);
      end else if(funct3 == 3'b001) begin
        // SH
        mem_write_mask = (32'h0000ffff << half_shift);
        mem_write_value = (rs2_val[15:0] << half_shift);
      end else if(funct3 == 3'b010) begin
        // SW
        mem_write_mask = 32'hffffffff;
        mem_write_value = rs2_val;
      end else begin
        mem_write_mask = 32'h0;
        mem_write_value = 32'h0;
        trap_req = 1'b1;
        trap_cause = 32'd2;  // illegal instruction
        trap_tval = instr;
      end

      // UART MMIO: any store to UART_MMIO_ADDR triggers a one-cycle pulse and does not touch RAM.
      if(mem_addr == `UART_MMIO_ADDR) begin
        mem_write_en = 1'b0;
        uart_pulse = 1'b1;
        uart_pulse_data = rs2_val[7:0];
      end
    end
    // OP-IMM (0010011)
    else if(opcode == 7'b0010011) begin
      reg_write_en = 1'b1;
      if(funct3 == 3'b000) begin
        // ADDI
        reg_write_data = rs1_val + imm_i;
      end else if(funct3 == 3'b010) begin
        // SLTI (signed)
        reg_write_data = (rs1_s < imm_i_s) ? 32'd1 : 32'd0;
      end else if(funct3 == 3'b011) begin
        // SLTIU (unsigned)
        reg_write_data = (rs1_val < imm_i) ? 32'd1 : 32'd0;
      end else if(funct3 == 3'b100) begin
        // XORI
        reg_write_data = rs1_val ^ imm_i;
      end else if(funct3 == 3'b110) begin
        // ORI
        reg_write_data = rs1_val | imm_i;
      end else if(funct3 == 3'b111) begin
        // ANDI
        reg_write_data = rs1_val & imm_i;
      end else if(funct3 == 3'b001) begin
        // SLLI (funct7==0000000)
        reg_write_data = rs1_val << instr[24:20];
      end else if(funct3 == 3'b101) begin
        // SRLI/SRAI (funct7 distinguishes)
        if(instr[30]) begin
          sra_in = rs1_val;
          sra_shamt = instr[24:20];
          reg_write_data = sra_out;
        end else begin
          reg_write_data = rs1_val >> instr[24:20];
        end
      end else begin
        reg_write_data = 32'h0;
      end
    end
    // OP (0110011)
    else if(opcode == 7'b0110011) begin
	      // RV32M uses funct7=0000001 for MUL/DIV/REM family.
	      if(funct7 == 7'b0000001) begin
	        if(funct3 == 3'b000 || funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b011) begin
	          // MUL/MULH/MULHSU/MULHU are handled by a multi-cycle multiplier.
	          start_mul = 1'b1;
	          next_pc = pc;
	        end else if(funct3 == 3'b100 || funct3 == 3'b101 || funct3 == 3'b110 || funct3 == 3'b111) begin
	          // DIV/DIVU/REM/REMU are handled by a multi-cycle divider.
	          start_div = 1'b1;
	          next_pc = pc;
	        end else begin
          trap_req = 1'b1;
          trap_cause = 32'd2;
          trap_tval = instr;
        end
      end else begin
        reg_write_en = 1'b1;
        if(funct3 == 3'b000) begin
          // ADD/SUB
          if(instr[30]) reg_write_data = rs1_val - rs2_val;
          else reg_write_data = rs1_val + rs2_val;
        end else if(funct3 == 3'b001) begin
          // SLL
          reg_write_data = rs1_val << rs2_val[4:0];
        end else if(funct3 == 3'b010) begin
          // SLT (signed)
          reg_write_data = (rs1_s < rs2_s) ? 32'd1 : 32'd0;
        end else if(funct3 == 3'b011) begin
          // SLTU
          reg_write_data = (rs1_val < rs2_val) ? 32'd1 : 32'd0;
        end else if(funct3 == 3'b100) begin
          // XOR
          reg_write_data = rs1_val ^ rs2_val;
        end else if(funct3 == 3'b101) begin
          // SRL/SRA
          if(instr[30]) begin
            sra_in = rs1_val;
            sra_shamt = rs2_val[4:0];
            reg_write_data = sra_out;
          end else begin
            reg_write_data = rs1_val >> rs2_val[4:0];
          end
        end else if(funct3 == 3'b110) begin
          // OR
          reg_write_data = rs1_val | rs2_val;
        end else if(funct3 == 3'b111) begin
          // AND
          reg_write_data = rs1_val & rs2_val;
        end else begin
          reg_write_data = 32'h0;
          trap_req = 1'b1;
          trap_cause = 32'd2;
          trap_tval = instr;
        end
      end
    end
    // SYSTEM (1110011)
    else if(opcode == 7'b1110011) begin
      // CSR / traps / mret.
      if(is_mret) begin
        mret_req = 1'b1;
        next_pc = pc;
      end else if(is_ecall) begin
        trap_req = 1'b1;
        trap_cause = 32'd11; // ecall from M-mode
        trap_tval = 32'h0;
        next_pc = pc;
      end else if(is_ebreak) begin
        trap_req = 1'b1;
        trap_cause = 32'd3;  // breakpoint
        trap_tval = 32'h0;
        next_pc = pc;
      end else if(funct3 == 3'b001 || funct3 == 3'b010 || funct3 == 3'b011 ||
                  funct3 == 3'b101 || funct3 == 3'b110 || funct3 == 3'b111) begin
        // CSRxx
        reg_write_en = 1'b1;
        reg_write_data = csr_rdata;
        csr_we = 1'b1;
        csr_waddr = csr_addr;
        // RS1 or ZIMM
        if(funct3 == 3'b001) begin
          // CSRRW
          csr_wdata = rs1_val;
        end else if(funct3 == 3'b010) begin
          // CSRRS
          if(rs1 != 5'd0) csr_wdata = csr_rdata | rs1_val;
          else csr_we = 1'b0;
        end else if(funct3 == 3'b011) begin
          // CSRRC
          if(rs1 != 5'd0) csr_wdata = csr_rdata & ~rs1_val;
          else csr_we = 1'b0;
        end else if(funct3 == 3'b101) begin
          // CSRRWI
          csr_wdata = {27'h0, rs1};
        end else if(funct3 == 3'b110) begin
          // CSRRSI
          if(rs1 != 5'd0) csr_wdata = csr_rdata | {27'h0, rs1};
          else csr_we = 1'b0;
        end else if(funct3 == 3'b111) begin
          // CSRRCI
          if(rs1 != 5'd0) csr_wdata = csr_rdata & ~{27'h0, rs1};
          else csr_we = 1'b0;
        end
      end else begin
        trap_req = 1'b1;
        trap_cause = 32'd2;
        trap_tval = instr;
        next_pc = pc;
      end
    end
    // FENCE (0001111) and unknowns -> NOP.
    else begin
      if(opcode == 7'b0001111) begin
        reg_write_en = 1'b0; // FENCE/FENCE.I treated as NOP
      end else begin
        trap_req = 1'b1;
        trap_cause = 32'd2;
        trap_tval = instr;
        next_pc = pc;
      end
    end
    end
  end

  // Commit (sequential): PC, registers, and memory writes.
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      pc <= `RESET_PC;
      uart_valid_r <= 1'b0;
      uart_data_r <= 8'h00;

      csr_mstatus <= 32'h0;
      csr_mtvec <= 32'h0;
      csr_mepc <= 32'h0;
      csr_mcause <= 32'h0;
      csr_mtval <= 32'h0;
      csr_mcycle <= 32'h0;
      csr_minstret <= 32'h0;

      div_busy <= 1'b0;
      div_cnt <= 6'd0;
      div_dvd <= 32'h0;
      div_dvs <= 32'h0;
      div_rem <= 32'h0;
      div_quot <= 32'h0;
      div_rd <= 5'd0;
      div_is_signed <= 1'b0;
      div_is_rem <= 1'b0;
      div_neg_q <= 1'b0;
      div_neg_r <= 1'b0;

      mul_busy <= 1'b0;
      mul_cnt <= 6'd0;
      mul_acc <= 64'h0;
      mul_a <= 64'h0;
      mul_b <= 32'h0;
      mul_rd <= 5'd0;
      mul_take_high <= 1'b0;
      mul_neg <= 1'b0;

      x1 <= 32'h0;
      x2 <= 32'h0;
      x3 <= 32'h0;
      x4 <= 32'h0;
      x5 <= 32'h0;
      x6 <= 32'h0;
      x7 <= 32'h0;
      x8 <= 32'h0;
      x9 <= 32'h0;
      x10 <= 32'h0;
      x11 <= 32'h0;
      x12 <= 32'h0;
      x13 <= 32'h0;
      x14 <= 32'h0;
      x15 <= 32'h0;
      x16 <= 32'h0;
      x17 <= 32'h0;
      x18 <= 32'h0;
      x19 <= 32'h0;
      x20 <= 32'h0;
      x21 <= 32'h0;
      x22 <= 32'h0;
      x23 <= 32'h0;
      x24 <= 32'h0;
      x25 <= 32'h0;
      x26 <= 32'h0;
      x27 <= 32'h0;
      x28 <= 32'h0;
      x29 <= 32'h0;
      x30 <= 32'h0;
      x31 <= 32'h0;

      mem0 <= 32'h0;
      mem1 <= 32'h0;
      mem2 <= 32'h0;
      mem3 <= 32'h0;
      mem4 <= 32'h0;
      mem5 <= 32'h0;
      mem6 <= 32'h0;
      mem7 <= 32'h0;
      mem8 <= 32'h0;
      mem9 <= 32'h0;
      mem10 <= 32'h0;
      mem11 <= 32'h0;
      mem12 <= 32'h0;
      mem13 <= 32'h0;
      mem14 <= 32'h0;
      mem15 <= 32'h0;
    end else begin
      // default pulse is 0; raise for one cycle on MMIO store.
      uart_valid_r <= 1'b0;
      if(uart_pulse) begin
        uart_valid_r <= 1'b1;
        uart_data_r <= uart_pulse_data;
      end

      // counters
      csr_mcycle <= csr_mcycle + 32'd1;

      // Divider microcode has highest priority (stalls fetch/execute).
	      if(div_busy) begin
	        if(div_cnt == 6'd31) begin
          if(div_rd != 5'd0) begin
            if(div_is_rem) begin
              case(div_rd)
                5'd1: x1 <= div_finish_r_final;
                5'd2: x2 <= div_finish_r_final;
                5'd3: x3 <= div_finish_r_final;
                5'd4: x4 <= div_finish_r_final;
                5'd5: x5 <= div_finish_r_final;
                5'd6: x6 <= div_finish_r_final;
                5'd7: x7 <= div_finish_r_final;
                5'd8: x8 <= div_finish_r_final;
                5'd9: x9 <= div_finish_r_final;
                5'd10: x10 <= div_finish_r_final;
                5'd11: x11 <= div_finish_r_final;
                5'd12: x12 <= div_finish_r_final;
                5'd13: x13 <= div_finish_r_final;
                5'd14: x14 <= div_finish_r_final;
                5'd15: x15 <= div_finish_r_final;
                5'd16: x16 <= div_finish_r_final;
                5'd17: x17 <= div_finish_r_final;
                5'd18: x18 <= div_finish_r_final;
                5'd19: x19 <= div_finish_r_final;
                5'd20: x20 <= div_finish_r_final;
                5'd21: x21 <= div_finish_r_final;
                5'd22: x22 <= div_finish_r_final;
                5'd23: x23 <= div_finish_r_final;
                5'd24: x24 <= div_finish_r_final;
                5'd25: x25 <= div_finish_r_final;
                5'd26: x26 <= div_finish_r_final;
                5'd27: x27 <= div_finish_r_final;
                5'd28: x28 <= div_finish_r_final;
                5'd29: x29 <= div_finish_r_final;
                5'd30: x30 <= div_finish_r_final;
                5'd31: x31 <= div_finish_r_final;
                default: begin end
              endcase
            end else begin
              case(div_rd)
                5'd1: x1 <= div_finish_q_final;
                5'd2: x2 <= div_finish_q_final;
                5'd3: x3 <= div_finish_q_final;
                5'd4: x4 <= div_finish_q_final;
                5'd5: x5 <= div_finish_q_final;
                5'd6: x6 <= div_finish_q_final;
                5'd7: x7 <= div_finish_q_final;
                5'd8: x8 <= div_finish_q_final;
                5'd9: x9 <= div_finish_q_final;
                5'd10: x10 <= div_finish_q_final;
                5'd11: x11 <= div_finish_q_final;
                5'd12: x12 <= div_finish_q_final;
                5'd13: x13 <= div_finish_q_final;
                5'd14: x14 <= div_finish_q_final;
                5'd15: x15 <= div_finish_q_final;
                5'd16: x16 <= div_finish_q_final;
                5'd17: x17 <= div_finish_q_final;
                5'd18: x18 <= div_finish_q_final;
                5'd19: x19 <= div_finish_q_final;
                5'd20: x20 <= div_finish_q_final;
                5'd21: x21 <= div_finish_q_final;
                5'd22: x22 <= div_finish_q_final;
                5'd23: x23 <= div_finish_q_final;
                5'd24: x24 <= div_finish_q_final;
                5'd25: x25 <= div_finish_q_final;
                5'd26: x26 <= div_finish_q_final;
                5'd27: x27 <= div_finish_q_final;
                5'd28: x28 <= div_finish_q_final;
                5'd29: x29 <= div_finish_q_final;
                5'd30: x30 <= div_finish_q_final;
                5'd31: x31 <= div_finish_q_final;
                default: begin end
              endcase
            end
          end

          div_busy <= 1'b0;
          div_cnt <= 6'd0;
          pc <= pc + 32'd4;
          csr_minstret <= csr_minstret + 32'd1;
	        end else begin
	          div_cnt <= div_cnt + 6'd1;
	          div_dvd <= div_dvd_shift;
	          div_quot <= div_quot_shift;
	          if(div_ge) div_rem <= div_rem_sub;
	          else div_rem <= div_rem_shift;
	          pc <= pc;  // stall
	        end
	      end
	      // Multiplier microcode (MUL/MULH/MULHSU/MULHU).
	      else if(mul_busy) begin
	        if(mul_cnt == 6'd31) begin
	          if(mul_rd != 5'd0) begin
	            case(mul_rd)
	              5'd1: x1 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd2: x2 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd3: x3 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd4: x4 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd5: x5 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd6: x6 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd7: x7 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd8: x8 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd9: x9 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd10: x10 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd11: x11 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd12: x12 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd13: x13 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd14: x14 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd15: x15 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd16: x16 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd17: x17 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd18: x18 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd19: x19 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd20: x20 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd21: x21 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd22: x22 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd23: x23 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd24: x24 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd25: x25 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd26: x26 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd27: x27 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd28: x28 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd29: x29 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd30: x30 <= (mul_take_high ? mul_result_high : mul_result_low);
	              5'd31: x31 <= (mul_take_high ? mul_result_high : mul_result_low);
	              default: begin end
	            endcase
	          end

	          mul_busy <= 1'b0;
	          mul_cnt <= 6'd0;
	          pc <= pc + 32'd4;
	          csr_minstret <= csr_minstret + 32'd1;
	        end else begin
	          mul_cnt <= mul_cnt + 6'd1;
	          mul_acc <= mul_acc_next;
	          mul_a <= mul_a_next;
	          mul_b <= mul_b_next;
	          pc <= pc;  // stall
	        end
	      end
	      else if(start_mul) begin
	        mul_busy <= 1'b1;
	        mul_cnt <= 6'd0;
	        mul_acc <= 64'h0;
	        mul_rd <= rd;
	        pc <= pc;  // stall

	        if(funct3 == 3'b000) begin
	          // MUL (low 32 bits; sign doesn't affect low half)
	          mul_take_high <= 1'b0;
	          mul_neg <= 1'b0;
	          mul_a <= {32'h0, rs1_val};
	          mul_b <= rs2_val;
	        end else if(funct3 == 3'b011) begin
	          // MULHU
	          mul_take_high <= 1'b1;
	          mul_neg <= 1'b0;
	          mul_a <= {32'h0, rs1_val};
	          mul_b <= rs2_val;
	        end else if(funct3 == 3'b010) begin
	          // MULHSU
	          mul_take_high <= 1'b1;
	          mul_neg <= rs1_val[31];
	          mul_a <= {32'h0, (rs1_val[31] ? ((~rs1_val) + 32'd1) : rs1_val)};
	          mul_b <= rs2_val;
	        end else begin
	          // MULH
	          mul_take_high <= 1'b1;
	          mul_neg <= rs1_val[31] ^ rs2_val[31];
	          mul_a <= {32'h0, (rs1_val[31] ? ((~rs1_val) + 32'd1) : rs1_val)};
	          mul_b <= (rs2_val[31] ? ((~rs2_val) + 32'd1) : rs2_val);
	        end
	      end
	      // Start a new DIV/REM operation (multi-cycle).
	      else if(start_div) begin
	        if(div_start_byzero) begin
          // Division by zero: quotient=-1, remainder=dividend.
          if(rd != 5'd0) begin
            if(div_start_rem) begin
              case(rd)
                5'd1: x1 <= rs1_val;
                5'd2: x2 <= rs1_val;
                5'd3: x3 <= rs1_val;
                5'd4: x4 <= rs1_val;
                5'd5: x5 <= rs1_val;
                5'd6: x6 <= rs1_val;
                5'd7: x7 <= rs1_val;
                5'd8: x8 <= rs1_val;
                5'd9: x9 <= rs1_val;
                5'd10: x10 <= rs1_val;
                5'd11: x11 <= rs1_val;
                5'd12: x12 <= rs1_val;
                5'd13: x13 <= rs1_val;
                5'd14: x14 <= rs1_val;
                5'd15: x15 <= rs1_val;
                5'd16: x16 <= rs1_val;
                5'd17: x17 <= rs1_val;
                5'd18: x18 <= rs1_val;
                5'd19: x19 <= rs1_val;
                5'd20: x20 <= rs1_val;
                5'd21: x21 <= rs1_val;
                5'd22: x22 <= rs1_val;
                5'd23: x23 <= rs1_val;
                5'd24: x24 <= rs1_val;
                5'd25: x25 <= rs1_val;
                5'd26: x26 <= rs1_val;
                5'd27: x27 <= rs1_val;
                5'd28: x28 <= rs1_val;
                5'd29: x29 <= rs1_val;
                5'd30: x30 <= rs1_val;
                5'd31: x31 <= rs1_val;
                default: begin end
              endcase
            end else begin
              case(rd)
                5'd1: x1 <= 32'hffffffff;
                5'd2: x2 <= 32'hffffffff;
                5'd3: x3 <= 32'hffffffff;
                5'd4: x4 <= 32'hffffffff;
                5'd5: x5 <= 32'hffffffff;
                5'd6: x6 <= 32'hffffffff;
                5'd7: x7 <= 32'hffffffff;
                5'd8: x8 <= 32'hffffffff;
                5'd9: x9 <= 32'hffffffff;
                5'd10: x10 <= 32'hffffffff;
                5'd11: x11 <= 32'hffffffff;
                5'd12: x12 <= 32'hffffffff;
                5'd13: x13 <= 32'hffffffff;
                5'd14: x14 <= 32'hffffffff;
                5'd15: x15 <= 32'hffffffff;
                5'd16: x16 <= 32'hffffffff;
                5'd17: x17 <= 32'hffffffff;
                5'd18: x18 <= 32'hffffffff;
                5'd19: x19 <= 32'hffffffff;
                5'd20: x20 <= 32'hffffffff;
                5'd21: x21 <= 32'hffffffff;
                5'd22: x22 <= 32'hffffffff;
                5'd23: x23 <= 32'hffffffff;
                5'd24: x24 <= 32'hffffffff;
                5'd25: x25 <= 32'hffffffff;
                5'd26: x26 <= 32'hffffffff;
                5'd27: x27 <= 32'hffffffff;
                5'd28: x28 <= 32'hffffffff;
                5'd29: x29 <= 32'hffffffff;
                5'd30: x30 <= 32'hffffffff;
                5'd31: x31 <= 32'hffffffff;
                default: begin end
              endcase
            end
          end
          pc <= pc + 32'd4;
          csr_minstret <= csr_minstret + 32'd1;
        end else if(div_start_overflow) begin
          if(rd != 5'd0) begin
            if(div_start_rem) begin
              case(rd)
                5'd1: x1 <= 32'h0;
                5'd2: x2 <= 32'h0;
                5'd3: x3 <= 32'h0;
                5'd4: x4 <= 32'h0;
                5'd5: x5 <= 32'h0;
                5'd6: x6 <= 32'h0;
                5'd7: x7 <= 32'h0;
                5'd8: x8 <= 32'h0;
                5'd9: x9 <= 32'h0;
                5'd10: x10 <= 32'h0;
                5'd11: x11 <= 32'h0;
                5'd12: x12 <= 32'h0;
                5'd13: x13 <= 32'h0;
                5'd14: x14 <= 32'h0;
                5'd15: x15 <= 32'h0;
                5'd16: x16 <= 32'h0;
                5'd17: x17 <= 32'h0;
                5'd18: x18 <= 32'h0;
                5'd19: x19 <= 32'h0;
                5'd20: x20 <= 32'h0;
                5'd21: x21 <= 32'h0;
                5'd22: x22 <= 32'h0;
                5'd23: x23 <= 32'h0;
                5'd24: x24 <= 32'h0;
                5'd25: x25 <= 32'h0;
                5'd26: x26 <= 32'h0;
                5'd27: x27 <= 32'h0;
                5'd28: x28 <= 32'h0;
                5'd29: x29 <= 32'h0;
                5'd30: x30 <= 32'h0;
                5'd31: x31 <= 32'h0;
                default: begin end
              endcase
            end else begin
              case(rd)
                5'd1: x1 <= 32'h80000000;
                5'd2: x2 <= 32'h80000000;
                5'd3: x3 <= 32'h80000000;
                5'd4: x4 <= 32'h80000000;
                5'd5: x5 <= 32'h80000000;
                5'd6: x6 <= 32'h80000000;
                5'd7: x7 <= 32'h80000000;
                5'd8: x8 <= 32'h80000000;
                5'd9: x9 <= 32'h80000000;
                5'd10: x10 <= 32'h80000000;
                5'd11: x11 <= 32'h80000000;
                5'd12: x12 <= 32'h80000000;
                5'd13: x13 <= 32'h80000000;
                5'd14: x14 <= 32'h80000000;
                5'd15: x15 <= 32'h80000000;
                5'd16: x16 <= 32'h80000000;
                5'd17: x17 <= 32'h80000000;
                5'd18: x18 <= 32'h80000000;
                5'd19: x19 <= 32'h80000000;
                5'd20: x20 <= 32'h80000000;
                5'd21: x21 <= 32'h80000000;
                5'd22: x22 <= 32'h80000000;
                5'd23: x23 <= 32'h80000000;
                5'd24: x24 <= 32'h80000000;
                5'd25: x25 <= 32'h80000000;
                5'd26: x26 <= 32'h80000000;
                5'd27: x27 <= 32'h80000000;
                5'd28: x28 <= 32'h80000000;
                5'd29: x29 <= 32'h80000000;
                5'd30: x30 <= 32'h80000000;
                5'd31: x31 <= 32'h80000000;
                default: begin end
              endcase
            end
          end
          pc <= pc + 32'd4;
          csr_minstret <= csr_minstret + 32'd1;
        end else begin
          div_busy <= 1'b1;
          div_cnt <= 6'd0;
          div_dvd <= div_start_dvd_abs;
          div_dvs <= div_start_dvs_abs;
          div_rem <= 32'h0;
          div_quot <= 32'h0;
          div_rd <= rd;
          div_is_signed <= div_start_signed;
          div_is_rem <= div_start_rem;
          div_neg_q <= div_start_neg_q;
          div_neg_r <= div_start_neg_r;
          pc <= pc;  // stall
        end
      end
      // Trap has priority over normal execution.
      else if(trap_req) begin
        csr_mepc <= pc;
        csr_mcause <= trap_cause;
        csr_mtval <= trap_tval;
        pc <= csr_mtvec;
      end
      // Return from trap.
      else if(mret_req) begin
        pc <= csr_mepc;
        csr_minstret <= csr_minstret + 32'd1;
      end
      else begin
        pc <= next_pc;

      if(reg_write_en && (rd != 5'd0)) begin
        case(rd)
          5'd1: x1 <= reg_write_data;
          5'd2: x2 <= reg_write_data;
          5'd3: x3 <= reg_write_data;
          5'd4: x4 <= reg_write_data;
          5'd5: x5 <= reg_write_data;
          5'd6: x6 <= reg_write_data;
          5'd7: x7 <= reg_write_data;
          5'd8: x8 <= reg_write_data;
          5'd9: x9 <= reg_write_data;
          5'd10: x10 <= reg_write_data;
          5'd11: x11 <= reg_write_data;
          5'd12: x12 <= reg_write_data;
          5'd13: x13 <= reg_write_data;
          5'd14: x14 <= reg_write_data;
          5'd15: x15 <= reg_write_data;
          5'd16: x16 <= reg_write_data;
          5'd17: x17 <= reg_write_data;
          5'd18: x18 <= reg_write_data;
          5'd19: x19 <= reg_write_data;
          5'd20: x20 <= reg_write_data;
          5'd21: x21 <= reg_write_data;
          5'd22: x22 <= reg_write_data;
          5'd23: x23 <= reg_write_data;
          5'd24: x24 <= reg_write_data;
          5'd25: x25 <= reg_write_data;
          5'd26: x26 <= reg_write_data;
          5'd27: x27 <= reg_write_data;
          5'd28: x28 <= reg_write_data;
          5'd29: x29 <= reg_write_data;
          5'd30: x30 <= reg_write_data;
          5'd31: x31 <= reg_write_data;
          default: begin end
        endcase
      end

      if(mem_write_en) begin
        case(mem_addr[5:2])
          4'd0: mem0 <= (mem0 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd1: mem1 <= (mem1 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd2: mem2 <= (mem2 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd3: mem3 <= (mem3 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd4: mem4 <= (mem4 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd5: mem5 <= (mem5 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd6: mem6 <= (mem6 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd7: mem7 <= (mem7 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd8: mem8 <= (mem8 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd9: mem9 <= (mem9 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd10: mem10 <= (mem10 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd11: mem11 <= (mem11 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd12: mem12 <= (mem12 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd13: mem13 <= (mem13 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd14: mem14 <= (mem14 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          4'd15: mem15 <= (mem15 & ~mem_write_mask) | (mem_write_value & mem_write_mask);
          default: begin end
        endcase
      end

        // CSR writeback (best-effort; unknown CSR addresses are ignored).
        if(csr_we) begin
          if(csr_waddr == 12'h300) csr_mstatus <= csr_wdata;
          else if(csr_waddr == 12'h305) csr_mtvec <= csr_wdata;
          else if(csr_waddr == 12'h341) csr_mepc <= csr_wdata;
          else if(csr_waddr == 12'h342) csr_mcause <= csr_wdata;
          else if(csr_waddr == 12'h343) csr_mtval <= csr_wdata;
          else if(csr_waddr == 12'hb00) csr_mcycle <= csr_wdata;
          else if(csr_waddr == 12'hb02) csr_minstret <= csr_wdata;
        end

        // Retired instruction counter.
        csr_minstret <= csr_minstret + 32'd1;
      end
    end
  end
endmodule
