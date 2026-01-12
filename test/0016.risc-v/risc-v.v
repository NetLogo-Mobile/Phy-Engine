// RV32I (unprivileged) single-cycle core implemented in a restricted Verilog subset
// (no memories/arrays), intended for Verilog->PE netlist synthesis tests.
//
// - ISA: RV32I base integer (most common opcodes), plus SYSTEM treated as NOP except ECALL/EBREAK.
// - Memory: tiny internal data RAM implemented as 16x32-bit registers (word addressed by addr[5:2]).
// - Instruction ROM: hard-coded program via case(pc[6:2]) (no $readmemh).
// - Register file: x1..x31 explicit regs; x0 is hard-wired to 0 via decode mux.
//
// This is not a cycle-accurate micro-architectural model; it's a synthesizable functional core for logic synthesis.

module riscv_top(input clk, input rst_n, output done);
  // Program counter and fetched instruction.
  reg [31:0] pc;
  reg [31:0] instr;

  // done is driven by x10[0] (a0 bit0) for the test harness.
  // If the program writes x10=1, done becomes 1.
  wire done_w;
  assign done = done_w;

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

  // Instruction ROM (word-addressed by pc[6:2]; 32 words max).
  // Program:
  //   x1=10; x2=20; x3=x1+x2=30;
  //   sw x3,0(x0); lw x4,0(x0);
  //   if (x4==30) x10=1 else x10=0;
  //   loop forever.
  always @* begin
    instr = 32'h00000013; // NOP (addi x0,x0,0)
    case(pc[6:2])
      5'd0: instr = 32'h00a00093; // addi x1, x0, 10
      5'd1: instr = 32'h01400113; // addi x2, x0, 20
      5'd2: instr = 32'h002081b3; // add  x3, x1, x2
      5'd3: instr = 32'h00302023; // sw   x3, 0(x0)
      5'd4: instr = 32'h00002203; // lw   x4, 0(x0)
      5'd5: instr = 32'h01e00293; // addi x5, x0, 30
      5'd6: instr = 32'h00520663; // beq  x4, x5, +12 (to index 9)
      5'd7: instr = 32'h00000513; // addi x10, x0, 0
      5'd8: instr = 32'h0000006f; // jal  x0, 0  (loop)
      5'd9: instr = 32'h00100513; // addi x10, x0, 1
      5'd10: instr = 32'h0000006f; // jal x0, 0
      default: instr = 32'h00000013;
    endcase
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

  // Decode + execute (combinational).
  always @* begin
    // Defaults
    next_pc = pc + 32'd4;
    reg_write_en = 1'b0;
    reg_write_data = 32'h0;
    mem_write_en = 1'b0;
    mem_write_mask = 32'h0;
    mem_write_value = 32'h0;

    mem_addr = rs1_val + imm_i;
    sra_in = rs1_val;
    sra_shamt = 5'd0;
    byte_shift = {mem_addr[1:0], 3'b000};  // 0,8,16,24
    half_shift = {mem_addr[1], 4'b0000};   // 0,16

    // RV32I opcodes
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
      end
    end
    // SYSTEM (1110011)
    else if(opcode == 7'b1110011) begin
      // This test core does not implement CSRs/traps.
      // Treat as NOP; software can still communicate by writing x10.
      reg_write_en = 1'b0;
    end
    // FENCE (0001111) and unknowns -> NOP.
    else begin
      reg_write_en = 1'b0;
    end
  end

  // Commit (sequential): PC, registers, and memory writes.
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
      pc <= 32'h0;

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
    end
  end
endmodule
