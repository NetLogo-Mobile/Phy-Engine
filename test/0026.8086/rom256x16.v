module rom256x16(
    input  wire [7:0]  addr,
    output reg  [15:0] data
);
    always @(*) begin
        case (addr)
            // Encoding:
            //   [15:12]=opcode
            //   [11:10]=reg_dst
            //   [9:8]=reg_src
            //   [7:0]=imm8/addr8
            //
            // Program:
            //   R0 = 5
            //   R1 = 7
            //   R2 = 1
            //   SHL R2, 1
            //   SHR R2, 1
            //   R0 = R0 + R1
            //   CMP R0, R1   (Z=0)
            //   JZ skip
            //   R0 = R0 - 12 (=>0, Z=1)
            //   JZ halt
            // skip:
            //   R1 = 0x55    (should be skipped)
            // halt:
            //   HLT
            8'h00: data = 16'h1005; // MOVI R0, 5
            8'h01: data = 16'h1407; // MOVI R1, 7
            8'h02: data = 16'h1801; // MOVI R2, 1
            8'h03: data = 16'h0811; // EXT: SHLI R2, 1   (imm8[7:4]=1, shamt=1)
            8'h04: data = 16'h0821; // EXT: SHRI R2, 1   (imm8[7:4]=2, shamt=1)
            8'h05: data = 16'h7100; // ADDR R0, R1
            8'h06: data = 16'hC100; // CMPR R0, R1
            8'h07: data = 16'h500A; // JZ 0x0A
            8'h08: data = 16'hE00C; // SUBI R0, 12
            8'h09: data = 16'h500C; // JZ 0x0C
            8'h0A: data = 16'h1455; // MOVI R1, 0x55 (should be skipped)
            8'h0B: data = 16'h400C; // JMP 0x0C
            8'h0C: data = 16'hF000; // HLT
            default: data = 16'h0000;
        endcase
    end
endmodule
