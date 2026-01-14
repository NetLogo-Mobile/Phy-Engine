module control16(
    input  wire [3:0] opcode,
    input  wire [1:0] reg_dst,
    input  wire [1:0] reg_src,
    input  wire [7:0] imm8,
    input  wire [7:0] pc,
    input  wire       flag_z,
    input  wire       flag_c,
    input  wire       flag_s,
    output reg  [7:0] pc_next,
    output reg        pc_we,
    output reg        reg_we,
    output reg  [1:0] rf_waddr,
    output reg  [1:0] rf_raddr_a,
    output reg  [1:0] rf_raddr_b,
    output reg        alu_b_sel,   // 0: imm16, 1: reg_src
    output reg        flags_we_z,
    output reg        flags_we_c,
    output reg        flags_we_s,
    output reg  [2:0] alu_op,
    output reg        halt
);
    always @(*) begin
        pc_next  = pc + 8'd1;
        pc_we    = 1'b1;
        reg_we   = 1'b0;
        rf_waddr = reg_dst;
        rf_raddr_a = reg_dst;
        rf_raddr_b = reg_src;
        alu_b_sel = 1'b0;
        flags_we_z = 1'b0;
        flags_we_c = 1'b0;
        flags_we_s = 1'b0;
        alu_op   = 3'd0;
        halt     = 1'b0;

        case (opcode)
            4'h0: begin
                // EXT / NOP (use imm8[7:4] as sub-opcode; imm8[3:0] as small immediate)
                //   0x0?: NOP
                //   0x1s: SHLI dst, shamt4
                //   0x2s: SHRI dst, shamt4
                //   0x3?: SHLR dst, src   (shift-left by R[src][3:0])
                //   0x4?: SHRR dst, src   (shift-right by R[src][3:0])
                case (imm8[7:4])
                    4'h0: begin
                        // NOP
                    end
                    4'h1: begin
                        // SHLI dst, imm4
                        reg_we = 1'b1;
                        alu_op = 3'd6;
                        alu_b_sel = 1'b0;
                        flags_we_z = 1'b1;
                        flags_we_c = 1'b1;
                        flags_we_s = 1'b1;
                    end
                    4'h2: begin
                        // SHRI dst, imm4
                        reg_we = 1'b1;
                        alu_op = 3'd7;
                        alu_b_sel = 1'b0;
                        flags_we_z = 1'b1;
                        flags_we_c = 1'b1;
                        flags_we_s = 1'b1;
                    end
                    4'h3: begin
                        // SHLR dst, src
                        reg_we = 1'b1;
                        alu_op = 3'd6;
                        alu_b_sel = 1'b1;
                        flags_we_z = 1'b1;
                        flags_we_c = 1'b1;
                        flags_we_s = 1'b1;
                    end
                    4'h4: begin
                        // SHRR dst, src
                        reg_we = 1'b1;
                        alu_op = 3'd7;
                        alu_b_sel = 1'b1;
                        flags_we_z = 1'b1;
                        flags_we_c = 1'b1;
                        flags_we_s = 1'b1;
                    end
                    default: begin
                        // undefined -> NOP
                    end
                endcase
            end
            4'h1: begin
                // MOVI reg, imm8
                reg_we = 1'b1;
                alu_op = 3'd5;
                alu_b_sel = 1'b0;
            end
            4'h2: begin
                // ADDI reg, imm8
                reg_we   = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
                alu_op   = 3'd0;
                alu_b_sel = 1'b0;
            end
            4'h3: begin
                // XORI reg, imm8
                reg_we   = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
                alu_op   = 3'd4;
                alu_b_sel = 1'b0;
            end
            4'h4: begin
                // JMP imm8
                pc_next = imm8;
            end
            4'h5: begin
                // JZ imm8 (uses previous Z flag)
                if(flag_z) begin
                    pc_next = imm8;
                end
            end
            4'h6: begin
                // MOVR dst, src
                reg_we = 1'b1;
                alu_op = 3'd5;
                alu_b_sel = 1'b1;
            end
            4'h7: begin
                // ADDR dst, src
                reg_we   = 1'b1;
                alu_op   = 3'd0;
                alu_b_sel = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'h8: begin
                // SUBR dst, src
                reg_we   = 1'b1;
                alu_op   = 3'd1;
                alu_b_sel = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'h9: begin
                // ANDR dst, src
                reg_we   = 1'b1;
                alu_op   = 3'd2;
                alu_b_sel = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'hA: begin
                // ORR dst, src
                reg_we   = 1'b1;
                alu_op   = 3'd3;
                alu_b_sel = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'hB: begin
                // XORR dst, src
                reg_we   = 1'b1;
                alu_op   = 3'd4;
                alu_b_sel = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'hC: begin
                // CMPR dst, src  (flags <- dst - src)
                reg_we   = 1'b0;
                alu_op   = 3'd1;
                alu_b_sel = 1'b1;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'hD: begin
                // JNZ imm8
                if(!flag_z) begin
                    pc_next = imm8;
                end
            end
            4'hE: begin
                // SUBI reg, imm8
                reg_we   = 1'b1;
                alu_op   = 3'd1;
                alu_b_sel = 1'b0;
                flags_we_z = 1'b1;
                flags_we_c = 1'b1;
                flags_we_s = 1'b1;
            end
            4'hF: begin
                // HLT
                halt  = 1'b1;
                pc_we = 1'b0;
            end
            default: begin
                // undefined -> NOP
            end
        endcase
    end
endmodule
