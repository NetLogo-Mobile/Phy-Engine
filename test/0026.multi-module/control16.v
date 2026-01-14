module control16(
    input  wire [3:0] opcode,
    input  wire [1:0] reg_sel,
    input  wire [7:0] imm8,
    input  wire [7:0] pc,
    input  wire       flag_z,
    output reg  [7:0] pc_next,
    output reg        pc_we,
    output reg        reg_we,
    output reg        flags_we,
    output reg  [2:0] alu_op,
    output reg        halt
);
    always @(*) begin
        pc_next  = pc + 8'd1;
        pc_we    = 1'b1;
        reg_we   = 1'b0;
        flags_we = 1'b0;
        alu_op   = 3'd0;
        halt     = 1'b0;

        case (opcode)
            4'h0: begin
                // NOP
            end
            4'h1: begin
                // MOVI reg, imm8
                reg_we = 1'b1;
                alu_op = 3'd5;
            end
            4'h2: begin
                // ADDI reg, imm8
                reg_we   = 1'b1;
                flags_we = 1'b1;
                alu_op   = 3'd0;
            end
            4'h3: begin
                // XORI reg, imm8
                reg_we   = 1'b1;
                flags_we = 1'b1;
                alu_op   = 3'd4;
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

