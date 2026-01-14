module decode16(
    input  wire [15:0] instr,
    output wire [3:0]  opcode,
    output wire [1:0]  reg_sel,
    output wire [7:0]  imm8
);
    assign opcode  = instr[15:12];
    assign reg_sel = instr[9:8];
    assign imm8    = instr[7:0];
endmodule

