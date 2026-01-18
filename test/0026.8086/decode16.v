module decode16(
    input  wire [15:0] instr,
    output wire [3:0]  opcode,
    output wire [1:0]  reg_dst,
    output wire [1:0]  reg_src,
    output wire [7:0]  imm8
);
    assign opcode  = instr[15:12];
    assign reg_dst = instr[11:10];
    assign reg_src = instr[9:8];
    assign imm8    = instr[7:0];
endmodule
