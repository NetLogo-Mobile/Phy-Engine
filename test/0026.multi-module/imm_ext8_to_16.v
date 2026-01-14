module imm_ext8_to_16(
    input  wire [7:0]  imm8,
    output wire [15:0] imm16
);
    assign imm16 = {{8{imm8[7]}}, imm8};
endmodule

