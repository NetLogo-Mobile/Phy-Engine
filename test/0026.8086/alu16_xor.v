module alu16_xor(
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] y
);
    assign y = a ^ b;
endmodule
