module alu16_and(
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] y
);
    assign y = a & b;
endmodule
