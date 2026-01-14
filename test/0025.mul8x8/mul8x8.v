module mul8 (
    input  wire [7:0] a,
    input  wire [7:0] b,
    output wire [15:0] p
);
    assign p = a * b;
endmodule

