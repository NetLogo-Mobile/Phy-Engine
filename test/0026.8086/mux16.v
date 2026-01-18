module mux16(
    input  wire        sel,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] y
);
    assign y = sel ? b : a;
endmodule

