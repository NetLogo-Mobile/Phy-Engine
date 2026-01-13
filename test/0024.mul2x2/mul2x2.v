module half_adder1(
    input  a,
    input  b,
    output s,
    output c
);
    assign s = a ^ b;
    assign c = a & b;
endmodule

module mul2x2_top(
    input  [1:0] a,
    input  [1:0] b,
    output [3:0] y
);
    wire pp0;
    wire pp1;
    wire pp2;
    wire pp3;

    assign pp0 = a[0] & b[0];
    assign pp1 = a[1] & b[0];
    assign pp2 = a[0] & b[1];
    assign pp3 = a[1] & b[1];

    wire s1;
    wire c1;
    wire s2;
    wire c2;

    // y = pp0 + ((pp1 + pp2) << 1) + (pp3 << 2)
    half_adder1 ha1(.a(pp1), .b(pp2), .s(s1), .c(c1));
    half_adder1 ha2(.a(pp3), .b(c1), .s(s2), .c(c2));

    assign y[0] = pp0;
    assign y[1] = s1;
    assign y[2] = s2;
    assign y[3] = c2;
endmodule

