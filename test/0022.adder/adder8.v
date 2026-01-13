module full_adder1(
    input  a,
    input  b,
    input  cin,
    output s,
    output cout
);
    assign s = a ^ b ^ cin;
    assign cout = (a & b) | (a & cin) | (b & cin);
endmodule

module adder8_top(
    input  [7:0] a,
    input  [7:0] b,
    input        c,     // carry in
    output [7:0] s,
    output       cout
);
    wire [8:0] carry;
    assign carry[0] = c;

    genvar i;
    generate
        for(i = 0; i < 8; i = i + 1)
        begin : g
            full_adder1 u(
                .a(a[i]),
                .b(b[i]),
                .cin(carry[i]),
                .s(s[i]),
                .cout(carry[i + 1])
            );
        end
    endgenerate

    assign cout = carry[8];
endmodule
