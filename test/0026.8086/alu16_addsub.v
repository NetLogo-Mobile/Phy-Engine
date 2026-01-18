module alu16_addsub(
    // sub=0: y = a + b,     cf = carry_out
    // sub=1: y = a - b,     cf = borrow      (borrow = ~carry_out of a + (~b) + 1)
    input  wire        sub,
    input  wire [15:0] a,
    input  wire [15:0] b,
    output wire [15:0] y,
    output wire        cf
);
    // Conditional invert (XOR) and cin implement two's-complement subtraction:
    //   a - b == a + (b ^ {16{sub}}) + sub
    wire [15:0] bx;
    assign bx = b ^ {16{sub}};

    wire [16:0] tmp;
    assign tmp = {1'b0, a} + {1'b0, bx} + {16'd0, sub};

    assign y = tmp[15:0];

    wire carry_out;
    assign carry_out = tmp[16];

    assign cf = sub ? ~carry_out : carry_out;
endmodule

