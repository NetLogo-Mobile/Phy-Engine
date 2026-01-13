`include "../0020.fp16_calc/fp16_addsub.v"
`include "../0020.fp16_calc/fp16_mul.v"
`include "../0020.fp16_calc/fp16_div.v"

module fp16_fpu_top(
    input  [15:0] a,
    input  [15:0] b,
    input  [1:0]  op,  // 0:add 1:sub 2:mul 3:div
    output [15:0] y
);
    wire [15:0] y_add;
    wire [15:0] y_sub;
    wire [15:0] y_mul;
    wire [15:0] y_div;

    fp16_addsub_unit u_add(.a(a), .b(b), .sub(1'b0), .y(y_add));
    fp16_addsub_unit u_sub(.a(a), .b(b), .sub(1'b1), .y(y_sub));
    fp16_mul_unit    u_mul(.a(a), .b(b), .y(y_mul));
    fp16_div_unit    u_div(.a(a), .b(b), .y(y_div));

    reg [15:0] y_r;
    assign y = y_r;

    always @*
    begin
        case(op)
            2'b00: y_r = y_add;
            2'b01: y_r = y_sub;
            2'b10: y_r = y_mul;
            default: y_r = y_div;
        endcase
    end
endmodule

