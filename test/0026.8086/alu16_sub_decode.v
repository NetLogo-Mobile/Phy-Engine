module alu16_sub_decode(
    input  wire [2:0] op,
    output wire       sub
);
    assign sub = (op == 3'd1);
endmodule

