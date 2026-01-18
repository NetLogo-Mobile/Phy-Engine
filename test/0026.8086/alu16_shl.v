module alu16_shl(
    input  wire [15:0] a,
    input  wire [15:0] b,
    output reg  [15:0] y,
    output reg         cf
);
    reg [3:0] sh;

    always @(*) begin
        sh = b[3:0];
        y = a << sh;
        cf = 1'b0;
        if(sh != 0) cf = a[16 - sh];
    end
endmodule

