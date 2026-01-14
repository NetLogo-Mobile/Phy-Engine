module mul8u_shiftadd (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    output wire [15:0] p
);
    integer i;
    reg [15:0] sum;

    always @* begin
        sum = 16'd0;
        for (i = 0; i < 8; i = i + 1) begin
            if (b[i])
                sum = sum + ({8'd0, a} << i);
        end
    end

    assign p = sum;
endmodule
