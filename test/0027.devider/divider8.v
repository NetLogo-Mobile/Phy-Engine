module divider_8bit (
    input  [7:0] dividend,
    input  [7:0] divisor,
    output reg [7:0] quotient,
    output reg [7:0] remainder,
    output reg div_zero
);

    integer i;
    reg [15:0] temp;  // 扩展寄存器

    always @(*) begin
        quotient  = 8'd0;
        remainder = 8'd0;
        div_zero  = 1'b0;
        temp      = 16'd0;

        if (divisor == 0) begin
            div_zero  = 1'b1;
            quotient  = 8'd0;
            remainder = 8'd0;
        end else begin
            temp = {8'd0, dividend};

            for (i = 0; i < 8; i = i + 1) begin
                temp = temp << 1;
                if (temp[15:8] >= divisor) begin
                    temp[15:8] = temp[15:8] - divisor;
                    quotient = (quotient << 1) | 1'b1;
                end else begin
                    quotient = quotient << 1;
                end
            end

            remainder = temp[15:8];
        end
    end

endmodule
