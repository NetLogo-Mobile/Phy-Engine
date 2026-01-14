module flag1(
    input  wire clk,
    input  wire rst_n,
    input  wire we,
    input  wire d,
    output reg  q
);
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            q <= 1'b0;
        end else if(we) begin
            q <= d;
        end
    end
endmodule

