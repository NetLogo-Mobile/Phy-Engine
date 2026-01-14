module ir16(
    input  wire        clk,
    input  wire        rst_n,
    input  wire [15:0] d,
    output reg  [15:0] q
);
    // Latch on negedge so the fetched instruction is stable during the next posedge (execute phase).
    always @(negedge clk or negedge rst_n) begin
        if(!rst_n) begin
            q <= 16'd0;
        end else begin
            q <= d;
        end
    end
endmodule
