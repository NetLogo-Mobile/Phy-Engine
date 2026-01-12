module tetris_top(input clk,
                  input rst_n,
                  input btn_left,
                  input btn_right,
                  input btn_rot,
                  input btn_drop,
                  output [63:0] pix);
    reg [2:0] x;
    reg [2:0] y;
    reg       shape; // 0: horizontal 2-block, 1: vertical 2-block

    wire [5:0] idx0;
    wire [5:0] idx1_h;
    wire [5:0] idx1_v;
    assign idx0 = {y, x};
    assign idx1_h = {y, (x == 3'd7) ? 3'd0 : (x + 3'd1)};
    assign idx1_v = {((y == 3'd7) ? 3'd0 : (y + 3'd1)), x};

    wire [63:0] p0;
    wire [63:0] p1;
    assign p0 = (64'h1 << idx0);
    assign p1 = (64'h1 << (shape ? idx1_v : idx1_h));
    assign pix = p0 | p1;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            x <= 3'd3;
            y <= 3'd0;
            shape <= 1'b0;
        end else begin
            if(btn_left && x != 3'd0) begin
                x <= x - 3'd1;
            end else if(btn_right && x != 3'd7) begin
                x <= x + 3'd1;
            end

            if(btn_rot) begin
                shape <= ~shape;
            end

            if(btn_drop) begin
                y <= y + 3'd2;
            end else begin
                y <= y + 3'd1;
            end
        end
    end
endmodule
