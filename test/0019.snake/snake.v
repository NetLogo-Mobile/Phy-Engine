module snake_top(input clk,
                 input rst_n,
                 input btn_up,
                 input btn_down,
                 input btn_left,
                 input btn_right,
                 output [63:0] pix);
    // 8x8 snake (fixed length = 4). This is intentionally simple: it exercises
    // Verilog->PE synthesis and PE->PL export for a "game-like" design.

    // Direction encoding:
    // 00: right, 01: down, 10: left, 11: up
    reg [1:0] dir;

    reg [2:0] head_x;
    reg [2:0] head_y;

    reg [2:0] s0_x, s0_y;
    reg [2:0] s1_x, s1_y;
    reg [2:0] s2_x, s2_y;

    wire [1:0] next_dir;
    assign next_dir = btn_up    ? 2'b11 :
                      btn_down  ? 2'b01 :
                      btn_left  ? 2'b10 :
                      btn_right ? 2'b00 :
                                  dir;

    wire [2:0] next_head_x;
    wire [2:0] next_head_y;

    assign next_head_x =
        (next_dir == 2'b00) ? ((head_x == 3'd7) ? 3'd0 : (head_x + 3'd1)) :
        (next_dir == 2'b10) ? ((head_x == 3'd0) ? 3'd7 : (head_x - 3'd1)) :
                              head_x;

    assign next_head_y =
        (next_dir == 2'b01) ? ((head_y == 3'd7) ? 3'd0 : (head_y + 3'd1)) :
        (next_dir == 2'b11) ? ((head_y == 3'd0) ? 3'd7 : (head_y - 3'd1)) :
                              head_y;

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dir <= 2'b00;
            head_x <= 3'd3;
            head_y <= 3'd3;
            s0_x <= 3'd2; s0_y <= 3'd3;
            s1_x <= 3'd1; s1_y <= 3'd3;
            s2_x <= 3'd0; s2_y <= 3'd3;
        end else begin
            dir <= next_dir;

            s2_x <= s1_x; s2_y <= s1_y;
            s1_x <= s0_x; s1_y <= s0_y;
            s0_x <= head_x; s0_y <= head_y;

            head_x <= next_head_x;
            head_y <= next_head_y;
        end
    end

    wire [5:0] idx_head;
    wire [5:0] idx0;
    wire [5:0] idx1;
    wire [5:0] idx2;
    assign idx_head = {head_y, head_x};
    assign idx0 = {s0_y, s0_x};
    assign idx1 = {s1_y, s1_x};
    assign idx2 = {s2_y, s2_x};

    wire [63:0] p_head;
    wire [63:0] p0;
    wire [63:0] p1;
    wire [63:0] p2;
    assign p_head = (64'h1 << idx_head);
    assign p0 = (64'h1 << idx0);
    assign p1 = (64'h1 << idx1);
    assign p2 = (64'h1 << idx2);

    assign pix = p_head | p0 | p1 | p2;
endmodule

