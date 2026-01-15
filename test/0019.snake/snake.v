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

    // Food (static until eaten). When the head moves onto the food cell, a new
    // food position is generated from a small LFSR.
    reg [2:0] food_x;
    reg [2:0] food_y;
    reg [5:0] lfsr;
    wire [5:0] lfsr_next;
    assign lfsr_next = {lfsr[4:0], (lfsr[5] ^ lfsr[4])};

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

    wire eat_next;
    assign eat_next = (next_head_x == food_x) && (next_head_y == food_y);

    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dir <= 2'b00;
            head_x <= 3'd3;
            head_y <= 3'd3;
            s0_x <= 3'd2; s0_y <= 3'd3;
            s1_x <= 3'd1; s1_y <= 3'd3;
            s2_x <= 3'd0; s2_y <= 3'd3;

            food_x <= 3'd5;
            food_y <= 3'd5;
            lfsr <= 6'h3A;
        end else begin
            dir <= next_dir;

            s2_x <= s1_x; s2_y <= s1_y;
            s1_x <= s0_x; s1_y <= s0_y;
            s0_x <= head_x; s0_y <= head_y;

            head_x <= next_head_x;
            head_y <= next_head_y;

            if(eat_next) begin
                lfsr <= lfsr_next;
                food_x <= lfsr_next[2:0];
                food_y <= lfsr_next[5:3];
            end
        end
    end

    wire [5:0] idx_head;
    wire [5:0] idx0;
    wire [5:0] idx1;
    wire [5:0] idx2;
    wire [5:0] idx_food;
    assign idx_head = {head_y, head_x};
    assign idx0 = {s0_y, s0_x};
    assign idx1 = {s1_y, s1_x};
    assign idx2 = {s2_y, s2_x};
    assign idx_food = {food_y, food_x};

    wire [63:0] p_head;
    wire [63:0] p0;
    wire [63:0] p1;
    wire [63:0] p2;
    wire [63:0] p_food;
    assign p_head = (idx_head < 6'd32) ? (64'h1 << idx_head) : ((64'h1 << 6'd32) << (idx_head - 6'd32));
    assign p0 = (idx0 < 6'd32) ? (64'h1 << idx0) : ((64'h1 << 6'd32) << (idx0 - 6'd32));
    assign p1 = (idx1 < 6'd32) ? (64'h1 << idx1) : ((64'h1 << 6'd32) << (idx1 - 6'd32));
    assign p2 = (idx2 < 6'd32) ? (64'h1 << idx2) : ((64'h1 << 6'd32) << (idx2 - 6'd32));
    assign p_food = (idx_food < 6'd32) ? (64'h1 << idx_food) : ((64'h1 << 6'd32) << (idx_food - 6'd32));

    assign pix = p_head | p0 | p1 | p2 | p_food;
endmodule
