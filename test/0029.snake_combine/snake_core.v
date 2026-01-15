module snake_core(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       btn_up,
    input  wire       btn_down,
    input  wire       btn_left,
    input  wire       btn_right,
    input  wire [3:0] rnd_pl,
    input  wire [3:0] rnd_pe,
    output wire [5:0] idx_head,
    output wire [5:0] idx0,
    output wire [5:0] idx1,
    output wire [5:0] idx2,
    output wire [5:0] idx_food,
    output reg        game_over
);
    // Direction encoding:
    // 00: right, 01: down, 10: left, 11: up
    reg [1:0] dir;

    reg [2:0] head_x, head_y;
    reg [2:0] s0_x, s0_y;
    reg [2:0] s1_x, s1_y;
    reg [2:0] s2_x, s2_y;

    reg [2:0] food_x, food_y;

    wire [5:0] idx_head_next;
    wire [1:0] raw_dir;
    assign raw_dir = btn_up    ? 2'b11 :
                     btn_down  ? 2'b01 :
                     btn_left  ? 2'b10 :
                     btn_right ? 2'b00 :
                                 dir;

    // Prevent immediate 180-degree reversal (for length=4).
    wire [1:0] next_dir;
    assign next_dir =
        ((dir == 2'b00) && (raw_dir == 2'b10)) ? dir :
        ((dir == 2'b10) && (raw_dir == 2'b00)) ? dir :
        ((dir == 2'b01) && (raw_dir == 2'b11)) ? dir :
        ((dir == 2'b11) && (raw_dir == 2'b01)) ? dir :
        raw_dir;

    wire [2:0] head_x_next;
    wire [2:0] head_y_next;
    assign head_x_next =
        (next_dir == 2'b00) ? ((head_x == 3'd7) ? 3'd0 : (head_x + 3'd1)) :
        (next_dir == 2'b10) ? ((head_x == 3'd0) ? 3'd7 : (head_x - 3'd1)) :
                              head_x;
    assign head_y_next =
        (next_dir == 2'b01) ? ((head_y == 3'd7) ? 3'd0 : (head_y + 3'd1)) :
        (next_dir == 2'b11) ? ((head_y == 3'd0) ? 3'd7 : (head_y - 3'd1)) :
                              head_y;

    assign idx_head_next = {head_y_next, head_x_next};

    // Random candidate (6-bit) built from two 4-bit generators.
    wire [5:0] cand_base;
    assign cand_base = {rnd_pe[2:0], rnd_pl[2:0]};

    wire [5:0] idx_head_now;
    wire [5:0] idx0_now;
    wire [5:0] idx1_now;
    wire [5:0] idx2_now;
    assign idx_head_now = {head_y, head_x};
    assign idx0_now = {s0_y, s0_x};
    assign idx1_now = {s1_y, s1_x};
    assign idx2_now = {s2_y, s2_x};

    wire coll_base;
    wire coll1;
    wire coll2;
    wire coll3;
    wire coll4;
    wire coll5;
    wire coll6;
    wire coll7;

    function automatic coll_idx(input [5:0] idx);
        begin
            coll_idx =
                (idx == idx_head_next) ||
                (idx == idx_head_now) ||
                (idx == idx0_now) ||
                (idx == idx1_now) ||
                (idx == idx2_now);
        end
    endfunction

    wire [5:0] cand1;
    wire [5:0] cand2;
    wire [5:0] cand3;
    wire [5:0] cand4;
    wire [5:0] cand5;
    wire [5:0] cand6;
    wire [5:0] cand7;
    assign cand1 = cand_base + 6'd1;
    assign cand2 = cand_base + 6'd2;
    assign cand3 = cand_base + 6'd3;
    assign cand4 = cand_base + 6'd4;
    assign cand5 = cand_base + 6'd5;
    assign cand6 = cand_base + 6'd6;
    assign cand7 = cand_base + 6'd7;

    assign coll_base = coll_idx(cand_base);
    assign coll1 = coll_idx(cand1);
    assign coll2 = coll_idx(cand2);
    assign coll3 = coll_idx(cand3);
    assign coll4 = coll_idx(cand4);
    assign coll5 = coll_idx(cand5);
    assign coll6 = coll_idx(cand6);
    assign coll7 = coll_idx(cand7);

    wire [5:0] new_food_idx;
    assign new_food_idx =
        (!coll_base) ? cand_base :
        (!coll1) ? cand1 :
        (!coll2) ? cand2 :
        (!coll3) ? cand3 :
        (!coll4) ? cand4 :
        (!coll5) ? cand5 :
        (!coll6) ? cand6 :
        (!coll7) ? cand7 :
        (cand_base + 6'd8);

    wire eat;
    assign eat = (head_x_next == food_x) && (head_y_next == food_y);

    wire hit_body;
    assign hit_body =
        ((head_x_next == s0_x) && (head_y_next == s0_y)) ||
        ((head_x_next == s1_x) && (head_y_next == s1_y)) ||
        ((head_x_next == s2_x) && (head_y_next == s2_y));

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
            game_over <= 1'b0;
        end else begin
            if(!game_over) begin
                dir <= next_dir;

                s2_x <= s1_x; s2_y <= s1_y;
                s1_x <= s0_x; s1_y <= s0_y;
                s0_x <= head_x; s0_y <= head_y;

                head_x <= head_x_next;
                head_y <= head_y_next;

                if(hit_body) begin
                    game_over <= 1'b1;
                end

                if(eat) begin
                    food_x <= new_food_idx[2:0];
                    food_y <= new_food_idx[5:3];
                end
            end
        end
    end

    assign idx_head = {head_y, head_x};
    assign idx0 = {s0_y, s0_x};
    assign idx1 = {s1_y, s1_x};
    assign idx2 = {s2_y, s2_x};
    assign idx_food = {food_y, food_x};
endmodule
