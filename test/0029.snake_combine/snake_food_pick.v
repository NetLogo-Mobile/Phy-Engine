module snake_food_pick(
    input  wire [3:0] rnd_a,
    input  wire [3:0] rnd_b,
    input  wire [5:0] idx_head_next,
    input  wire [5:0] idx_head_now,
    input  wire [5:0] idx0_now,
    input  wire [5:0] idx1_now,
    input  wire [5:0] idx2_now,
    output wire [5:0] new_food_idx
);
    // Random candidate (6-bit) built from two 4-bit generators.
    wire [5:0] cand_base;
    assign cand_base = {rnd_a[2:0], rnd_b[2:0]};

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

    wire coll_base;
    wire coll1;
    wire coll2;
    wire coll3;
    wire coll4;
    wire coll5;
    wire coll6;
    wire coll7;
    assign coll_base = coll_idx(cand_base);
    assign coll1 = coll_idx(cand1);
    assign coll2 = coll_idx(cand2);
    assign coll3 = coll_idx(cand3);
    assign coll4 = coll_idx(cand4);
    assign coll5 = coll_idx(cand5);
    assign coll6 = coll_idx(cand6);
    assign coll7 = coll_idx(cand7);

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
endmodule

