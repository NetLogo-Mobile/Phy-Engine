module snake_hit_eat(
    input  wire [5:0] idx_head_next,
    input  wire [5:0] idx0,
    input  wire [5:0] idx1,
    input  wire [5:0] idx2,
    input  wire [5:0] idx_food,
    output wire       eat,
    output wire       hit_body
);
    assign eat = (idx_head_next == idx_food);
    assign hit_body =
        (idx_head_next == idx0) ||
        (idx_head_next == idx1) ||
        (idx_head_next == idx2);
endmodule

