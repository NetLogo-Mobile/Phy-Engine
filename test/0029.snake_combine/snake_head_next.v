module snake_head_next(
    input  wire [5:0] idx_head,
    input  wire [1:0] next_dir,
    output wire [5:0] idx_head_next
);
    wire [2:0] head_x;
    wire [2:0] head_y;
    assign head_x = idx_head[2:0];
    assign head_y = idx_head[5:3];

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
endmodule

