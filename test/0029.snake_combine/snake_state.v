module snake_state(
    input  wire       clk,
    input  wire       rst_n,
    input  wire [1:0] next_dir,
    input  wire [5:0] idx_head_next,
    input  wire       eat,
    input  wire       hit_body,
    input  wire [5:0] new_food_idx,
    output reg  [1:0] dir,
    output reg  [5:0] idx_head,
    output reg  [5:0] idx0,
    output reg  [5:0] idx1,
    output reg  [5:0] idx2,
    output reg  [5:0] idx_food,
    output reg        game_over
);
    always_ff @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            dir <= 2'b00;
            idx_head <= 6'b011_011;  // (3,3)
            idx0 <= 6'b011_010;      // (2,3)
            idx1 <= 6'b011_001;      // (1,3)
            idx2 <= 6'b011_000;      // (0,3)
            idx_food <= 6'b101_101;  // (5,5)
            game_over <= 1'b0;
        end else begin
            if(!game_over) begin
                dir <= next_dir;

                idx2 <= idx1;
                idx1 <= idx0;
                idx0 <= idx_head;
                idx_head <= idx_head_next;

                if(hit_body) begin
                    game_over <= 1'b1;
                end

                if(eat) begin
                    idx_food <= new_food_idx;
                end
            end
        end
    end
endmodule

