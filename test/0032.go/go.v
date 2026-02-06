module go19x19_english (
    input         clk,
    input         rst_n,
    input  [18:0] x,
    input  [18:0] y,
    input         place,

    output        black,
    output        white,
    output [18:0] row0,
    output [18:0] row1,
    output [18:0] row2,
    output [18:0] row3,
    output [18:0] row4,
    output [18:0] row5,
    output [18:0] row6,
    output [18:0] row7,
    output [18:0] row8,
    output [18:0] row9,
    output [18:0] row10,
    output [18:0] row11,
    output [18:0] row12,
    output [18:0] row13,
    output [18:0] row14,
    output [18:0] row15,
    output [18:0] row16,
    output [18:0] row17,
    output [18:0] row18
);

    // Occupancy-only board. LSB is x=0.
    reg [18:0] occ0, occ1, occ2, occ3, occ4, occ5, occ6, occ7, occ8, occ9;
    reg [18:0] occ10, occ11, occ12, occ13, occ14, occ15, occ16, occ17, occ18;

    // UI helpers: blink the selected empty intersection. "black/white" is time-multiplexed by phase.
    reg phase;
    reg turn_white;
    reg game_over;
    reg winner_white;
    reg move_captured;

    // place edge detect.
    reg place_d;
    wire place_pulse;
    assign place_pulse = place & (~place_d);

    // Latched move (onehot coords).
    reg [18:0] placed_x;
    reg [18:0] placed_y;
    reg        captured_any;

    // Scan/liberty check context (single-stone groups only).
    reg [2:0]  dir;
    reg [1:0]  subdir;
    reg        has_liberty;
    reg        self_check;
    reg [18:0] stone_x;
    reg [18:0] stone_y;

    // FSM
    parameter S_IDLE     = 3'd0;
    parameter S_NEIGHBOR = 3'd1;
    parameter S_LIB_STEP = 3'd2;
    parameter S_DECIDE   = 3'd3;
    reg [2:0] state;

    // Onehot neighbors via shifts.
    reg [18:0] nx;
    reg [18:0] ny;
    reg        neigh_valid;
    always @(*) begin
        nx = placed_x;
        ny = placed_y;
        neigh_valid = 1'b0;
        if (dir == 3'd0) begin
            nx = (placed_x >> 1);
            ny = placed_y;
            neigh_valid = (|nx);
        end else if (dir == 3'd1) begin
            nx = (placed_x << 1);
            ny = placed_y;
            neigh_valid = (|nx);
        end else if (dir == 3'd2) begin
            nx = placed_x;
            ny = (placed_y >> 1);
            neigh_valid = (|ny);
        end else if (dir == 3'd3) begin
            nx = placed_x;
            ny = (placed_y << 1);
            neigh_valid = (|ny);
        end
    end

    reg [18:0] ax;
    reg [18:0] ay;
    reg        avalid;
    always @(*) begin
        ax = stone_x;
        ay = stone_y;
        avalid = 1'b0;
        if (subdir == 2'd0) begin
            ax = (stone_x >> 1);
            ay = stone_y;
            avalid = (|ax);
        end else if (subdir == 2'd1) begin
            ax = (stone_x << 1);
            ay = stone_y;
            avalid = (|ax);
        end else if (subdir == 2'd2) begin
            ax = stone_x;
            ay = (stone_y >> 1);
            avalid = (|ay);
        end else begin
            ax = stone_x;
            ay = (stone_y << 1);
            avalid = (|ay);
        end
    end

    // Shared query (qx,qy) onehot.
    reg [18:0] qx;
    reg [18:0] qy;
    always @(*) begin
        if (state == S_IDLE) begin
            qx = x;
            qy = y;
        end else if (state == S_NEIGHBOR) begin
            qx = nx;
            qy = ny;
        end else if (state == S_LIB_STEP) begin
            qx = ax;
            qy = ay;
        end else begin
            qx = 19'd0;
            qy = 19'd0;
        end
    end

    wire q_valid;
    assign q_valid = (|qx) & (|qy);

    wire [18:0] sel_occ =
        (occ0  & {19{qy[0]}})  | (occ1  & {19{qy[1]}})  | (occ2  & {19{qy[2]}})  | (occ3  & {19{qy[3]}})  |
        (occ4  & {19{qy[4]}})  | (occ5  & {19{qy[5]}})  | (occ6  & {19{qy[6]}})  | (occ7  & {19{qy[7]}})  |
        (occ8  & {19{qy[8]}})  | (occ9  & {19{qy[9]}})  | (occ10 & {19{qy[10]}}) | (occ11 & {19{qy[11]}}) |
        (occ12 & {19{qy[12]}}) | (occ13 & {19{qy[13]}}) | (occ14 & {19{qy[14]}}) | (occ15 & {19{qy[15]}}) |
        (occ16 & {19{qy[16]}}) | (occ17 & {19{qy[17]}}) | (occ18 & {19{qy[18]}});

    wire q_occ;
    assign q_occ = q_valid & (|(sel_occ & qx));

    // Controls and masks.
    wire do_place;
    assign do_place = (state == S_IDLE) & (~game_over) & place_pulse & q_valid & (~q_occ);

    wire do_capture;
    assign do_capture = (state == S_DECIDE) & (~self_check) & (~has_liberty);
    wire do_undo;
    assign do_undo = (state == S_DECIDE) & (self_check) & (~has_liberty) & (~captured_any);

    wire [18:0] place0  = x & {19{y[0]}}  & {19{do_place}};
    wire [18:0] place1  = x & {19{y[1]}}  & {19{do_place}};
    wire [18:0] place2  = x & {19{y[2]}}  & {19{do_place}};
    wire [18:0] place3  = x & {19{y[3]}}  & {19{do_place}};
    wire [18:0] place4  = x & {19{y[4]}}  & {19{do_place}};
    wire [18:0] place5  = x & {19{y[5]}}  & {19{do_place}};
    wire [18:0] place6  = x & {19{y[6]}}  & {19{do_place}};
    wire [18:0] place7  = x & {19{y[7]}}  & {19{do_place}};
    wire [18:0] place8  = x & {19{y[8]}}  & {19{do_place}};
    wire [18:0] place9  = x & {19{y[9]}}  & {19{do_place}};
    wire [18:0] place10 = x & {19{y[10]}} & {19{do_place}};
    wire [18:0] place11 = x & {19{y[11]}} & {19{do_place}};
    wire [18:0] place12 = x & {19{y[12]}} & {19{do_place}};
    wire [18:0] place13 = x & {19{y[13]}} & {19{do_place}};
    wire [18:0] place14 = x & {19{y[14]}} & {19{do_place}};
    wire [18:0] place15 = x & {19{y[15]}} & {19{do_place}};
    wire [18:0] place16 = x & {19{y[16]}} & {19{do_place}};
    wire [18:0] place17 = x & {19{y[17]}} & {19{do_place}};
    wire [18:0] place18 = x & {19{y[18]}} & {19{do_place}};

    wire [18:0] cap0  = stone_x & {19{stone_y[0]}}  & {19{do_capture}};
    wire [18:0] cap1  = stone_x & {19{stone_y[1]}}  & {19{do_capture}};
    wire [18:0] cap2  = stone_x & {19{stone_y[2]}}  & {19{do_capture}};
    wire [18:0] cap3  = stone_x & {19{stone_y[3]}}  & {19{do_capture}};
    wire [18:0] cap4  = stone_x & {19{stone_y[4]}}  & {19{do_capture}};
    wire [18:0] cap5  = stone_x & {19{stone_y[5]}}  & {19{do_capture}};
    wire [18:0] cap6  = stone_x & {19{stone_y[6]}}  & {19{do_capture}};
    wire [18:0] cap7  = stone_x & {19{stone_y[7]}}  & {19{do_capture}};
    wire [18:0] cap8  = stone_x & {19{stone_y[8]}}  & {19{do_capture}};
    wire [18:0] cap9  = stone_x & {19{stone_y[9]}}  & {19{do_capture}};
    wire [18:0] cap10 = stone_x & {19{stone_y[10]}} & {19{do_capture}};
    wire [18:0] cap11 = stone_x & {19{stone_y[11]}} & {19{do_capture}};
    wire [18:0] cap12 = stone_x & {19{stone_y[12]}} & {19{do_capture}};
    wire [18:0] cap13 = stone_x & {19{stone_y[13]}} & {19{do_capture}};
    wire [18:0] cap14 = stone_x & {19{stone_y[14]}} & {19{do_capture}};
    wire [18:0] cap15 = stone_x & {19{stone_y[15]}} & {19{do_capture}};
    wire [18:0] cap16 = stone_x & {19{stone_y[16]}} & {19{do_capture}};
    wire [18:0] cap17 = stone_x & {19{stone_y[17]}} & {19{do_capture}};
    wire [18:0] cap18 = stone_x & {19{stone_y[18]}} & {19{do_capture}};

    wire [18:0] undo0  = placed_x & {19{placed_y[0]}}  & {19{do_undo}};
    wire [18:0] undo1  = placed_x & {19{placed_y[1]}}  & {19{do_undo}};
    wire [18:0] undo2  = placed_x & {19{placed_y[2]}}  & {19{do_undo}};
    wire [18:0] undo3  = placed_x & {19{placed_y[3]}}  & {19{do_undo}};
    wire [18:0] undo4  = placed_x & {19{placed_y[4]}}  & {19{do_undo}};
    wire [18:0] undo5  = placed_x & {19{placed_y[5]}}  & {19{do_undo}};
    wire [18:0] undo6  = placed_x & {19{placed_y[6]}}  & {19{do_undo}};
    wire [18:0] undo7  = placed_x & {19{placed_y[7]}}  & {19{do_undo}};
    wire [18:0] undo8  = placed_x & {19{placed_y[8]}}  & {19{do_undo}};
    wire [18:0] undo9  = placed_x & {19{placed_y[9]}}  & {19{do_undo}};
    wire [18:0] undo10 = placed_x & {19{placed_y[10]}} & {19{do_undo}};
    wire [18:0] undo11 = placed_x & {19{placed_y[11]}} & {19{do_undo}};
    wire [18:0] undo12 = placed_x & {19{placed_y[12]}} & {19{do_undo}};
    wire [18:0] undo13 = placed_x & {19{placed_y[13]}} & {19{do_undo}};
    wire [18:0] undo14 = placed_x & {19{placed_y[14]}} & {19{do_undo}};
    wire [18:0] undo15 = placed_x & {19{placed_y[15]}} & {19{do_undo}};
    wire [18:0] undo16 = placed_x & {19{placed_y[16]}} & {19{do_undo}};
    wire [18:0] undo17 = placed_x & {19{placed_y[17]}} & {19{do_undo}};
    wire [18:0] undo18 = placed_x & {19{placed_y[18]}} & {19{do_undo}};

    wire cursor_show;
    assign cursor_show = (state == S_IDLE) & (~game_over) & q_valid & (~q_occ) & (phase == turn_white);

    wire blink;
    assign blink = phase;

    // Turn indicator: blink the side-to-move. When game_over, show the winner as solid.
    assign black = game_over ? (~winner_white) : ((~turn_white) & blink);
    assign white = game_over ? (winner_white) : (turn_white & blink);

    assign row0  = occ0  | (x & {19{y[0]}}  & {19{cursor_show}});
    assign row1  = occ1  | (x & {19{y[1]}}  & {19{cursor_show}});
    assign row2  = occ2  | (x & {19{y[2]}}  & {19{cursor_show}});
    assign row3  = occ3  | (x & {19{y[3]}}  & {19{cursor_show}});
    assign row4  = occ4  | (x & {19{y[4]}}  & {19{cursor_show}});
    assign row5  = occ5  | (x & {19{y[5]}}  & {19{cursor_show}});
    assign row6  = occ6  | (x & {19{y[6]}}  & {19{cursor_show}});
    assign row7  = occ7  | (x & {19{y[7]}}  & {19{cursor_show}});
    assign row8  = occ8  | (x & {19{y[8]}}  & {19{cursor_show}});
    assign row9  = occ9  | (x & {19{y[9]}}  & {19{cursor_show}});
    assign row10 = occ10 | (x & {19{y[10]}} & {19{cursor_show}});
    assign row11 = occ11 | (x & {19{y[11]}} & {19{cursor_show}});
    assign row12 = occ12 | (x & {19{y[12]}} & {19{cursor_show}});
    assign row13 = occ13 | (x & {19{y[13]}} & {19{cursor_show}});
    assign row14 = occ14 | (x & {19{y[14]}} & {19{cursor_show}});
    assign row15 = occ15 | (x & {19{y[15]}} & {19{cursor_show}});
    assign row16 = occ16 | (x & {19{y[16]}} & {19{cursor_show}});
    assign row17 = occ17 | (x & {19{y[17]}} & {19{cursor_show}});
    assign row18 = occ18 | (x & {19{y[18]}} & {19{cursor_show}});

    always @(posedge clk) begin
        if (!rst_n) begin
            occ0 <= 19'd0;  occ1 <= 19'd0;  occ2 <= 19'd0;  occ3 <= 19'd0;  occ4 <= 19'd0;
            occ5 <= 19'd0;  occ6 <= 19'd0;  occ7 <= 19'd0;  occ8 <= 19'd0;  occ9 <= 19'd0;
            occ10 <= 19'd0; occ11 <= 19'd0; occ12 <= 19'd0; occ13 <= 19'd0; occ14 <= 19'd0;
            occ15 <= 19'd0; occ16 <= 19'd0; occ17 <= 19'd0; occ18 <= 19'd0;

            phase <= 1'b0;
            turn_white <= 1'b0;
            game_over <= 1'b0;
            winner_white <= 1'b0;
            move_captured <= 1'b0;
            place_d <= 1'b0;

            placed_x <= 19'd0;
            placed_y <= 19'd0;
            captured_any <= 1'b0;
            dir <= 3'd0;
            subdir <= 2'd0;
            has_liberty <= 1'b0;
            self_check <= 1'b0;
            stone_x <= 19'd0;
            stone_y <= 19'd0;
            state <= S_IDLE;
        end else begin
            phase <= ~phase;
            place_d <= place;

            // Writeback: (row | set_mask) & ~clr_mask
            occ0  <= (occ0  | place0)  & ~(cap0  | undo0);
            occ1  <= (occ1  | place1)  & ~(cap1  | undo1);
            occ2  <= (occ2  | place2)  & ~(cap2  | undo2);
            occ3  <= (occ3  | place3)  & ~(cap3  | undo3);
            occ4  <= (occ4  | place4)  & ~(cap4  | undo4);
            occ5  <= (occ5  | place5)  & ~(cap5  | undo5);
            occ6  <= (occ6  | place6)  & ~(cap6  | undo6);
            occ7  <= (occ7  | place7)  & ~(cap7  | undo7);
            occ8  <= (occ8  | place8)  & ~(cap8  | undo8);
            occ9  <= (occ9  | place9)  & ~(cap9  | undo9);
            occ10 <= (occ10 | place10) & ~(cap10 | undo10);
            occ11 <= (occ11 | place11) & ~(cap11 | undo11);
            occ12 <= (occ12 | place12) & ~(cap12 | undo12);
            occ13 <= (occ13 | place13) & ~(cap13 | undo13);
            occ14 <= (occ14 | place14) & ~(cap14 | undo14);
            occ15 <= (occ15 | place15) & ~(cap15 | undo15);
            occ16 <= (occ16 | place16) & ~(cap16 | undo16);
            occ17 <= (occ17 | place17) & ~(cap17 | undo17);
            occ18 <= (occ18 | place18) & ~(cap18 | undo18);

            // Control / state updates.
            if (state == S_IDLE) begin
                if (do_place) begin
                    placed_x <= x;
                    placed_y <= y;
                    captured_any <= 1'b0;
                    move_captured <= 1'b0;
                    dir <= 3'd0;
                    self_check <= 1'b0;
                    state <= S_NEIGHBOR;
                end
            end else if (state == S_NEIGHBOR) begin
                if (dir == 3'd4) begin
                    self_check <= 1'b1;
                    stone_x <= placed_x;
                    stone_y <= placed_y;
                    subdir <= 2'd0;
                    has_liberty <= 1'b0;
                    state <= S_LIB_STEP;
                end else begin
                    if (neigh_valid && q_occ) begin
                        self_check <= 1'b0;
                        stone_x <= nx;
                        stone_y <= ny;
                        subdir <= 2'd0;
                        has_liberty <= 1'b0;
                        state <= S_LIB_STEP;
                    end else begin
                        dir <= dir + 3'd1;
                    end
                end
            end else if (state == S_LIB_STEP) begin
                if (avalid && q_valid && (~q_occ)) begin
                    has_liberty <= 1'b1;
                end
                if (subdir == 2'd3) begin
                    state <= S_DECIDE;
                end
                subdir <= subdir + 2'd1;
            end else if (state == S_DECIDE) begin
                if (do_capture) begin
                    captured_any <= 1'b1;
                    move_captured <= 1'b1;
                end
                if (self_check) begin
                    if (~do_undo) begin
                        if (move_captured) begin
                            game_over <= 1'b1;
                            winner_white <= turn_white;
                        end else begin
                            turn_white <= ~turn_white;
                        end
                    end
                    state <= S_IDLE;
                end else begin
                    dir <= dir + 3'd1;
                    state <= S_NEIGHBOR;
                end
            end
        end
    end

endmodule
