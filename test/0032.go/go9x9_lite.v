module go9x9_lite_core (
    input         clk,
    input         rst_n,
    input  [8:0]  x,
    input  [8:0]  y,
    input         place,
    input         pass,

    output        black,
    output        white,
    output [8:0]  row0,
    output [8:0]  row1,
    output [8:0]  row2,
    output [8:0]  row3,
    output [8:0]  row4,
    output [8:0]  row5,
    output [8:0]  row6,
    output [8:0]  row7,
    output [8:0]  row8
);

    // Gate-budget version:
    // - separate black/white planes
    // - occupied-point prohibition
    // - single-stone capture only (groups > 1 are not captured correctly)
    // - suicide prohibited
    // - no ko (to fit <=5000 gates at O4)

    reg phase;
    reg turn_white;

    reg [8:0] b0, b1, b2, b3, b4, b5, b6, b7, b8;
    reg [8:0] w0, w1, w2, w3, w4, w5, w6, w7, w8;

    reg place_d;
    reg pass_d;
    wire place_pulse = place & (~place_d);
    wire pass_pulse  = pass  & (~pass_d);

    reg [8:0] placed_x;
    reg [8:0] placed_y;

    // Opponent neighbor scan.
    reg [2:0] dir;
    reg [1:0] subdir;
    reg       self_check;
    reg [8:0] stone_x;
    reg [8:0] stone_y;
    reg       has_lib;
    reg [1:0] self_lib_cnt_sat;

    // Captures during this move (saturating to 2); record first captured point.
    reg [1:0] cap_cnt_sat;
    reg [8:0] cap_x1;
    reg [8:0] cap_y1;

    parameter S_IDLE     = 3'd0;
    parameter S_NEIGHBOR = 3'd1;
    parameter S_LIB_STEP = 3'd2;
    parameter S_DECIDE   = 3'd3;
    reg [2:0] state;

    assign black = ~phase;
    assign white = phase;
    wire show_white = phase;

    wire [8:0] occ0 = b0 | w0;
    wire [8:0] occ1 = b1 | w1;
    wire [8:0] occ2 = b2 | w2;
    wire [8:0] occ3 = b3 | w3;
    wire [8:0] occ4 = b4 | w4;
    wire [8:0] occ5 = b5 | w5;
    wire [8:0] occ6 = b6 | w6;
    wire [8:0] occ7 = b7 | w7;
    wire [8:0] occ8 = b8 | w8;

    // Neighbor coordinate from placed_x/placed_y.
    reg [8:0] nx;
    reg [8:0] ny;
    reg       neigh_valid;
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

    // Adjacent coordinate from stone_x/stone_y.
    reg [8:0] ax;
    reg [8:0] ay;
    reg       avalid;
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

    // Shared query coordinate (qx,qy).
    reg [8:0] qx;
    reg [8:0] qy;
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
            qx = 9'd0;
            qy = 9'd0;
        end
    end

    wire [8:0] sel_occ =
        (occ0 & {9{qy[0]}}) | (occ1 & {9{qy[1]}}) | (occ2 & {9{qy[2]}}) | (occ3 & {9{qy[3]}}) |
        (occ4 & {9{qy[4]}}) | (occ5 & {9{qy[5]}}) | (occ6 & {9{qy[6]}}) | (occ7 & {9{qy[7]}}) |
        (occ8 & {9{qy[8]}});
    wire q_occ = (|(sel_occ & qx));

    wire [8:0] sel_b =
        (b0 & {9{qy[0]}}) | (b1 & {9{qy[1]}}) | (b2 & {9{qy[2]}}) | (b3 & {9{qy[3]}}) |
        (b4 & {9{qy[4]}}) | (b5 & {9{qy[5]}}) | (b6 & {9{qy[6]}}) | (b7 & {9{qy[7]}}) |
        (b8 & {9{qy[8]}});
    wire [8:0] sel_w =
        (w0 & {9{qy[0]}}) | (w1 & {9{qy[1]}}) | (w2 & {9{qy[2]}}) | (w3 & {9{qy[3]}}) |
        (w4 & {9{qy[4]}}) | (w5 & {9{qy[5]}}) | (w6 & {9{qy[6]}}) | (w7 & {9{qy[7]}}) |
        (w8 & {9{qy[8]}});
    wire q_black = (|(sel_b & qx));
    wire q_white = (|(sel_w & qx));

    wire do_place = (state == S_IDLE) & place_pulse & (~q_occ);
    wire do_pass  = (state == S_IDLE) & pass_pulse;

    wire do_cap  = (state == S_DECIDE) & (~self_check) & (~has_lib);
    wire do_undo = (state == S_DECIDE) & (self_check) & (~has_lib) & (cap_cnt_sat == 2'd0);

    wire [8:0] place0 = x & {9{y[0]}} & {9{do_place}};
    wire [8:0] place1 = x & {9{y[1]}} & {9{do_place}};
    wire [8:0] place2 = x & {9{y[2]}} & {9{do_place}};
    wire [8:0] place3 = x & {9{y[3]}} & {9{do_place}};
    wire [8:0] place4 = x & {9{y[4]}} & {9{do_place}};
    wire [8:0] place5 = x & {9{y[5]}} & {9{do_place}};
    wire [8:0] place6 = x & {9{y[6]}} & {9{do_place}};
    wire [8:0] place7 = x & {9{y[7]}} & {9{do_place}};
    wire [8:0] place8 = x & {9{y[8]}} & {9{do_place}};

    wire [8:0] undo0 = placed_x & {9{placed_y[0]}} & {9{do_undo}};
    wire [8:0] undo1 = placed_x & {9{placed_y[1]}} & {9{do_undo}};
    wire [8:0] undo2 = placed_x & {9{placed_y[2]}} & {9{do_undo}};
    wire [8:0] undo3 = placed_x & {9{placed_y[3]}} & {9{do_undo}};
    wire [8:0] undo4 = placed_x & {9{placed_y[4]}} & {9{do_undo}};
    wire [8:0] undo5 = placed_x & {9{placed_y[5]}} & {9{do_undo}};
    wire [8:0] undo6 = placed_x & {9{placed_y[6]}} & {9{do_undo}};
    wire [8:0] undo7 = placed_x & {9{placed_y[7]}} & {9{do_undo}};
    wire [8:0] undo8 = placed_x & {9{placed_y[8]}} & {9{do_undo}};

    wire [8:0] cap0 = stone_x & {9{stone_y[0]}} & {9{do_cap}};
    wire [8:0] cap1 = stone_x & {9{stone_y[1]}} & {9{do_cap}};
    wire [8:0] cap2 = stone_x & {9{stone_y[2]}} & {9{do_cap}};
    wire [8:0] cap3 = stone_x & {9{stone_y[3]}} & {9{do_cap}};
    wire [8:0] cap4 = stone_x & {9{stone_y[4]}} & {9{do_cap}};
    wire [8:0] cap5 = stone_x & {9{stone_y[5]}} & {9{do_cap}};
    wire [8:0] cap6 = stone_x & {9{stone_y[6]}} & {9{do_cap}};
    wire [8:0] cap7 = stone_x & {9{stone_y[7]}} & {9{do_cap}};
    wire [8:0] cap8 = stone_x & {9{stone_y[8]}} & {9{do_cap}};

    wire [8:0] set_b0 = place0 & {9{~turn_white}};
    wire [8:0] set_b1 = place1 & {9{~turn_white}};
    wire [8:0] set_b2 = place2 & {9{~turn_white}};
    wire [8:0] set_b3 = place3 & {9{~turn_white}};
    wire [8:0] set_b4 = place4 & {9{~turn_white}};
    wire [8:0] set_b5 = place5 & {9{~turn_white}};
    wire [8:0] set_b6 = place6 & {9{~turn_white}};
    wire [8:0] set_b7 = place7 & {9{~turn_white}};
    wire [8:0] set_b8 = place8 & {9{~turn_white}};

    wire [8:0] set_w0 = place0 & {9{turn_white}};
    wire [8:0] set_w1 = place1 & {9{turn_white}};
    wire [8:0] set_w2 = place2 & {9{turn_white}};
    wire [8:0] set_w3 = place3 & {9{turn_white}};
    wire [8:0] set_w4 = place4 & {9{turn_white}};
    wire [8:0] set_w5 = place5 & {9{turn_white}};
    wire [8:0] set_w6 = place6 & {9{turn_white}};
    wire [8:0] set_w7 = place7 & {9{turn_white}};
    wire [8:0] set_w8 = place8 & {9{turn_white}};

    wire [8:0] clr_b0 = cap0 & {9{turn_white}};
    wire [8:0] clr_b1 = cap1 & {9{turn_white}};
    wire [8:0] clr_b2 = cap2 & {9{turn_white}};
    wire [8:0] clr_b3 = cap3 & {9{turn_white}};
    wire [8:0] clr_b4 = cap4 & {9{turn_white}};
    wire [8:0] clr_b5 = cap5 & {9{turn_white}};
    wire [8:0] clr_b6 = cap6 & {9{turn_white}};
    wire [8:0] clr_b7 = cap7 & {9{turn_white}};
    wire [8:0] clr_b8 = cap8 & {9{turn_white}};

    wire [8:0] clr_w0 = cap0 & {9{~turn_white}};
    wire [8:0] clr_w1 = cap1 & {9{~turn_white}};
    wire [8:0] clr_w2 = cap2 & {9{~turn_white}};
    wire [8:0] clr_w3 = cap3 & {9{~turn_white}};
    wire [8:0] clr_w4 = cap4 & {9{~turn_white}};
    wire [8:0] clr_w5 = cap5 & {9{~turn_white}};
    wire [8:0] clr_w6 = cap6 & {9{~turn_white}};
    wire [8:0] clr_w7 = cap7 & {9{~turn_white}};
    wire [8:0] clr_w8 = cap8 & {9{~turn_white}};

    wire [8:0] undo_b0 = undo0 & {9{~turn_white}};
    wire [8:0] undo_b1 = undo1 & {9{~turn_white}};
    wire [8:0] undo_b2 = undo2 & {9{~turn_white}};
    wire [8:0] undo_b3 = undo3 & {9{~turn_white}};
    wire [8:0] undo_b4 = undo4 & {9{~turn_white}};
    wire [8:0] undo_b5 = undo5 & {9{~turn_white}};
    wire [8:0] undo_b6 = undo6 & {9{~turn_white}};
    wire [8:0] undo_b7 = undo7 & {9{~turn_white}};
    wire [8:0] undo_b8 = undo8 & {9{~turn_white}};

    wire [8:0] undo_w0 = undo0 & {9{turn_white}};
    wire [8:0] undo_w1 = undo1 & {9{turn_white}};
    wire [8:0] undo_w2 = undo2 & {9{turn_white}};
    wire [8:0] undo_w3 = undo3 & {9{turn_white}};
    wire [8:0] undo_w4 = undo4 & {9{turn_white}};
    wire [8:0] undo_w5 = undo5 & {9{turn_white}};
    wire [8:0] undo_w6 = undo6 & {9{turn_white}};
    wire [8:0] undo_w7 = undo7 & {9{turn_white}};
    wire [8:0] undo_w8 = undo8 & {9{turn_white}};

    wire cursor_show = (state == S_IDLE) & (~q_occ) & (phase == turn_white);
    wire [8:0] cursor0 = x & {9{y[0]}} & {9{cursor_show}};
    wire [8:0] cursor1 = x & {9{y[1]}} & {9{cursor_show}};
    wire [8:0] cursor2 = x & {9{y[2]}} & {9{cursor_show}};
    wire [8:0] cursor3 = x & {9{y[3]}} & {9{cursor_show}};
    wire [8:0] cursor4 = x & {9{y[4]}} & {9{cursor_show}};
    wire [8:0] cursor5 = x & {9{y[5]}} & {9{cursor_show}};
    wire [8:0] cursor6 = x & {9{y[6]}} & {9{cursor_show}};
    wire [8:0] cursor7 = x & {9{y[7]}} & {9{cursor_show}};
    wire [8:0] cursor8 = x & {9{y[8]}} & {9{cursor_show}};

    assign row0 = (show_white ? w0 : b0) | cursor0;
    assign row1 = (show_white ? w1 : b1) | cursor1;
    assign row2 = (show_white ? w2 : b2) | cursor2;
    assign row3 = (show_white ? w3 : b3) | cursor3;
    assign row4 = (show_white ? w4 : b4) | cursor4;
    assign row5 = (show_white ? w5 : b5) | cursor5;
    assign row6 = (show_white ? w6 : b6) | cursor6;
    assign row7 = (show_white ? w7 : b7) | cursor7;
    assign row8 = (show_white ? w8 : b8) | cursor8;

    always @(posedge clk) begin
        if (!rst_n) begin
            phase <= 1'b0;
            turn_white <= 1'b0;
            place_d <= 1'b0;
            pass_d <= 1'b0;

            b0 <= 9'd0; b1 <= 9'd0; b2 <= 9'd0; b3 <= 9'd0; b4 <= 9'd0; b5 <= 9'd0; b6 <= 9'd0; b7 <= 9'd0; b8 <= 9'd0;
            w0 <= 9'd0; w1 <= 9'd0; w2 <= 9'd0; w3 <= 9'd0; w4 <= 9'd0; w5 <= 9'd0; w6 <= 9'd0; w7 <= 9'd0; w8 <= 9'd0;

            placed_x <= 9'd0;
            placed_y <= 9'd0;
            dir <= 3'd0;
            subdir <= 2'd0;
            self_check <= 1'b0;
            stone_x <= 9'd0;
            stone_y <= 9'd0;
            has_lib <= 1'b0;
            self_lib_cnt_sat <= 2'd0;

            cap_cnt_sat <= 2'd0;
            cap_x1 <= 9'd0;
            cap_y1 <= 9'd0;

            state <= S_IDLE;
        end else begin
            phase <= ~phase;
            place_d <= place;
            pass_d <= pass;

            // Board writeback.
            b0 <= (b0 | set_b0) & ~(clr_b0 | undo_b0);
            b1 <= (b1 | set_b1) & ~(clr_b1 | undo_b1);
            b2 <= (b2 | set_b2) & ~(clr_b2 | undo_b2);
            b3 <= (b3 | set_b3) & ~(clr_b3 | undo_b3);
            b4 <= (b4 | set_b4) & ~(clr_b4 | undo_b4);
            b5 <= (b5 | set_b5) & ~(clr_b5 | undo_b5);
            b6 <= (b6 | set_b6) & ~(clr_b6 | undo_b6);
            b7 <= (b7 | set_b7) & ~(clr_b7 | undo_b7);
            b8 <= (b8 | set_b8) & ~(clr_b8 | undo_b8);

            w0 <= (w0 | set_w0) & ~(clr_w0 | undo_w0);
            w1 <= (w1 | set_w1) & ~(clr_w1 | undo_w1);
            w2 <= (w2 | set_w2) & ~(clr_w2 | undo_w2);
            w3 <= (w3 | set_w3) & ~(clr_w3 | undo_w3);
            w4 <= (w4 | set_w4) & ~(clr_w4 | undo_w4);
            w5 <= (w5 | set_w5) & ~(clr_w5 | undo_w5);
            w6 <= (w6 | set_w6) & ~(clr_w6 | undo_w6);
            w7 <= (w7 | set_w7) & ~(clr_w7 | undo_w7);
            w8 <= (w8 | set_w8) & ~(clr_w8 | undo_w8);

            if (state == S_IDLE) begin
                if (do_pass) begin
                    turn_white <= ~turn_white;
                end else if (do_place) begin
                    placed_x <= x;
                    placed_y <= y;

                    cap_cnt_sat <= 2'd0;
                    cap_x1 <= 9'd0;
                    cap_y1 <= 9'd0;
                    dir <= 3'd0;
                    state <= S_NEIGHBOR;
                end
            end else if (state == S_NEIGHBOR) begin
                if (dir == 3'd4) begin
                    // Self check: compute placed-stone liberties after any captures.
                    self_check <= 1'b1;
                    stone_x <= placed_x;
                    stone_y <= placed_y;
                    subdir <= 2'd0;
                    has_lib <= 1'b0;
                    self_lib_cnt_sat <= 2'd0;
                    state <= S_LIB_STEP;
                end else begin
                    // Opponent neighbor?
                    // qx/qy is (nx,ny) in this state.
                    if (neigh_valid && (turn_white ? q_black : q_white)) begin
                        self_check <= 1'b0;
                        stone_x <= nx;
                        stone_y <= ny;
                        subdir <= 2'd0;
                        has_lib <= 1'b0;
                        state <= S_LIB_STEP;
                    end else begin
                        dir <= dir + 3'd1;
                    end
                end
            end else if (state == S_LIB_STEP) begin
                // For single-stone groups, any empty adjacent point is a liberty.
                if (avalid && (~q_occ)) begin
                    has_lib <= 1'b1;
                    if (self_check) begin
                        if (self_lib_cnt_sat != 2'd2) self_lib_cnt_sat <= self_lib_cnt_sat + 2'd1;
                    end
                end
                if (subdir == 2'd3) begin
                    state <= S_DECIDE;
                end
                subdir <= subdir + 2'd1;
            end else if (state == S_DECIDE) begin
                if (!self_check) begin
                    // Capture single opponent stone if it has no liberties.
                    if (!has_lib) begin
                        if (cap_cnt_sat != 2'd2) cap_cnt_sat <= cap_cnt_sat + 2'd1;
                        if (cap_cnt_sat == 2'd0) begin
                            cap_x1 <= stone_x;
                            cap_y1 <= stone_y;
                        end
                    end

                    dir <= dir + 3'd1;
                    state <= S_NEIGHBOR;
                end else begin
                    // Suicide (no captures): undo.
                    if ((!has_lib) && (cap_cnt_sat == 2'd0)) begin
                        state <= S_IDLE;
                    end else begin
                        turn_white <= ~turn_white;
                        state <= S_IDLE;
                    end
                end
            end
        end
    end

endmodule

module go9x9_lite (
    input        clk,
    input        rst_n,
    input        up,
    input        down,
    input        left,
    input        right,
    input        place,
    input        pass,

    output       black,
    output       white,
    output [8:0] row0,
    output [8:0] row1,
    output [8:0] row2,
    output [8:0] row3,
    output [8:0] row4,
    output [8:0] row5,
    output [8:0] row6,
    output [8:0] row7,
    output [8:0] row8
);
    reg [8:0] cx;
    reg [8:0] cy;

    reg up_d, down_d, left_d, right_d;
    wire up_p    = up    & (~up_d);
    wire down_p  = down  & (~down_d);
    wire left_p  = left  & (~left_d);
    wire right_p = right & (~right_d);

    always @(posedge clk) begin
        if (!rst_n) begin
            cx <= 9'b000010000; // x=4
            cy <= 9'b000010000; // y=4
            up_d <= 1'b0;
            down_d <= 1'b0;
            left_d <= 1'b0;
            right_d <= 1'b0;
        end else begin
            up_d <= up;
            down_d <= down;
            left_d <= left;
            right_d <= right;

            if (left_p && (| (cx >> 1))) begin
                cx <= (cx >> 1);
            end else if (right_p && (| (cx << 1))) begin
                cx <= (cx << 1);
            end else if (up_p && (| (cy >> 1))) begin
                cy <= (cy >> 1);
            end else if (down_p && (| (cy << 1))) begin
                cy <= (cy << 1);
            end
        end
    end

    go9x9_lite_core core (
        .clk(clk),
        .rst_n(rst_n),
        .x(cx),
        .y(cy),
        .place(place),
        .pass(pass),
        .black(black),
        .white(white),
        .row0(row0),
        .row1(row1),
        .row2(row2),
        .row3(row3),
        .row4(row4),
        .row5(row5),
        .row6(row6),
        .row7(row7),
        .row8(row8)
    );
endmodule
