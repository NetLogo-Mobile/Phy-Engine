module go9x9_core (
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

    // 9x9 Go (rules-focused, gate-budget aware):
    // - alternating turns, occupied-point prohibition
    // - capture by liberties for full connected groups (orthogonal connectivity)
    // - suicide prohibited
    // - simple ko (one-ply) enforced for the classic single-stone ko shape
    //
    // Display:
    // - `black`/`white` are time-multiplexed enables (phase=0/1).
    // - `row*` shows stones for the active color; cursor blinks on current player's phase.

    reg phase;
    reg turn_white;

    // Board bitplanes.
    reg [8:0] b0, b1, b2, b3, b4, b5, b6, b7, b8;
    reg [8:0] w0, w1, w2, w3, w4, w5, w6, w7, w8;

    // Simple-ko point for the player-to-move (one ply).
    reg       ko_valid;
    reg [8:0] ko_x;
    reg [8:0] ko_y;

    // Edge detect.
    reg place_d;
    reg pass_d;
    wire place_pulse = place & (~place_d);
    wire pass_pulse  = pass  & (~pass_d);

    // Latched move coordinate.
    reg [8:0] placed_x;
    reg [8:0] placed_y;

    // Capture counting for the last move (saturating to 2), and first captured position.
    reg [1:0] cap_cnt_sat;
    reg [8:0] cap_x1;
    reg [8:0] cap_y1;

    // Connected-group traversal (frontier + visited masks).
    reg       search_white;
    reg       grp_self;
    reg [2:0] dir;
    reg [1:0] subdir;

    reg [8:0] f0, f1, f2, f3, f4, f5, f6, f7, f8;  // frontier
    reg [8:0] v0, v1, v2, v3, v4, v5, v6, v7, v8;  // visited

    // Scan cursor for frontier/visited.
    reg [8:0] scan_x;
    reg [8:0] scan_y;
    reg [8:0] cur_x;
    reg [8:0] cur_y;

    // Liberty tracking: keep 0/1/2+ distinct liberties (exact up to 2).
    reg       lib_found0;
    reg       lib_multi;
    reg [8:0] lib0_x;
    reg [8:0] lib0_y;

    // FSM.
    parameter S_IDLE     = 4'd0;
    parameter S_CAP_DIR  = 4'd1;
    parameter S_GRP_SCAN = 4'd2;
    parameter S_GRP_NEI  = 4'd3;
    parameter S_GRP_DONE = 4'd4;
    parameter S_DEL_SCAN = 4'd5;
    reg [3:0] state;

    // Display mux.
    assign black = ~phase;
    assign white = phase;
    wire show_white = phase;

    wire q_valid = (|x) & (|y);

    wire [8:0] occ0 = b0 | w0;
    wire [8:0] occ1 = b1 | w1;
    wire [8:0] occ2 = b2 | w2;
    wire [8:0] occ3 = b3 | w3;
    wire [8:0] occ4 = b4 | w4;
    wire [8:0] occ5 = b5 | w5;
    wire [8:0] occ6 = b6 | w6;
    wire [8:0] occ7 = b7 | w7;
    wire [8:0] occ8 = b8 | w8;

    wire [8:0] sel_occ =
        (occ0 & {9{y[0]}}) | (occ1 & {9{y[1]}}) | (occ2 & {9{y[2]}}) | (occ3 & {9{y[3]}}) |
        (occ4 & {9{y[4]}}) | (occ5 & {9{y[5]}}) | (occ6 & {9{y[6]}}) | (occ7 & {9{y[7]}}) |
        (occ8 & {9{y[8]}});
    wire q_occ = q_valid & (|(sel_occ & x));
    wire q_ko  = ko_valid & (x == ko_x) & (y == ko_y);

    wire do_place = (state == S_IDLE) & place_pulse & q_valid & (~q_occ) & (~q_ko);
    wire do_pass  = (state == S_IDLE) & pass_pulse;

    wire cursor_show = (state == S_IDLE) & q_valid & (~q_occ) & (phase == turn_white);
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

    // Neighbor (nx,ny) from placed_x/placed_y for dir scan.
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

    wire opp_is_white = ~turn_white;
    wire [8:0] opp0 = opp_is_white ? w0 : b0;
    wire [8:0] opp1 = opp_is_white ? w1 : b1;
    wire [8:0] opp2 = opp_is_white ? w2 : b2;
    wire [8:0] opp3 = opp_is_white ? w3 : b3;
    wire [8:0] opp4 = opp_is_white ? w4 : b4;
    wire [8:0] opp5 = opp_is_white ? w5 : b5;
    wire [8:0] opp6 = opp_is_white ? w6 : b6;
    wire [8:0] opp7 = opp_is_white ? w7 : b7;
    wire [8:0] opp8 = opp_is_white ? w8 : b8;

    wire [8:0] sel_opp =
        (opp0 & {9{ny[0]}}) | (opp1 & {9{ny[1]}}) | (opp2 & {9{ny[2]}}) | (opp3 & {9{ny[3]}}) |
        (opp4 & {9{ny[4]}}) | (opp5 & {9{ny[5]}}) | (opp6 & {9{ny[6]}}) | (opp7 & {9{ny[7]}}) |
        (opp8 & {9{ny[8]}});
    wire opp_at_neigh = neigh_valid & (|(sel_opp & nx));

    // Onehot scan helpers.
    wire scan_last_x = scan_x[8];
    wire scan_last_y = scan_y[8];
    wire scan_done   = scan_last_x & scan_last_y;

    // Frontier bit at scan position.
    wire [8:0] f_scan_sel =
        (f0 & {9{scan_y[0]}}) | (f1 & {9{scan_y[1]}}) | (f2 & {9{scan_y[2]}}) | (f3 & {9{scan_y[3]}}) |
        (f4 & {9{scan_y[4]}}) | (f5 & {9{scan_y[5]}}) | (f6 & {9{scan_y[6]}}) | (f7 & {9{scan_y[7]}}) |
        (f8 & {9{scan_y[8]}});
    wire f_scan_bit = |(f_scan_sel & scan_x);

    // Visited bit at scan position (used for deletion).
    wire [8:0] v_scan_sel =
        (v0 & {9{scan_y[0]}}) | (v1 & {9{scan_y[1]}}) | (v2 & {9{scan_y[2]}}) | (v3 & {9{scan_y[3]}}) |
        (v4 & {9{scan_y[4]}}) | (v5 & {9{scan_y[5]}}) | (v6 & {9{scan_y[6]}}) | (v7 & {9{scan_y[7]}}) |
        (v8 & {9{scan_y[8]}});
    wire v_scan_bit = |(v_scan_sel & scan_x);

    // Per-neighbor (ax,ay) from (cur_x,cur_y) and subdir.
    reg [8:0] ax;
    reg [8:0] ay;
    reg       avalid;
    always @(*) begin
        ax = cur_x;
        ay = cur_y;
        avalid = 1'b0;
        if (subdir == 2'd0) begin
            ax = (cur_x >> 1);
            ay = cur_y;
            avalid = (|ax);
        end else if (subdir == 2'd1) begin
            ax = (cur_x << 1);
            ay = cur_y;
            avalid = (|ax);
        end else if (subdir == 2'd2) begin
            ax = cur_x;
            ay = (cur_y >> 1);
            avalid = (|ay);
        end else begin
            ax = cur_x;
            ay = (cur_y << 1);
            avalid = (|ay);
        end
    end

    wire [8:0] s0 = search_white ? w0 : b0;
    wire [8:0] s1 = search_white ? w1 : b1;
    wire [8:0] s2 = search_white ? w2 : b2;
    wire [8:0] s3 = search_white ? w3 : b3;
    wire [8:0] s4 = search_white ? w4 : b4;
    wire [8:0] s5 = search_white ? w5 : b5;
    wire [8:0] s6 = search_white ? w6 : b6;
    wire [8:0] s7 = search_white ? w7 : b7;
    wire [8:0] s8 = search_white ? w8 : b8;

    wire [8:0] sel_s =
        (s0 & {9{ay[0]}}) | (s1 & {9{ay[1]}}) | (s2 & {9{ay[2]}}) | (s3 & {9{ay[3]}}) |
        (s4 & {9{ay[4]}}) | (s5 & {9{ay[5]}}) | (s6 & {9{ay[6]}}) | (s7 & {9{ay[7]}}) |
        (s8 & {9{ay[8]}});
    wire at_stone = avalid & (|(sel_s & ax));

    wire [8:0] sel_v =
        (v0 & {9{ay[0]}}) | (v1 & {9{ay[1]}}) | (v2 & {9{ay[2]}}) | (v3 & {9{ay[3]}}) |
        (v4 & {9{ay[4]}}) | (v5 & {9{ay[5]}}) | (v6 & {9{ay[6]}}) | (v7 & {9{ay[7]}}) |
        (v8 & {9{ay[8]}});
    wire at_visited = avalid & (|(sel_v & ax));

    wire [8:0] sel_o =
        (occ0 & {9{ay[0]}}) | (occ1 & {9{ay[1]}}) | (occ2 & {9{ay[2]}}) | (occ3 & {9{ay[3]}}) |
        (occ4 & {9{ay[4]}}) | (occ5 & {9{ay[5]}}) | (occ6 & {9{ay[6]}}) | (occ7 & {9{ay[7]}}) |
        (occ8 & {9{ay[8]}});
    wire at_occ = avalid & (|(sel_o & ax));
    wire at_empty = avalid & (~at_occ);

    always @(posedge clk) begin
        if (!rst_n) begin
            phase <= 1'b0;
            turn_white <= 1'b0;

            b0 <= 9'd0; b1 <= 9'd0; b2 <= 9'd0; b3 <= 9'd0; b4 <= 9'd0; b5 <= 9'd0; b6 <= 9'd0; b7 <= 9'd0; b8 <= 9'd0;
            w0 <= 9'd0; w1 <= 9'd0; w2 <= 9'd0; w3 <= 9'd0; w4 <= 9'd0; w5 <= 9'd0; w6 <= 9'd0; w7 <= 9'd0; w8 <= 9'd0;

            ko_valid <= 1'b0;
            ko_x <= 9'd0;
            ko_y <= 9'd0;

            place_d <= 1'b0;
            pass_d <= 1'b0;
            placed_x <= 9'd0;
            placed_y <= 9'd0;

            cap_cnt_sat <= 2'd0;
            cap_x1 <= 9'd0;
            cap_y1 <= 9'd0;

            search_white <= 1'b0;
            grp_self <= 1'b0;
            dir <= 3'd0;
            subdir <= 2'd0;

            f0 <= 9'd0; f1 <= 9'd0; f2 <= 9'd0; f3 <= 9'd0; f4 <= 9'd0; f5 <= 9'd0; f6 <= 9'd0; f7 <= 9'd0; f8 <= 9'd0;
            v0 <= 9'd0; v1 <= 9'd0; v2 <= 9'd0; v3 <= 9'd0; v4 <= 9'd0; v5 <= 9'd0; v6 <= 9'd0; v7 <= 9'd0; v8 <= 9'd0;
            scan_x <= 9'd0;
            scan_y <= 9'd0;
            cur_x <= 9'd0;
            cur_y <= 9'd0;

            lib_found0 <= 1'b0;
            lib_multi <= 1'b0;
            lib0_x <= 9'd0;
            lib0_y <= 9'd0;

            state <= S_IDLE;
        end else begin
            phase <= ~phase;
            place_d <= place;
            pass_d <= pass;

            if (state == S_IDLE) begin
                if (do_pass) begin
                    turn_white <= ~turn_white;
                    ko_valid <= 1'b0;
                end else if (do_place) begin
                    placed_x <= x;
                    placed_y <= y;

                    // Place stone.
                    if (turn_white) begin
                        if (y[0]) w0 <= (w0 | x);
                        else if (y[1]) w1 <= (w1 | x);
                        else if (y[2]) w2 <= (w2 | x);
                        else if (y[3]) w3 <= (w3 | x);
                        else if (y[4]) w4 <= (w4 | x);
                        else if (y[5]) w5 <= (w5 | x);
                        else if (y[6]) w6 <= (w6 | x);
                        else if (y[7]) w7 <= (w7 | x);
                        else if (y[8]) w8 <= (w8 | x);
                    end else begin
                        if (y[0]) b0 <= (b0 | x);
                        else if (y[1]) b1 <= (b1 | x);
                        else if (y[2]) b2 <= (b2 | x);
                        else if (y[3]) b3 <= (b3 | x);
                        else if (y[4]) b4 <= (b4 | x);
                        else if (y[5]) b5 <= (b5 | x);
                        else if (y[6]) b6 <= (b6 | x);
                        else if (y[7]) b7 <= (b7 | x);
                        else if (y[8]) b8 <= (b8 | x);
                    end

                    // Start capture scan.
                    cap_cnt_sat <= 2'd0;
                    cap_x1 <= 9'd0;
                    cap_y1 <= 9'd0;
                    dir <= 3'd0;
                    state <= S_CAP_DIR;
                end
            end else if (state == S_CAP_DIR) begin
                if (dir == 3'd4) begin
                    // Start self-group traversal after captures.
                    grp_self <= 1'b1;
                    search_white <= turn_white;
                    f0 <= placed_x & {9{placed_y[0]}};
                    f1 <= placed_x & {9{placed_y[1]}};
                    f2 <= placed_x & {9{placed_y[2]}};
                    f3 <= placed_x & {9{placed_y[3]}};
                    f4 <= placed_x & {9{placed_y[4]}};
                    f5 <= placed_x & {9{placed_y[5]}};
                    f6 <= placed_x & {9{placed_y[6]}};
                    f7 <= placed_x & {9{placed_y[7]}};
                    f8 <= placed_x & {9{placed_y[8]}};
                    v0 <= 9'd0; v1 <= 9'd0; v2 <= 9'd0; v3 <= 9'd0; v4 <= 9'd0; v5 <= 9'd0; v6 <= 9'd0; v7 <= 9'd0; v8 <= 9'd0;
                    lib_found0 <= 1'b0;
                    lib_multi <= 1'b0;
                    lib0_x <= 9'd0;
                    lib0_y <= 9'd0;
                    scan_x <= 9'b000000001;
                    scan_y <= 9'b000000001;
                    state <= S_GRP_SCAN;
                end else if (opp_at_neigh) begin
                    // Start opponent-group traversal from (nx,ny).
                    grp_self <= 1'b0;
                    search_white <= opp_is_white;
                    f0 <= nx & {9{ny[0]}};
                    f1 <= nx & {9{ny[1]}};
                    f2 <= nx & {9{ny[2]}};
                    f3 <= nx & {9{ny[3]}};
                    f4 <= nx & {9{ny[4]}};
                    f5 <= nx & {9{ny[5]}};
                    f6 <= nx & {9{ny[6]}};
                    f7 <= nx & {9{ny[7]}};
                    f8 <= nx & {9{ny[8]}};
                    v0 <= 9'd0; v1 <= 9'd0; v2 <= 9'd0; v3 <= 9'd0; v4 <= 9'd0; v5 <= 9'd0; v6 <= 9'd0; v7 <= 9'd0; v8 <= 9'd0;
                    lib_found0 <= 1'b0;
                    lib_multi <= 1'b0;
                    lib0_x <= 9'd0;
                    lib0_y <= 9'd0;
                    scan_x <= 9'b000000001;
                    scan_y <= 9'b000000001;
                    state <= S_GRP_SCAN;
                end else begin
                    dir <= dir + 3'd1;
                end
            end else if (state == S_GRP_SCAN) begin
                if (f_scan_bit) begin
                    cur_x <= scan_x;
                    cur_y <= scan_y;
                    subdir <= 2'd0;

                    // Pop from frontier, add to visited.
                    if (scan_y[0]) begin f0 <= (f0 & ~scan_x); v0 <= (v0 | scan_x); end
                    else if (scan_y[1]) begin f1 <= (f1 & ~scan_x); v1 <= (v1 | scan_x); end
                    else if (scan_y[2]) begin f2 <= (f2 & ~scan_x); v2 <= (v2 | scan_x); end
                    else if (scan_y[3]) begin f3 <= (f3 & ~scan_x); v3 <= (v3 | scan_x); end
                    else if (scan_y[4]) begin f4 <= (f4 & ~scan_x); v4 <= (v4 | scan_x); end
                    else if (scan_y[5]) begin f5 <= (f5 & ~scan_x); v5 <= (v5 | scan_x); end
                    else if (scan_y[6]) begin f6 <= (f6 & ~scan_x); v6 <= (v6 | scan_x); end
                    else if (scan_y[7]) begin f7 <= (f7 & ~scan_x); v7 <= (v7 | scan_x); end
                    else if (scan_y[8]) begin f8 <= (f8 & ~scan_x); v8 <= (v8 | scan_x); end

                    state <= S_GRP_NEI;
                end else if (scan_done) begin
                    state <= S_GRP_DONE;
                end else begin
                    if (scan_last_x) begin
                        scan_x <= 9'b000000001;
                        scan_y <= (scan_y << 1);
                    end else begin
                        scan_x <= (scan_x << 1);
                    end
                end
            end else if (state == S_GRP_NEI) begin
                // Process one neighbor per cycle (subdir 0..3).
                if (at_empty) begin
                    if (!lib_found0) begin
                        lib_found0 <= 1'b1;
                        lib0_x <= ax;
                        lib0_y <= ay;
                    end else if (!lib_multi) begin
                        if (!((ax == lib0_x) && (ay == lib0_y))) begin
                            lib_multi <= 1'b1;
                        end
                    end
                end

                if (at_stone && (~at_visited)) begin
                    // Add to frontier.
                    if (ay[0]) f0 <= (f0 | ax);
                    else if (ay[1]) f1 <= (f1 | ax);
                    else if (ay[2]) f2 <= (f2 | ax);
                    else if (ay[3]) f3 <= (f3 | ax);
                    else if (ay[4]) f4 <= (f4 | ax);
                    else if (ay[5]) f5 <= (f5 | ax);
                    else if (ay[6]) f6 <= (f6 | ax);
                    else if (ay[7]) f7 <= (f7 | ax);
                    else if (ay[8]) f8 <= (f8 | ax);
                end

                if (subdir == 2'd3) begin
                    scan_x <= 9'b000000001;
                    scan_y <= 9'b000000001;
                    state <= S_GRP_SCAN;
                end
                subdir <= subdir + 2'd1;
            end else if (state == S_GRP_DONE) begin
                if (!grp_self) begin
                    // Opponent group: if no liberties, delete it.
                    if (!lib_found0) begin
                        scan_x <= 9'b000000001;
                        scan_y <= 9'b000000001;
                        state <= S_DEL_SCAN;
                    end else begin
                        dir <= dir + 3'd1;
                        state <= S_CAP_DIR;
                    end
                end else begin
                    // Self group: suicide check (only possible if no captures).
                    if ((cap_cnt_sat == 2'd0) && (!lib_found0)) begin
                        // Undo the placed stone.
                        if (turn_white) begin
                            if (placed_y[0]) w0 <= (w0 & ~placed_x);
                            else if (placed_y[1]) w1 <= (w1 & ~placed_x);
                            else if (placed_y[2]) w2 <= (w2 & ~placed_x);
                            else if (placed_y[3]) w3 <= (w3 & ~placed_x);
                            else if (placed_y[4]) w4 <= (w4 & ~placed_x);
                            else if (placed_y[5]) w5 <= (w5 & ~placed_x);
                            else if (placed_y[6]) w6 <= (w6 & ~placed_x);
                            else if (placed_y[7]) w7 <= (w7 & ~placed_x);
                            else if (placed_y[8]) w8 <= (w8 & ~placed_x);
                        end else begin
                            if (placed_y[0]) b0 <= (b0 & ~placed_x);
                            else if (placed_y[1]) b1 <= (b1 & ~placed_x);
                            else if (placed_y[2]) b2 <= (b2 & ~placed_x);
                            else if (placed_y[3]) b3 <= (b3 & ~placed_x);
                            else if (placed_y[4]) b4 <= (b4 & ~placed_x);
                            else if (placed_y[5]) b5 <= (b5 & ~placed_x);
                            else if (placed_y[6]) b6 <= (b6 & ~placed_x);
                            else if (placed_y[7]) b7 <= (b7 & ~placed_x);
                            else if (placed_y[8]) b8 <= (b8 & ~placed_x);
                        end
                        state <= S_IDLE;
                    end else begin
                        // Legal move: update ko (for next player) and toggle turn.
                        if ((cap_cnt_sat == 2'd1) && lib_found0 && (~lib_multi)) begin
                            ko_valid <= 1'b1;
                            ko_x <= cap_x1;
                            ko_y <= cap_y1;
                        end else begin
                            ko_valid <= 1'b0;
                            ko_x <= 9'd0;
                            ko_y <= 9'd0;
                        end
                        turn_white <= ~turn_white;
                        state <= S_IDLE;
                    end
                end
            end else if (state == S_DEL_SCAN) begin
                if (v_scan_bit) begin
                    // Clear this visited bit.
                    if (scan_y[0]) v0 <= (v0 & ~scan_x);
                    else if (scan_y[1]) v1 <= (v1 & ~scan_x);
                    else if (scan_y[2]) v2 <= (v2 & ~scan_x);
                    else if (scan_y[3]) v3 <= (v3 & ~scan_x);
                    else if (scan_y[4]) v4 <= (v4 & ~scan_x);
                    else if (scan_y[5]) v5 <= (v5 & ~scan_x);
                    else if (scan_y[6]) v6 <= (v6 & ~scan_x);
                    else if (scan_y[7]) v7 <= (v7 & ~scan_x);
                    else if (scan_y[8]) v8 <= (v8 & ~scan_x);

                    // Delete stone from opponent board.
                    if (opp_is_white) begin
                        if (scan_y[0]) w0 <= (w0 & ~scan_x);
                        else if (scan_y[1]) w1 <= (w1 & ~scan_x);
                        else if (scan_y[2]) w2 <= (w2 & ~scan_x);
                        else if (scan_y[3]) w3 <= (w3 & ~scan_x);
                        else if (scan_y[4]) w4 <= (w4 & ~scan_x);
                        else if (scan_y[5]) w5 <= (w5 & ~scan_x);
                        else if (scan_y[6]) w6 <= (w6 & ~scan_x);
                        else if (scan_y[7]) w7 <= (w7 & ~scan_x);
                        else if (scan_y[8]) w8 <= (w8 & ~scan_x);
                    end else begin
                        if (scan_y[0]) b0 <= (b0 & ~scan_x);
                        else if (scan_y[1]) b1 <= (b1 & ~scan_x);
                        else if (scan_y[2]) b2 <= (b2 & ~scan_x);
                        else if (scan_y[3]) b3 <= (b3 & ~scan_x);
                        else if (scan_y[4]) b4 <= (b4 & ~scan_x);
                        else if (scan_y[5]) b5 <= (b5 & ~scan_x);
                        else if (scan_y[6]) b6 <= (b6 & ~scan_x);
                        else if (scan_y[7]) b7 <= (b7 & ~scan_x);
                        else if (scan_y[8]) b8 <= (b8 & ~scan_x);
                    end

                    // Capture count (saturating) and first captured coordinate.
                    if (cap_cnt_sat != 2'd2) cap_cnt_sat <= cap_cnt_sat + 2'd1;
                    if (cap_cnt_sat == 2'd0) begin
                        cap_x1 <= scan_x;
                        cap_y1 <= scan_y;
                    end

                    // Restart scan.
                    scan_x <= 9'b000000001;
                    scan_y <= 9'b000000001;
                end else if (scan_done) begin
                    dir <= dir + 3'd1;
                    state <= S_CAP_DIR;
                end else begin
                    if (scan_last_x) begin
                        scan_x <= 9'b000000001;
                        scan_y <= (scan_y << 1);
                    end else begin
                        scan_x <= (scan_x << 1);
                    end
                end
            end
        end
    end

endmodule

module go9x9 (
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

    go9x9_core core (
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

