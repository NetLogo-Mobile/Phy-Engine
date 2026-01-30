module snake6x6 (
    input        clk,
    input        rst,

    input        up,
    input        down,
    input        left,
    input        right,

    output reg [5:0] row0,
    output reg [5:0] row1,
    output reg [5:0] row2,
    output reg [5:0] row3,
    output reg [5:0] row4,
    output reg [5:0] row5
);

    // state: 0=RUN, 1=SPAWN, 2=GAME_OVER
    reg [1:0] state;

    // dir: 0=UP 1=RIGHT 2=DOWN 3=LEFT
    reg [1:0] dir;

    // snake occupancy: 36-bit (idx=r*6+c, idx 0..35)
    reg [35:0] snake_bits;

    // head coordinate
    reg [2:0] head_r, head_c;

    // tail index 0..35
    reg [5:0] tail_idx;

    // length
    reg [5:0] len;

    // next_dir bits: 2 bits per cell (idx*2+1 : idx*2)
    reg [71:0] next_dir_bits;

    // food
    reg       food_valid;
    reg [5:0] food_idx;

    // spawn scan
    reg [5:0] scan_idx;
    reg [5:0] scan_cnt;

    // 6-bit LFSR
    reg [5:0] lfsr;
    wire      lfsr_fb;
    assign lfsr_fb = lfsr[5] ^ lfsr[4];

    wire [5:0] rand36;
    assign rand36 = (lfsr < 6'd36) ? lfsr : (lfsr - 6'd36);

    // ------------------------------------------------------------
    // next head coord from dir + wall check
    // ------------------------------------------------------------
    reg [2:0] next_hr, next_hc;
    reg       hit_wall;

    always @(*) begin
        next_hr  = head_r;
        next_hc  = head_c;
        hit_wall = 1'b0;

        case (dir)
            2'd0: begin // UP
                if (head_r == 3'd0) hit_wall = 1'b1;
                else next_hr = head_r - 3'd1;
            end
            2'd1: begin // RIGHT
                if (head_c == 3'd5) hit_wall = 1'b1;
                else next_hc = head_c + 3'd1;
            end
            2'd2: begin // DOWN
                if (head_r == 3'd5) hit_wall = 1'b1;
                else next_hr = head_r + 3'd1;
            end
            default: begin // LEFT
                if (head_c == 3'd0) hit_wall = 1'b1;
                else next_hc = head_c - 3'd1;
            end
        endcase
    end

    // ------------------------------------------------------------
    // idx computation via base+col (avoid shifts)
    // ------------------------------------------------------------
    reg [5:0] head_base, next_base;
    reg [5:0] head_idx, next_idx;

    always @(*) begin
        case (head_r)
            3'd0: head_base = 6'd0;
            3'd1: head_base = 6'd6;
            3'd2: head_base = 6'd12;
            3'd3: head_base = 6'd18;
            3'd4: head_base = 6'd24;
            default: head_base = 6'd30;
        endcase
        head_idx = head_base + {3'b000, head_c};

        case (next_hr)
            3'd0: next_base = 6'd0;
            3'd1: next_base = 6'd6;
            3'd2: next_base = 6'd12;
            3'd3: next_base = 6'd18;
            3'd4: next_base = 6'd24;
            default: next_base = 6'd30;
        endcase
        next_idx = next_base + {3'b000, next_hc};
    end

    // ------------------------------------------------------------
    // occupancy read via case (old tools friendly)
    // ------------------------------------------------------------
    reg occ_next;
    reg occ_scan;

    always @(*) begin
        // occ_next
        occ_next = 1'b0;
        case (next_idx)
            6'd0:  occ_next = snake_bits[0];
            6'd1:  occ_next = snake_bits[1];
            6'd2:  occ_next = snake_bits[2];
            6'd3:  occ_next = snake_bits[3];
            6'd4:  occ_next = snake_bits[4];
            6'd5:  occ_next = snake_bits[5];
            6'd6:  occ_next = snake_bits[6];
            6'd7:  occ_next = snake_bits[7];
            6'd8:  occ_next = snake_bits[8];
            6'd9:  occ_next = snake_bits[9];
            6'd10: occ_next = snake_bits[10];
            6'd11: occ_next = snake_bits[11];
            6'd12: occ_next = snake_bits[12];
            6'd13: occ_next = snake_bits[13];
            6'd14: occ_next = snake_bits[14];
            6'd15: occ_next = snake_bits[15];
            6'd16: occ_next = snake_bits[16];
            6'd17: occ_next = snake_bits[17];
            6'd18: occ_next = snake_bits[18];
            6'd19: occ_next = snake_bits[19];
            6'd20: occ_next = snake_bits[20];
            6'd21: occ_next = snake_bits[21];
            6'd22: occ_next = snake_bits[22];
            6'd23: occ_next = snake_bits[23];
            6'd24: occ_next = snake_bits[24];
            6'd25: occ_next = snake_bits[25];
            6'd26: occ_next = snake_bits[26];
            6'd27: occ_next = snake_bits[27];
            6'd28: occ_next = snake_bits[28];
            6'd29: occ_next = snake_bits[29];
            6'd30: occ_next = snake_bits[30];
            6'd31: occ_next = snake_bits[31];
            6'd32: occ_next = snake_bits[32];
            6'd33: occ_next = snake_bits[33];
            6'd34: occ_next = snake_bits[34];
            default: occ_next = snake_bits[35];
        endcase

        // occ_scan
        occ_scan = 1'b0;
        case (scan_idx)
            6'd0:  occ_scan = snake_bits[0];
            6'd1:  occ_scan = snake_bits[1];
            6'd2:  occ_scan = snake_bits[2];
            6'd3:  occ_scan = snake_bits[3];
            6'd4:  occ_scan = snake_bits[4];
            6'd5:  occ_scan = snake_bits[5];
            6'd6:  occ_scan = snake_bits[6];
            6'd7:  occ_scan = snake_bits[7];
            6'd8:  occ_scan = snake_bits[8];
            6'd9:  occ_scan = snake_bits[9];
            6'd10: occ_scan = snake_bits[10];
            6'd11: occ_scan = snake_bits[11];
            6'd12: occ_scan = snake_bits[12];
            6'd13: occ_scan = snake_bits[13];
            6'd14: occ_scan = snake_bits[14];
            6'd15: occ_scan = snake_bits[15];
            6'd16: occ_scan = snake_bits[16];
            6'd17: occ_scan = snake_bits[17];
            6'd18: occ_scan = snake_bits[18];
            6'd19: occ_scan = snake_bits[19];
            6'd20: occ_scan = snake_bits[20];
            6'd21: occ_scan = snake_bits[21];
            6'd22: occ_scan = snake_bits[22];
            6'd23: occ_scan = snake_bits[23];
            6'd24: occ_scan = snake_bits[24];
            6'd25: occ_scan = snake_bits[25];
            6'd26: occ_scan = snake_bits[26];
            6'd27: occ_scan = snake_bits[27];
            6'd28: occ_scan = snake_bits[28];
            6'd29: occ_scan = snake_bits[29];
            6'd30: occ_scan = snake_bits[30];
            6'd31: occ_scan = snake_bits[31];
            6'd32: occ_scan = snake_bits[32];
            6'd33: occ_scan = snake_bits[33];
            6'd34: occ_scan = snake_bits[34];
            default: occ_scan = snake_bits[35];
        endcase
    end

    // eating / step-into-tail / body_hit (old style assigns)
    wire eating;
    wire step_into_tail;
    wire body_hit;

    assign eating         = food_valid && (next_idx == food_idx);
    assign step_into_tail = (!eating) && (next_idx == tail_idx);
    assign body_hit       = occ_next && (!step_into_tail);

    // ------------------------------------------------------------
    // tail_dir read via case from next_dir_bits (2 bits per idx)
    // ------------------------------------------------------------
    reg [1:0] tail_dir;
    always @(*) begin
        case (tail_idx)
            6'd0:  tail_dir = next_dir_bits[1:0];
            6'd1:  tail_dir = next_dir_bits[3:2];
            6'd2:  tail_dir = next_dir_bits[5:4];
            6'd3:  tail_dir = next_dir_bits[7:6];
            6'd4:  tail_dir = next_dir_bits[9:8];
            6'd5:  tail_dir = next_dir_bits[11:10];
            6'd6:  tail_dir = next_dir_bits[13:12];
            6'd7:  tail_dir = next_dir_bits[15:14];
            6'd8:  tail_dir = next_dir_bits[17:16];
            6'd9:  tail_dir = next_dir_bits[19:18];
            6'd10: tail_dir = next_dir_bits[21:20];
            6'd11: tail_dir = next_dir_bits[23:22];
            6'd12: tail_dir = next_dir_bits[25:24];
            6'd13: tail_dir = next_dir_bits[27:26];
            6'd14: tail_dir = next_dir_bits[29:28];
            6'd15: tail_dir = next_dir_bits[31:30];
            6'd16: tail_dir = next_dir_bits[33:32];
            6'd17: tail_dir = next_dir_bits[35:34];
            6'd18: tail_dir = next_dir_bits[37:36];
            6'd19: tail_dir = next_dir_bits[39:38];
            6'd20: tail_dir = next_dir_bits[41:40];
            6'd21: tail_dir = next_dir_bits[43:42];
            6'd22: tail_dir = next_dir_bits[45:44];
            6'd23: tail_dir = next_dir_bits[47:46];
            6'd24: tail_dir = next_dir_bits[49:48];
            6'd25: tail_dir = next_dir_bits[51:50];
            6'd26: tail_dir = next_dir_bits[53:52];
            6'd27: tail_dir = next_dir_bits[55:54];
            6'd28: tail_dir = next_dir_bits[57:56];
            6'd29: tail_dir = next_dir_bits[59:58];
            6'd30: tail_dir = next_dir_bits[61:60];
            6'd31: tail_dir = next_dir_bits[63:62];
            6'd32: tail_dir = next_dir_bits[65:64];
            6'd33: tail_dir = next_dir_bits[67:66];
            6'd34: tail_dir = next_dir_bits[69:68];
            default: tail_dir = next_dir_bits[71:70];
        endcase
    end

    // tail next idx from tail_dir (dir: 0=UP 1=RIGHT 2=DOWN 3=LEFT)
    reg [5:0] tail_next_idx;
    always @(*) begin
        case (tail_dir)
            2'd0: tail_next_idx = tail_idx - 6'd6;
            2'd1: tail_next_idx = tail_idx + 6'd1;
            2'd2: tail_next_idx = tail_idx + 6'd6;
            default: tail_next_idx = tail_idx - 6'd1;
        endcase
    end

    // ------------------------------------------------------------
    // MAIN sequential
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            state <= 2'd1; // SPAWN
            dir   <= 2'd1; // RIGHT

            head_r <= 3'd2;
            head_c <= 3'd3;

            tail_idx <= 6'd14; // (2,2)
            len      <= 6'd2;

            // idx 14 and 15 set
            snake_bits <= 36'h0000C000;

            // clear pointer memory then set next_dir[14]=RIGHT (01)
            next_dir_bits <= 72'b0;
            next_dir_bits[29:28] <= 2'd1;

            food_valid <= 1'b0;
            food_idx   <= 6'd0;

            scan_idx <= 6'd0;
            scan_cnt <= 6'd0;

            lfsr <= 6'b100111; // non-zero seed

        end else begin
            // advance LFSR every cycle
            lfsr <= {lfsr[4:0], lfsr_fb};
            if (lfsr == 6'b0) lfsr <= 6'b100111;

            // direction update only in RUN, forbid reverse
            if (state == 2'd0) begin
                if (up && (dir != 2'd2)) dir <= 2'd0;
                else if (right && (dir != 2'd3)) dir <= 2'd1;
                else if (down && (dir != 2'd0)) dir <= 2'd2;
                else if (left && (dir != 2'd1)) dir <= 2'd3;
            end

            if (state == 2'd2) begin
                // GAME_OVER hold
                state <= 2'd2;

            end else if (state == 2'd1) begin
                // SPAWN (1 cell check per cycle)
                if (len == 6'd36) begin
                    state <= 2'd2;
                end else begin
                    if (!food_valid && (scan_cnt == 6'd0)) begin
                        scan_idx <= rand36;
                        scan_cnt <= 6'd0;
                    end

                    if (!occ_scan) begin
                        food_valid <= 1'b1;
                        food_idx   <= scan_idx;
                        state      <= 2'd0; // RUN
                        scan_cnt   <= 6'd0;
                    end else begin
                        // advance scan
                        if (scan_idx == 6'd35) scan_idx <= 6'd0;
                        else scan_idx <= scan_idx + 6'd1;

                        if (scan_cnt == 6'd35) begin
                            state <= 2'd2;
                            scan_cnt <= 6'd0;
                        end else begin
                            scan_cnt <= scan_cnt + 6'd1;
                        end
                    end
                end

            end else begin
                // RUN
                if (hit_wall || body_hit) begin
                    state <= 2'd2;
                end else begin
                    // 1) write pointer at old head cell -> dir (towards new head)
                    case (head_idx)
                        6'd0:  next_dir_bits[1:0]   <= dir;
                        6'd1:  next_dir_bits[3:2]   <= dir;
                        6'd2:  next_dir_bits[5:4]   <= dir;
                        6'd3:  next_dir_bits[7:6]   <= dir;
                        6'd4:  next_dir_bits[9:8]   <= dir;
                        6'd5:  next_dir_bits[11:10] <= dir;
                        6'd6:  next_dir_bits[13:12] <= dir;
                        6'd7:  next_dir_bits[15:14] <= dir;
                        6'd8:  next_dir_bits[17:16] <= dir;
                        6'd9:  next_dir_bits[19:18] <= dir;
                        6'd10: next_dir_bits[21:20] <= dir;
                        6'd11: next_dir_bits[23:22] <= dir;
                        6'd12: next_dir_bits[25:24] <= dir;
                        6'd13: next_dir_bits[27:26] <= dir;
                        6'd14: next_dir_bits[29:28] <= dir;
                        6'd15: next_dir_bits[31:30] <= dir;
                        6'd16: next_dir_bits[33:32] <= dir;
                        6'd17: next_dir_bits[35:34] <= dir;
                        6'd18: next_dir_bits[37:36] <= dir;
                        6'd19: next_dir_bits[39:38] <= dir;
                        6'd20: next_dir_bits[41:40] <= dir;
                        6'd21: next_dir_bits[43:42] <= dir;
                        6'd22: next_dir_bits[45:44] <= dir;
                        6'd23: next_dir_bits[47:46] <= dir;
                        6'd24: next_dir_bits[49:48] <= dir;
                        6'd25: next_dir_bits[51:50] <= dir;
                        6'd26: next_dir_bits[53:52] <= dir;
                        6'd27: next_dir_bits[55:54] <= dir;
                        6'd28: next_dir_bits[57:56] <= dir;
                        6'd29: next_dir_bits[59:58] <= dir;
                        6'd30: next_dir_bits[61:60] <= dir;
                        6'd31: next_dir_bits[63:62] <= dir;
                        6'd32: next_dir_bits[65:64] <= dir;
                        6'd33: next_dir_bits[67:66] <= dir;
                        6'd34: next_dir_bits[69:68] <= dir;
                        default: next_dir_bits[71:70] <= dir;
                    endcase

                    // 2) if not eating, clear tail first (so stepping into tail works)
                    if (!eating) begin
                        case (tail_idx)
                            6'd0:  snake_bits[0]  <= 1'b0;
                            6'd1:  snake_bits[1]  <= 1'b0;
                            6'd2:  snake_bits[2]  <= 1'b0;
                            6'd3:  snake_bits[3]  <= 1'b0;
                            6'd4:  snake_bits[4]  <= 1'b0;
                            6'd5:  snake_bits[5]  <= 1'b0;
                            6'd6:  snake_bits[6]  <= 1'b0;
                            6'd7:  snake_bits[7]  <= 1'b0;
                            6'd8:  snake_bits[8]  <= 1'b0;
                            6'd9:  snake_bits[9]  <= 1'b0;
                            6'd10: snake_bits[10] <= 1'b0;
                            6'd11: snake_bits[11] <= 1'b0;
                            6'd12: snake_bits[12] <= 1'b0;
                            6'd13: snake_bits[13] <= 1'b0;
                            6'd14: snake_bits[14] <= 1'b0;
                            6'd15: snake_bits[15] <= 1'b0;
                            6'd16: snake_bits[16] <= 1'b0;
                            6'd17: snake_bits[17] <= 1'b0;
                            6'd18: snake_bits[18] <= 1'b0;
                            6'd19: snake_bits[19] <= 1'b0;
                            6'd20: snake_bits[20] <= 1'b0;
                            6'd21: snake_bits[21] <= 1'b0;
                            6'd22: snake_bits[22] <= 1'b0;
                            6'd23: snake_bits[23] <= 1'b0;
                            6'd24: snake_bits[24] <= 1'b0;
                            6'd25: snake_bits[25] <= 1'b0;
                            6'd26: snake_bits[26] <= 1'b0;
                            6'd27: snake_bits[27] <= 1'b0;
                            6'd28: snake_bits[28] <= 1'b0;
                            6'd29: snake_bits[29] <= 1'b0;
                            6'd30: snake_bits[30] <= 1'b0;
                            6'd31: snake_bits[31] <= 1'b0;
                            6'd32: snake_bits[32] <= 1'b0;
                            6'd33: snake_bits[33] <= 1'b0;
                            6'd34: snake_bits[34] <= 1'b0;
                            default: snake_bits[35] <= 1'b0;
                        endcase
                    end

                    // 3) set new head
                    case (next_idx)
                        6'd0:  snake_bits[0]  <= 1'b1;
                        6'd1:  snake_bits[1]  <= 1'b1;
                        6'd2:  snake_bits[2]  <= 1'b1;
                        6'd3:  snake_bits[3]  <= 1'b1;
                        6'd4:  snake_bits[4]  <= 1'b1;
                        6'd5:  snake_bits[5]  <= 1'b1;
                        6'd6:  snake_bits[6]  <= 1'b1;
                        6'd7:  snake_bits[7]  <= 1'b1;
                        6'd8:  snake_bits[8]  <= 1'b1;
                        6'd9:  snake_bits[9]  <= 1'b1;
                        6'd10: snake_bits[10] <= 1'b1;
                        6'd11: snake_bits[11] <= 1'b1;
                        6'd12: snake_bits[12] <= 1'b1;
                        6'd13: snake_bits[13] <= 1'b1;
                        6'd14: snake_bits[14] <= 1'b1;
                        6'd15: snake_bits[15] <= 1'b1;
                        6'd16: snake_bits[16] <= 1'b1;
                        6'd17: snake_bits[17] <= 1'b1;
                        6'd18: snake_bits[18] <= 1'b1;
                        6'd19: snake_bits[19] <= 1'b1;
                        6'd20: snake_bits[20] <= 1'b1;
                        6'd21: snake_bits[21] <= 1'b1;
                        6'd22: snake_bits[22] <= 1'b1;
                        6'd23: snake_bits[23] <= 1'b1;
                        6'd24: snake_bits[24] <= 1'b1;
                        6'd25: snake_bits[25] <= 1'b1;
                        6'd26: snake_bits[26] <= 1'b1;
                        6'd27: snake_bits[27] <= 1'b1;
                        6'd28: snake_bits[28] <= 1'b1;
                        6'd29: snake_bits[29] <= 1'b1;
                        6'd30: snake_bits[30] <= 1'b1;
                        6'd31: snake_bits[31] <= 1'b1;
                        6'd32: snake_bits[32] <= 1'b1;
                        6'd33: snake_bits[33] <= 1'b1;
                        6'd34: snake_bits[34] <= 1'b1;
                        default: snake_bits[35] <= 1'b1;
                    endcase

                    // 4) update head coords
                    head_r <= next_hr;
                    head_c <= next_hc;

                    if (eating) begin
                        food_valid <= 1'b0;
                        len <= len + 6'd1;
                        state <= 2'd1;
                        scan_cnt <= 6'd0; // re-init spawn
                    end else begin
                        // move tail
                        tail_idx <= tail_next_idx;
                        len <= len;
                        state <= 2'd0;
                    end
                end
            end
        end
    end

    // ------------------------------------------------------------
    // OUTPUT: snake + food (food overlay)
    // ------------------------------------------------------------
    reg [2:0] food_r, food_c;
    reg [5:0] food_col_mask;
    reg [5:0] f0,f1,f2,f3,f4,f5;

    always @(*) begin
        // idx -> (r,c)
        if (food_idx >= 6'd30) begin food_r = 3'd5; food_c = food_idx - 6'd30; end
        else if (food_idx >= 6'd24) begin food_r = 3'd4; food_c = food_idx - 6'd24; end
        else if (food_idx >= 6'd18) begin food_r = 3'd3; food_c = food_idx - 6'd18; end
        else if (food_idx >= 6'd12) begin food_r = 3'd2; food_c = food_idx - 6'd12; end
        else if (food_idx >= 6'd6)  begin food_r = 3'd1; food_c = food_idx - 6'd6;  end
        else begin food_r = 3'd0; food_c = food_idx[2:0]; end

        case (food_c)
            3'd0: food_col_mask = 6'b000001;
            3'd1: food_col_mask = 6'b000010;
            3'd2: food_col_mask = 6'b000100;
            3'd3: food_col_mask = 6'b001000;
            3'd4: food_col_mask = 6'b010000;
            default: food_col_mask = 6'b100000;
        endcase

        f0=0; f1=0; f2=0; f3=0; f4=0; f5=0;
        if (food_valid) begin
            if (food_r == 3'd0) f0 = food_col_mask;
            else if (food_r == 3'd1) f1 = food_col_mask;
            else if (food_r == 3'd2) f2 = food_col_mask;
            else if (food_r == 3'd3) f3 = food_col_mask;
            else if (food_r == 3'd4) f4 = food_col_mask;
            else f5 = food_col_mask;
        end

        // snake rows by constant slices
        row0 = snake_bits[5:0]    | f0;
        row1 = snake_bits[11:6]   | f1;
        row2 = snake_bits[17:12]  | f2;
        row3 = snake_bits[23:18]  | f3;
        row4 = snake_bits[29:24]  | f4;
        row5 = snake_bits[35:30]  | f5;
    end

endmodule

