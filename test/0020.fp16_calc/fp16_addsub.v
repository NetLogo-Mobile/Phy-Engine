module fp16_addsub_unit(
    input  [15:0] a,
    input  [15:0] b,
    input         sub,
    output [15:0] y
);
    localparam QNAN = 16'h7E00;

    reg [15:0] y_r;
    assign y = y_r;

    reg sign_a;
    reg sign_b;
    reg sign_res;
    reg [4:0] exp_a_f;
    reg [4:0] exp_b_f;
    reg [9:0] frac_a;
    reg [9:0] frac_b;
    reg nan_a;
    reg nan_b;
    reg inf_a;
    reg inf_b;
    reg zero_a;
    reg zero_b;

    reg [5:0] exp_a_adj;
    reg [5:0] exp_b_adj;
    reg [5:0] exp_res;
    reg [10:0] mant_a;
    reg [10:0] mant_b;
    reg [14:0] ext_a;
    reg [14:0] ext_b;
    reg [14:0] mant_res_ext;

    reg [5:0] diff;
    reg [14:0] tmp;
    reg sticky;

    reg [4:0] shl;

    reg guard;
    reg roundb;
    reg sticky_r;
    reg lsb;
    reg inc;
    reg [11:0] mant_main;
    reg [11:0] mant_round;

    reg [4:0] exp_field;
    reg [9:0] frac_field;

    always @*
    begin
        sign_a  = a[15];
        exp_a_f = a[14:10];
        frac_a  = a[9:0];

        sign_b  = b[15] ^ sub;
        exp_b_f = b[14:10];
        frac_b  = b[9:0];

        nan_a  = (exp_a_f == 5'h1F) && (frac_a != 0);
        nan_b  = (exp_b_f == 5'h1F) && (frac_b != 0);
        inf_a  = (exp_a_f == 5'h1F) && (frac_a == 0);
        inf_b  = (exp_b_f == 5'h1F) && (frac_b == 0);
        zero_a = (exp_a_f == 0) && (frac_a == 0);
        zero_b = (exp_b_f == 0) && (frac_b == 0);

        y_r = 16'd0;

        if (nan_a || nan_b)
        begin
            y_r = QNAN;
        end
        else if (inf_a && inf_b)
        begin
            y_r = (sign_a != sign_b) ? QNAN : {sign_a, 5'h1F, 10'd0};
        end
        else if (inf_a)
        begin
            y_r = {sign_a, 5'h1F, 10'd0};
        end
        else if (inf_b)
        begin
            y_r = {sign_b, 5'h1F, 10'd0};
        end
        else if (zero_a && zero_b)
        begin
            y_r = {sign_a & sign_b, 15'd0};
        end
        else if (zero_a)
        begin
            y_r = {sign_b, exp_b_f, frac_b};
        end
        else if (zero_b)
        begin
            y_r = {sign_a, exp_a_f, frac_a};
        end
        else
        begin
            // Subnormals are handled as exp_adj=1, hidden=0.
            exp_a_adj = (exp_a_f == 0) ? 6'd1 : {1'b0, exp_a_f};
            exp_b_adj = (exp_b_f == 0) ? 6'd1 : {1'b0, exp_b_f};

            mant_a = (exp_a_f == 0) ? {1'b0, frac_a} : {1'b1, frac_a};
            mant_b = (exp_b_f == 0) ? {1'b0, frac_b} : {1'b1, frac_b};

            ext_a = {1'b0, mant_a, 3'b000};
            ext_b = {1'b0, mant_b, 3'b000};

            // Align.
            if (exp_a_adj > exp_b_adj)
            begin
                exp_res = exp_a_adj;
                diff = exp_a_adj - exp_b_adj;
                // Shift-right with sticky, without dynamic shifts (PE synth limitation).
                case(diff)
                    0:  begin tmp = ext_b;                     sticky = 1'b0;             end
                    1:  begin tmp = {1'b0, ext_b[14:1]};        sticky = ext_b[0];          end
                    2:  begin tmp = {2'b0, ext_b[14:2]};        sticky = (|ext_b[1:0]);     end
                    3:  begin tmp = {3'b0, ext_b[14:3]};        sticky = (|ext_b[2:0]);     end
                    4:  begin tmp = {4'b0, ext_b[14:4]};        sticky = (|ext_b[3:0]);     end
                    5:  begin tmp = {5'b0, ext_b[14:5]};        sticky = (|ext_b[4:0]);     end
                    6:  begin tmp = {6'b0, ext_b[14:6]};        sticky = (|ext_b[5:0]);     end
                    7:  begin tmp = {7'b0, ext_b[14:7]};        sticky = (|ext_b[6:0]);     end
                    8:  begin tmp = {8'b0, ext_b[14:8]};        sticky = (|ext_b[7:0]);     end
                    9:  begin tmp = {9'b0, ext_b[14:9]};        sticky = (|ext_b[8:0]);     end
                    10: begin tmp = {10'b0, ext_b[14:10]};      sticky = (|ext_b[9:0]);     end
                    11: begin tmp = {11'b0, ext_b[14:11]};      sticky = (|ext_b[10:0]);    end
                    12: begin tmp = {12'b0, ext_b[14:12]};      sticky = (|ext_b[11:0]);    end
                    13: begin tmp = {13'b0, ext_b[14:13]};      sticky = (|ext_b[12:0]);    end
                    14: begin tmp = {14'b0, ext_b[14]};         sticky = (|ext_b[13:0]);    end
                    default: begin tmp = 15'd0;                 sticky = (|ext_b);          end
                endcase
                ext_b = tmp;
                ext_b[0] = tmp[0] | sticky;
            end
            else if (exp_b_adj > exp_a_adj)
            begin
                exp_res = exp_b_adj;
                diff = exp_b_adj - exp_a_adj;
                // Shift-right with sticky, without dynamic shifts (PE synth limitation).
                case(diff)
                    0:  begin tmp = ext_a;                     sticky = 1'b0;             end
                    1:  begin tmp = {1'b0, ext_a[14:1]};        sticky = ext_a[0];          end
                    2:  begin tmp = {2'b0, ext_a[14:2]};        sticky = (|ext_a[1:0]);     end
                    3:  begin tmp = {3'b0, ext_a[14:3]};        sticky = (|ext_a[2:0]);     end
                    4:  begin tmp = {4'b0, ext_a[14:4]};        sticky = (|ext_a[3:0]);     end
                    5:  begin tmp = {5'b0, ext_a[14:5]};        sticky = (|ext_a[4:0]);     end
                    6:  begin tmp = {6'b0, ext_a[14:6]};        sticky = (|ext_a[5:0]);     end
                    7:  begin tmp = {7'b0, ext_a[14:7]};        sticky = (|ext_a[6:0]);     end
                    8:  begin tmp = {8'b0, ext_a[14:8]};        sticky = (|ext_a[7:0]);     end
                    9:  begin tmp = {9'b0, ext_a[14:9]};        sticky = (|ext_a[8:0]);     end
                    10: begin tmp = {10'b0, ext_a[14:10]};      sticky = (|ext_a[9:0]);     end
                    11: begin tmp = {11'b0, ext_a[14:11]};      sticky = (|ext_a[10:0]);    end
                    12: begin tmp = {12'b0, ext_a[14:12]};      sticky = (|ext_a[11:0]);    end
                    13: begin tmp = {13'b0, ext_a[14:13]};      sticky = (|ext_a[12:0]);    end
                    14: begin tmp = {14'b0, ext_a[14]};         sticky = (|ext_a[13:0]);    end
                    default: begin tmp = 15'd0;                 sticky = (|ext_a);          end
                endcase
                ext_a = tmp;
                ext_a[0] = tmp[0] | sticky;
            end
            else
            begin
                exp_res = exp_a_adj;
            end

            // Add/sub aligned mantissas.
            if (sign_a == sign_b)
            begin
                mant_res_ext = ext_a - ((~ext_b) + 15'd1);
                sign_res = sign_a;
            end
            else
            begin
                if (ext_a >= ext_b)
                begin
                    mant_res_ext = ext_a - ext_b;
                    sign_res = sign_a;
                end
                else
                begin
                    mant_res_ext = ext_b - ext_a;
                    sign_res = sign_b;
                end
            end

            if (mant_res_ext == 0)
            begin
                // Exact cancellation => +0
                y_r = 16'd0;
            end
            else
            begin
                // Normalize (hidden target at bit13; bit14 is carry).
                if (mant_res_ext[14])
                begin
                    sticky = mant_res_ext[0];
                    mant_res_ext = (mant_res_ext >> 1);
                    mant_res_ext[0] = mant_res_ext[0] | sticky;
                    exp_res = exp_res + 1;
                end
                else
                begin
                    // Determine how much to left-shift to put the leading 1 at bit13.
                    if (mant_res_ext[13]) shl = 0;
                    else if (mant_res_ext[12]) shl = 1;
                    else if (mant_res_ext[11]) shl = 2;
                    else if (mant_res_ext[10]) shl = 3;
                    else if (mant_res_ext[9]) shl = 4;
                    else if (mant_res_ext[8]) shl = 5;
                    else if (mant_res_ext[7]) shl = 6;
                    else if (mant_res_ext[6]) shl = 7;
                    else if (mant_res_ext[5]) shl = 8;
                    else if (mant_res_ext[4]) shl = 9;
                    else if (mant_res_ext[3]) shl = 10;
                    else if (mant_res_ext[2]) shl = 11;
                    else if (mant_res_ext[1]) shl = 12;
                    else if (mant_res_ext[0]) shl = 13;
                    else shl = 0;

                    // Limit shift so exponent doesn't go below 1 (subnormal boundary).
                    if (exp_res <= 1) shl = 0;
                    else if (shl > (exp_res - 1)) shl = (exp_res - 1);

                    if (shl != 0)
                    begin
                        // Variable shift-left rewritten as constant shifts.
                        case(shl)
                            1:  mant_res_ext = {mant_res_ext[13:0], 1'b0};
                            2:  mant_res_ext = {mant_res_ext[12:0], 2'b0};
                            3:  mant_res_ext = {mant_res_ext[11:0], 3'b0};
                            4:  mant_res_ext = {mant_res_ext[10:0], 4'b0};
                            5:  mant_res_ext = {mant_res_ext[9:0],  5'b0};
                            6:  mant_res_ext = {mant_res_ext[8:0],  6'b0};
                            7:  mant_res_ext = {mant_res_ext[7:0],  7'b0};
                            8:  mant_res_ext = {mant_res_ext[6:0],  8'b0};
                            9:  mant_res_ext = {mant_res_ext[5:0],  9'b0};
                            10: mant_res_ext = {mant_res_ext[4:0],  10'b0};
                            11: mant_res_ext = {mant_res_ext[3:0],  11'b0};
                            12: mant_res_ext = {mant_res_ext[2:0],  12'b0};
                            13: mant_res_ext = {mant_res_ext[1:0],  13'b0};
                            default: mant_res_ext = mant_res_ext;
                        endcase
                        exp_res = exp_res - shl;
                    end
                end

                // Round-to-nearest-even: keep 11 bits (hidden+frac) and GRS in [2:0].
                guard    = mant_res_ext[2];
                roundb   = mant_res_ext[1];
                sticky_r = mant_res_ext[0];
                lsb      = mant_res_ext[3];
                inc      = guard & (roundb | sticky_r | lsb);

                mant_main = {1'b0, mant_res_ext[13:3]};  // 12 bits (top is unused unless carry)
                mant_round = inc ? (mant_main + 12'd1) : mant_main;

                if (mant_round[11])
                begin
                    mant_round = (mant_round >> 1);
                    exp_res = exp_res + 1;
                end

                if (exp_res >= 31)
                begin
                    y_r = {sign_res, 5'h1F, 10'd0};
                end
                else if (mant_round[10:0] == 0)
                begin
                    y_r = {sign_res, 15'd0};
                end
                else
                begin
                    frac_field = mant_round[9:0];
                    if ((exp_res == 1) && (mant_round[10] == 1'b0))
                    begin
                        exp_field = 5'd0;  // subnormal
                    end
                    else
                    begin
                        exp_field = exp_res[4:0];
                    end
                    y_r = {sign_res, exp_field, frac_field};
                end
            end
        end
    end
endmodule
