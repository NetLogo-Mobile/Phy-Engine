module div_stage_u24_u11(
    input  [11:0] rem_in,
    input  [23:0] q_in,
    input         dividend_bit,
    input  [10:0] div,
    output [11:0] rem_out,
    output [23:0] q_out
);
    wire [11:0] rem_shift;
    assign rem_shift = {rem_in[10:0], dividend_bit};
    wire        ge;
    assign ge = (rem_shift >= {1'b0, div});
    wire [11:0] rem_sub;
    assign rem_sub = rem_shift - {1'b0, div};
    assign rem_out = ge ? rem_sub : rem_shift;
    assign q_out   = {q_in[22:0], ge};
endmodule

module fp16_div_unit(
    input  [15:0] a,
    input  [15:0] b,
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
    reg [8:0] exp_tmp;
    reg [8:0] exp_res;
    reg [10:0] mant_a;
    reg [10:0] mant_b;

    reg [23:0] num;
    reg [14:0] q;
    reg [10:0] rem;
    reg [14:0] mant_ext;
    reg [5:0] uf_sh;
    reg [14:0] tmp;
    reg sticky_u;

    reg guard;
    reg roundb;
    reg sticky;
    reg lsb;
    reg inc;
    reg [11:0] mant_main;
    reg [11:0] mant_round;
    reg [4:0] exp_field;
    reg [9:0] frac_field;

    wire [4:0] exp_a_f_w;
    wire [4:0] exp_b_f_w;
    wire [9:0] frac_a_w;
    wire [9:0] frac_b_w;
    wire [5:0] exp_a_adj_w;
    wire [5:0] exp_b_adj_w;
    wire [10:0] mant_a_w;
    wire [10:0] mant_b_w;
    assign exp_a_f_w = a[14:10];
    assign exp_b_f_w = b[14:10];
    assign frac_a_w = a[9:0];
    assign frac_b_w = b[9:0];
    assign exp_a_adj_w = (exp_a_f_w == 0) ? 6'd1 : {1'b0, exp_a_f_w};
    assign exp_b_adj_w = (exp_b_f_w == 0) ? 6'd1 : {1'b0, exp_b_f_w};
    assign mant_a_w = (exp_a_f_w == 0) ? {1'b0, frac_a_w} : {1'b1, frac_a_w};
    assign mant_b_w = (exp_b_f_w == 0) ? {1'b0, frac_b_w} : {1'b1, frac_b_w};

    // Normalize subnormal mantissas to [1,2) by shifting left, and compensate via exponent adjustment.
    wire sub_a_w;
    wire sub_b_w;
    assign sub_a_w = (exp_a_f_w == 0) && (frac_a_w != 0);
    assign sub_b_w = (exp_b_f_w == 0) && (frac_b_w != 0);
    wire [3:0] sh_a_w;
    wire [3:0] sh_b_w;
    assign sh_a_w =
        sub_a_w ?
            (frac_a_w[9] ? 4'd1 :
             frac_a_w[8] ? 4'd2 :
             frac_a_w[7] ? 4'd3 :
             frac_a_w[6] ? 4'd4 :
             frac_a_w[5] ? 4'd5 :
             frac_a_w[4] ? 4'd6 :
             frac_a_w[3] ? 4'd7 :
             frac_a_w[2] ? 4'd8 :
             frac_a_w[1] ? 4'd9 :
             frac_a_w[0] ? 4'd10 : 4'd0)
        : 4'd0;
    assign sh_b_w =
        sub_b_w ?
            (frac_b_w[9] ? 4'd1 :
             frac_b_w[8] ? 4'd2 :
             frac_b_w[7] ? 4'd3 :
             frac_b_w[6] ? 4'd4 :
             frac_b_w[5] ? 4'd5 :
             frac_b_w[4] ? 4'd6 :
             frac_b_w[3] ? 4'd7 :
             frac_b_w[2] ? 4'd8 :
             frac_b_w[1] ? 4'd9 :
             frac_b_w[0] ? 4'd10 : 4'd0)
        : 4'd0;
    wire [10:0] mant_a_n_w;
    wire [10:0] mant_b_n_w;
    assign mant_a_n_w =
        sub_a_w ?
            (frac_a_w[9] ? {frac_a_w, 1'b0} :
             frac_a_w[8] ? {frac_a_w[8:0], 2'b0} :
             frac_a_w[7] ? {frac_a_w[7:0], 3'b0} :
             frac_a_w[6] ? {frac_a_w[6:0], 4'b0} :
             frac_a_w[5] ? {frac_a_w[5:0], 5'b0} :
             frac_a_w[4] ? {frac_a_w[4:0], 6'b0} :
             frac_a_w[3] ? {frac_a_w[3:0], 7'b0} :
             frac_a_w[2] ? {frac_a_w[2:0], 8'b0} :
             frac_a_w[1] ? {frac_a_w[1:0], 9'b0} :
             frac_a_w[0] ? {frac_a_w[0], 10'b0} : 11'd0)
        : mant_a_w;
    assign mant_b_n_w =
        sub_b_w ?
            (frac_b_w[9] ? {frac_b_w, 1'b0} :
             frac_b_w[8] ? {frac_b_w[8:0], 2'b0} :
             frac_b_w[7] ? {frac_b_w[7:0], 3'b0} :
             frac_b_w[6] ? {frac_b_w[6:0], 4'b0} :
             frac_b_w[5] ? {frac_b_w[5:0], 5'b0} :
             frac_b_w[4] ? {frac_b_w[4:0], 6'b0} :
             frac_b_w[3] ? {frac_b_w[3:0], 7'b0} :
             frac_b_w[2] ? {frac_b_w[2:0], 8'b0} :
             frac_b_w[1] ? {frac_b_w[1:0], 9'b0} :
             frac_b_w[0] ? {frac_b_w[0], 10'b0} : 11'd0)
        : mant_b_w;

    wire [23:0] num_w;
    // Widen before shifting: in this subset (and Verilog), (a << 13) keeps the width of a.
    assign num_w = {mant_a_n_w, 13'd0};
    // Long division: 24-bit dividend / 11-bit divisor -> 24-bit quotient and 12-bit remainder.
    wire [11:0] r0;
    wire [23:0] q0;
    assign r0 = 12'd0;
    assign q0 = 24'd0;
    wire [11:0] r1;  wire [23:0] q1;
    wire [11:0] r2;  wire [23:0] q2;
    wire [11:0] r3;  wire [23:0] q3;
    wire [11:0] r4;  wire [23:0] q4;
    wire [11:0] r5;  wire [23:0] q5;
    wire [11:0] r6;  wire [23:0] q6;
    wire [11:0] r7;  wire [23:0] q7;
    wire [11:0] r8;  wire [23:0] q8;
    wire [11:0] r9;  wire [23:0] q9;
    wire [11:0] r10; wire [23:0] q10;
    wire [11:0] r11; wire [23:0] q11;
    wire [11:0] r12; wire [23:0] q12;
    wire [11:0] r13; wire [23:0] q13;
    wire [11:0] r14; wire [23:0] q14;
    wire [11:0] r15; wire [23:0] q15;
    wire [11:0] r16; wire [23:0] q16;
    wire [11:0] r17; wire [23:0] q17;
    wire [11:0] r18; wire [23:0] q18;
    wire [11:0] r19; wire [23:0] q19;
    wire [11:0] r20; wire [23:0] q20;
    wire [11:0] r21; wire [23:0] q21;
    wire [11:0] r22; wire [23:0] q22;
    wire [11:0] r23; wire [23:0] q23;
    wire [11:0] r24; wire [23:0] q24;

    div_stage_u24_u11 d1 (.rem_in(r0),  .q_in(q0),  .dividend_bit(num_w[23]), .div(mant_b_n_w), .rem_out(r1),  .q_out(q1));
    div_stage_u24_u11 d2 (.rem_in(r1),  .q_in(q1),  .dividend_bit(num_w[22]), .div(mant_b_n_w), .rem_out(r2),  .q_out(q2));
    div_stage_u24_u11 d3 (.rem_in(r2),  .q_in(q2),  .dividend_bit(num_w[21]), .div(mant_b_n_w), .rem_out(r3),  .q_out(q3));
    div_stage_u24_u11 d4 (.rem_in(r3),  .q_in(q3),  .dividend_bit(num_w[20]), .div(mant_b_n_w), .rem_out(r4),  .q_out(q4));
    div_stage_u24_u11 d5 (.rem_in(r4),  .q_in(q4),  .dividend_bit(num_w[19]), .div(mant_b_n_w), .rem_out(r5),  .q_out(q5));
    div_stage_u24_u11 d6 (.rem_in(r5),  .q_in(q5),  .dividend_bit(num_w[18]), .div(mant_b_n_w), .rem_out(r6),  .q_out(q6));
    div_stage_u24_u11 d7 (.rem_in(r6),  .q_in(q6),  .dividend_bit(num_w[17]), .div(mant_b_n_w), .rem_out(r7),  .q_out(q7));
    div_stage_u24_u11 d8 (.rem_in(r7),  .q_in(q7),  .dividend_bit(num_w[16]), .div(mant_b_n_w), .rem_out(r8),  .q_out(q8));
    div_stage_u24_u11 d9 (.rem_in(r8),  .q_in(q8),  .dividend_bit(num_w[15]), .div(mant_b_n_w), .rem_out(r9),  .q_out(q9));
    div_stage_u24_u11 d10(.rem_in(r9),  .q_in(q9),  .dividend_bit(num_w[14]), .div(mant_b_n_w), .rem_out(r10), .q_out(q10));
    div_stage_u24_u11 d11(.rem_in(r10), .q_in(q10), .dividend_bit(num_w[13]), .div(mant_b_n_w), .rem_out(r11), .q_out(q11));
    div_stage_u24_u11 d12(.rem_in(r11), .q_in(q11), .dividend_bit(num_w[12]), .div(mant_b_n_w), .rem_out(r12), .q_out(q12));
    div_stage_u24_u11 d13(.rem_in(r12), .q_in(q12), .dividend_bit(num_w[11]), .div(mant_b_n_w), .rem_out(r13), .q_out(q13));
    div_stage_u24_u11 d14(.rem_in(r13), .q_in(q13), .dividend_bit(num_w[10]), .div(mant_b_n_w), .rem_out(r14), .q_out(q14));
    div_stage_u24_u11 d15(.rem_in(r14), .q_in(q14), .dividend_bit(num_w[9]),  .div(mant_b_n_w), .rem_out(r15), .q_out(q15));
    div_stage_u24_u11 d16(.rem_in(r15), .q_in(q15), .dividend_bit(num_w[8]),  .div(mant_b_n_w), .rem_out(r16), .q_out(q16));
    div_stage_u24_u11 d17(.rem_in(r16), .q_in(q16), .dividend_bit(num_w[7]),  .div(mant_b_n_w), .rem_out(r17), .q_out(q17));
    div_stage_u24_u11 d18(.rem_in(r17), .q_in(q17), .dividend_bit(num_w[6]),  .div(mant_b_n_w), .rem_out(r18), .q_out(q18));
    div_stage_u24_u11 d19(.rem_in(r18), .q_in(q18), .dividend_bit(num_w[5]),  .div(mant_b_n_w), .rem_out(r19), .q_out(q19));
    div_stage_u24_u11 d20(.rem_in(r19), .q_in(q19), .dividend_bit(num_w[4]),  .div(mant_b_n_w), .rem_out(r20), .q_out(q20));
    div_stage_u24_u11 d21(.rem_in(r20), .q_in(q20), .dividend_bit(num_w[3]),  .div(mant_b_n_w), .rem_out(r21), .q_out(q21));
    div_stage_u24_u11 d22(.rem_in(r21), .q_in(q21), .dividend_bit(num_w[2]),  .div(mant_b_n_w), .rem_out(r22), .q_out(q22));
    div_stage_u24_u11 d23(.rem_in(r22), .q_in(q22), .dividend_bit(num_w[1]),  .div(mant_b_n_w), .rem_out(r23), .q_out(q23));
    div_stage_u24_u11 d24(.rem_in(r23), .q_in(q23), .dividend_bit(num_w[0]),  .div(mant_b_n_w), .rem_out(r24), .q_out(q24));

    wire [14:0] q15_w;
    wire [11:0] rem_w;
    assign q15_w = q24[14:0];
    assign rem_w = r24;

    always @*
    begin
        sign_a  = a[15];
        exp_a_f = exp_a_f_w;
        frac_a  = frac_a_w;

        sign_b  = b[15];
        exp_b_f = exp_b_f_w;
        frac_b  = frac_b_w;

        nan_a  = (exp_a_f == 5'h1F) && (frac_a != 0);
        nan_b  = (exp_b_f == 5'h1F) && (frac_b != 0);
        inf_a  = (exp_a_f == 5'h1F) && (frac_a == 0);
        inf_b  = (exp_b_f == 5'h1F) && (frac_b == 0);
        zero_a = (exp_a_f == 0) && (frac_a == 0);
        zero_b = (exp_b_f == 0) && (frac_b == 0);

        sign_res = sign_a ^ sign_b;
        y_r = 16'd0;

        if (nan_a || nan_b)
        begin
            y_r = QNAN;
        end
        else if ((inf_a && inf_b) || (zero_a && zero_b))
        begin
            y_r = QNAN;
        end
        else if (inf_a)
        begin
            y_r = {sign_res, 5'h1F, 10'd0};
        end
        else if (inf_b)
        begin
            y_r = {sign_res, 15'd0};
        end
        else if (zero_b)
        begin
            y_r = {sign_res, 5'h1F, 10'd0};
        end
        else if (zero_a)
        begin
            y_r = {sign_res, 15'd0};
        end
        else
        begin
            exp_a_adj = exp_a_adj_w;
            exp_b_adj = exp_b_adj_w;
            mant_a = mant_a_n_w;
            mant_b = mant_b_n_w;

            // Biased exponent (signed, two's complement in 9 bits):
            // exp_res = exp_a_adj - exp_b_adj + 15 + sh_b - sh_a
            exp_res = {3'd0, exp_a_adj_w} + 9'd15;
            exp_res = exp_res - {3'd0, exp_b_adj_w};
            if (sh_b_w >= sh_a_w) exp_res = exp_res + (sh_b_w - sh_a_w);
            else exp_res = exp_res - (sh_a_w - sh_b_w);

            // Mantissa ratio in Q13 using unrolled restoring division (avoid broken '/' and loops).
            num = num_w;
            q = q15_w;
            rem = rem_w[10:0];

            mant_ext = q15_w;
            mant_ext[0] = mant_ext[0] | (|rem_w);

            // Normalize: mantissa ratio in [0.5,2) -> hidden bit should be mant_ext[13].
            // If ratio < 1, shift-left once and decrement exponent (may underflow into subnormal, handled below).
            if ((mant_ext != 0) && (mant_ext[13] == 1'b0))
            begin
                mant_ext = (mant_ext << 1);
                exp_res = exp_res - 1;
            end

            // Underflow to subnormal: if exp_res <= 0, shift mantissa right by (1 - exp_res) with sticky and set exp_res=0.
            if (exp_res[8] || (exp_res == 0))
            begin
                uf_sh = ((~exp_res) + 9'd1) + 9'd1;  // 1 - exp_res
                case(uf_sh)
                    0:  begin tmp = mant_ext;                  sticky_u = 1'b0;              end
                    1:  begin tmp = {1'b0, mant_ext[14:1]};    sticky_u = mant_ext[0];        end
                    2:  begin tmp = {2'b0, mant_ext[14:2]};    sticky_u = (|mant_ext[1:0]);   end
                    3:  begin tmp = {3'b0, mant_ext[14:3]};    sticky_u = (|mant_ext[2:0]);   end
                    4:  begin tmp = {4'b0, mant_ext[14:4]};    sticky_u = (|mant_ext[3:0]);   end
                    5:  begin tmp = {5'b0, mant_ext[14:5]};    sticky_u = (|mant_ext[4:0]);   end
                    6:  begin tmp = {6'b0, mant_ext[14:6]};    sticky_u = (|mant_ext[5:0]);   end
                    7:  begin tmp = {7'b0, mant_ext[14:7]};    sticky_u = (|mant_ext[6:0]);   end
                    8:  begin tmp = {8'b0, mant_ext[14:8]};    sticky_u = (|mant_ext[7:0]);   end
                    9:  begin tmp = {9'b0, mant_ext[14:9]};    sticky_u = (|mant_ext[8:0]);   end
                    10: begin tmp = {10'b0, mant_ext[14:10]};  sticky_u = (|mant_ext[9:0]);   end
                    11: begin tmp = {11'b0, mant_ext[14:11]};  sticky_u = (|mant_ext[10:0]);  end
                    12: begin tmp = {12'b0, mant_ext[14:12]};  sticky_u = (|mant_ext[11:0]);  end
                    13: begin tmp = {13'b0, mant_ext[14:13]};  sticky_u = (|mant_ext[12:0]);  end
                    14: begin tmp = {14'b0, mant_ext[14]};     sticky_u = (|mant_ext[13:0]);  end
                    default: begin tmp = 15'd0;                sticky_u = (|mant_ext);        end
                endcase
                mant_ext = tmp;
                mant_ext[0] = tmp[0] | sticky_u;
                exp_res = 0;
            end

            // Round.
            guard  = mant_ext[2];
            roundb = mant_ext[1];
            sticky = mant_ext[0];
            lsb    = mant_ext[3];
            inc    = guard & (roundb | sticky | lsb);

            mant_main = {1'b0, mant_ext[13:3]};
            mant_round = inc ? (mant_main + 12'd1) : mant_main;
            if (mant_round[11])
            begin
                mant_round = (mant_round >> 1);
                exp_res = exp_res + 1;
            end

            if (!exp_res[8] && (exp_res >= 9'd31))
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
                if ((exp_res == 9'd1) && (mant_round[10] == 1'b0)) exp_field = 5'd0;
                else exp_field = exp_res[4:0];
                y_r = {sign_res, exp_field, frac_field};
            end
        end
    end
endmodule
