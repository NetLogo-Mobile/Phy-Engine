module add22_via_sub(
    input  [21:0] a,
    input  [21:0] b,
    output [21:0] y
);
    wire [21:0] b_neg;
    assign b_neg = (~b) + 1'b1;  // two's complement, uses var+const (supported)
    assign y = a - b_neg;             // a + b
endmodule

module mul11x11(
    input  [10:0] a,
    input  [10:0] b,
    output [21:0] p
);
    wire [21:0] a_ext;
    assign a_ext = {11'd0, a};

    wire [21:0] pp0;
    wire [21:0] pp1;
    wire [21:0] pp2;
    wire [21:0] pp3;
    wire [21:0] pp4;
    wire [21:0] pp5;
    wire [21:0] pp6;
    wire [21:0] pp7;
    wire [21:0] pp8;
    wire [21:0] pp9;
    wire [21:0] pp10;

    assign pp0  = b[0]  ? (a_ext << 0)  : 22'd0;
    assign pp1  = b[1]  ? (a_ext << 1)  : 22'd0;
    assign pp2  = b[2]  ? (a_ext << 2)  : 22'd0;
    assign pp3  = b[3]  ? (a_ext << 3)  : 22'd0;
    assign pp4  = b[4]  ? (a_ext << 4)  : 22'd0;
    assign pp5  = b[5]  ? (a_ext << 5)  : 22'd0;
    assign pp6  = b[6]  ? (a_ext << 6)  : 22'd0;
    assign pp7  = b[7]  ? (a_ext << 7)  : 22'd0;
    assign pp8  = b[8]  ? (a_ext << 8)  : 22'd0;
    assign pp9  = b[9]  ? (a_ext << 9)  : 22'd0;
    assign pp10 = b[10] ? (a_ext << 10) : 22'd0;

    wire [21:0] s01;
    wire [21:0] s02;
    wire [21:0] s03;
    wire [21:0] s04;
    wire [21:0] s05;
    wire [21:0] s06;
    wire [21:0] s07;
    wire [21:0] s08;
    wire [21:0] s09;
    wire [21:0] s10;

    add22_via_sub a01(.a(pp0),  .b(pp1),  .y(s01));
    add22_via_sub a02(.a(s01),  .b(pp2),  .y(s02));
    add22_via_sub a03(.a(s02),  .b(pp3),  .y(s03));
    add22_via_sub a04(.a(s03),  .b(pp4),  .y(s04));
    add22_via_sub a05(.a(s04),  .b(pp5),  .y(s05));
    add22_via_sub a06(.a(s05),  .b(pp6),  .y(s06));
    add22_via_sub a07(.a(s06),  .b(pp7),  .y(s07));
    add22_via_sub a08(.a(s07),  .b(pp8),  .y(s08));
    add22_via_sub a09(.a(s08),  .b(pp9),  .y(s09));
    add22_via_sub a10(.a(s09),  .b(pp10), .y(s10));

    assign p = s10;
endmodule

module fp16_mul_unit(
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
    reg [8:0] exp_res;
    reg [10:0] mant_a;
    reg [10:0] mant_b;
    reg [21:0] prod;
    reg [14:0] mant_ext;
    reg [8:0] sh_sum;
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
    wire [21:0] prod_w;
    mul11x11 u_mul(.a(mant_a_n_w), .b(mant_b_n_w), .p(prod_w));

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
        else if ((inf_a && zero_b) || (inf_b && zero_a))
        begin
            y_r = QNAN;
        end
        else if (inf_a || inf_b)
        begin
            y_r = {sign_res, 5'h1F, 10'd0};
        end
        else if (zero_a || zero_b)
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
            // exp_res = exp_a_adj + exp_b_adj - 15 - sh_a - sh_b
            exp_res = {3'd0, exp_a_adj_w} - ((~{3'd0, exp_b_adj_w}) + 9'd1);
            exp_res = exp_res - 9'd15;
            sh_sum = {5'd0, sh_a_w} - ((~{5'd0, sh_b_w}) + 9'd1);  // sh_a + sh_b
            exp_res = exp_res - sh_sum;

            prod = prod_w;                  // Q20
            mant_ext = prod_w[21:7];        // Q13
            mant_ext[0] = mant_ext[0] | (|prod[6:0]);

            // Normalize if >=2.
            if (mant_ext[14])
            begin
                sticky = mant_ext[0];
                mant_ext = (mant_ext >> 1);
                mant_ext[0] = mant_ext[0] | sticky;
                exp_res = exp_res + 1;
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
