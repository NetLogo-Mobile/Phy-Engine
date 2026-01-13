// NVIDIA FP8 (E4M3-like, finite numbers) add.
// Format: [7]=sign, [6:3]=exp(bias=7), [2:0]=mantissa.
// exp==15 treated as NaN (canonical 0x7F), no Inf.

module fp8_add_top(
    input  [7:0] a,
    input  [7:0] b,
    output [7:0] y
);
    localparam integer BIAS = 7;
    localparam integer EMIN = -6;  // 1-bias
    localparam integer EMAX = 7;   // 14-bias
    localparam integer N = 10;     // internal frac bits

    reg [7:0] y_r;
    assign y = y_r;

    reg sa, sb;
    reg [3:0] ea_f, eb_f;
    reg [2:0] ma_f, mb_f;
    reg a_is_nan, b_is_nan;
    reg a_is_zero, b_is_zero;

    reg signed [7:0] ea, eb, e_out;
    reg [15:0] ma_int, mb_int;  // mantissa scaled by 2^N

    reg [31:0] mant_a, mant_b;
    reg signed [32:0] va, vb, vsum;
    reg [32:0] mant_work;
    reg s_out;

    reg [5:0] shift;
    reg sticky;
    reg [31:0] lost_mask;

    reg [31:0] sig_trunc;
    reg [31:0] rem;
    reg round_up;

    reg [3:0] exp_field;
    reg [2:0] mant_field;

    integer i;

    always @(a or b)
    begin
        // defaults
        y_r = 8'h00;
        sa = 1'b0;
        sb = 1'b0;
        ea_f = 4'h0;
        eb_f = 4'h0;
        ma_f = 3'h0;
        mb_f = 3'h0;
        a_is_nan = 1'b0;
        b_is_nan = 1'b0;
        a_is_zero = 1'b0;
        b_is_zero = 1'b0;
        ea = EMIN;
        eb = EMIN;
        ma_int = 16'h0;
        mb_int = 16'h0;
        mant_a = 32'h0;
        mant_b = 32'h0;
        va = 33'sd0;
        vb = 33'sd0;
        vsum = 33'sd0;
        mant_work = 33'd0;
        s_out = 1'b0;
        e_out = EMIN;
        shift = 6'd0;
        sticky = 1'b0;
        lost_mask = 32'h0;
        sig_trunc = 32'h0;
        rem = 32'h0;
        round_up = 1'b0;
        exp_field = 4'h0;
        mant_field = 3'h0;

        // decode
        sa = a[7];
        sb = b[7];
        ea_f = a[6:3];
        eb_f = b[6:3];
        ma_f = a[2:0];
        mb_f = b[2:0];

        a_is_nan = (ea_f == 4'hF);
        b_is_nan = (eb_f == 4'hF);
        a_is_zero = (ea_f == 4'h0) && (ma_f == 3'h0);
        b_is_zero = (eb_f == 4'h0) && (mb_f == 3'h0);

        if(ea_f == 4'h0) begin ea = EMIN; ma_int = {13'b0, ma_f} << (N - 3); end
        else if(ea_f != 4'hF)
        begin
            ea = {4'b0, ea_f};
            ea = ea - BIAS;
            ma_int = {12'b0, 1'b1, ma_f} << (N - 3);
        end

        if(eb_f == 4'h0) begin eb = EMIN; mb_int = {13'b0, mb_f} << (N - 3); end
        else if(eb_f != 4'hF)
        begin
            eb = {4'b0, eb_f};
            eb = eb - BIAS;
            mb_int = {12'b0, 1'b1, mb_f} << (N - 3);
        end

        // specials
        if(a_is_nan || b_is_nan) begin y_r = 8'h7F; end
        else if(a_is_zero && b_is_zero) begin y_r = 8'h00; end
        else if(a_is_zero) begin y_r = {sb, eb_f, mb_f}; end
        else if(b_is_zero) begin y_r = {sa, ea_f, ma_f}; end
        else
        begin
            // align to max exponent
            mant_a = ma_int;
            mant_b = mb_int;
            e_out = ea;

            if(ea > eb)
            begin
                shift = ea - eb;
                e_out = ea;
                if(shift >= 6'd31) begin mant_b = 0; end
                else if(shift != 0)
                begin
                    lost_mask = (32'h1 << shift) - 1;
                    sticky = ((mant_b & lost_mask) != 0);
                    mant_b = mant_b >> shift;
                    if(sticky) mant_b[0] = 1'b1;
                end
            end
            else if(eb > ea)
            begin
                shift = eb - ea;
                e_out = eb;
                if(shift >= 6'd31) begin mant_a = 0; end
                else if(shift != 0)
                begin
                    lost_mask = (32'h1 << shift) - 1;
                    sticky = ((mant_a & lost_mask) != 0);
                    mant_a = mant_a >> shift;
                    if(sticky) mant_a[0] = 1'b1;
                end
            end

            va = {1'b0, mant_a};
            if(sa) va = -va;
            vb = {1'b0, mant_b};
            if(sb) vb = -vb;
            vsum = va + vb;

            if(vsum == 0)
            begin
                y_r = 8'h00;
            end
            else
            begin
                s_out = vsum[32];
                mant_work = s_out ? -vsum : vsum;

                // normalize down if >=2.0
                for(i = 0; i < 8; i = i + 1)
                begin
                    if(mant_work >= (33'd1 << (N + 1)))
                    begin
                        sticky = mant_work[0];
                        mant_work = mant_work >> 1;
                        if(sticky) mant_work[0] = 1'b1;
                        e_out = e_out + 1;
                    end
                end

                // normalize up while allowed
                for(i = 0; i < 16; i = i + 1)
                begin
                    if(mant_work < (33'd1 << N) && e_out > EMIN)
                    begin
                        mant_work = mant_work << 1;
                        e_out = e_out - 1;
                    end
                end

                // underflow into subnorm
                if(e_out < EMIN)
                begin
                    shift = EMIN - e_out;
                    if(shift >= 6'd31) begin mant_work = 0; end
                    else if(shift != 0)
                    begin
                        lost_mask = (32'h1 << shift) - 1;
                        sticky = ((mant_work[31:0] & lost_mask) != 0);
                        mant_work = mant_work >> shift;
                        if(sticky) mant_work[0] = 1'b1;
                    end
                    e_out = EMIN;
                end

                sig_trunc = mant_work >> (N - 3);             // 0..15
                rem = mant_work[31:0] & ((32'h1 << (N - 3)) - 1);
                round_up = 1'b0;
                if(rem > (32'h1 << ((N - 3) - 1))) round_up = 1'b1;
                else if(rem == (32'h1 << ((N - 3) - 1))) round_up = sig_trunc[0];
                if(round_up) sig_trunc = sig_trunc + 1;

                if(sig_trunc >= 16)
                begin
                    sig_trunc = 8;
                    e_out = e_out + 1;
                end

                if(e_out > EMAX)
                begin
                    y_r = {s_out, 4'hE, 3'h7};
                end
                else
                begin
                    if(e_out == EMIN && sig_trunc < 8)
                    begin
                        mant_field = sig_trunc[2:0];
                        y_r = {s_out, 4'h0, mant_field};
                    end
                    else if(e_out == EMIN && sig_trunc == 8)
                    begin
                        y_r = {s_out, 4'h1, 3'h0};
                    end
                    else
                    begin
                        exp_field = e_out + BIAS;
                        mant_field = sig_trunc - 8;
                        y_r = {s_out, exp_field, mant_field};
                    end
                end
            end
        end
    end
endmodule
