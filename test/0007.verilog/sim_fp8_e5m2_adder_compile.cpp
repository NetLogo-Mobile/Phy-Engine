#include <cstdint>
#include <string>
#include <string_view>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

namespace
{
std::string to_string(char8_t const* p, std::size_t n) { return std::string(reinterpret_cast<char const*>(p), n); }

bool starts_with(std::string const& s, std::string_view prefix)
{
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

std::size_t bit_index_from_port_name(std::string const& pn)
{
    auto const lb = pn.find('[');
    if(lb == std::string::npos) { return 0; }
    return static_cast<std::size_t>(std::stoi(pn.substr(lb + 1, pn.size() - lb - 2)));
}

void set_u8(::phy_engine::verilog::digital::instance_state& top, std::string_view base, std::uint8_t v)
{
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::input) { continue; }

        std::size_t const idx = bit_index_from_port_name(pn);
        bool const bit = ((v >> idx) & 1u) != 0u;
        top.state.values.index_unchecked(p.signal) =
            bit ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state;
    }
}

std::uint8_t get_u8(::phy_engine::verilog::digital::instance_state& top, std::string_view base)
{
    std::uint8_t out{};
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }

        std::size_t const idx = bit_index_from_port_name(pn);
        auto const v = top.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return 0xFFu; }
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { out |= static_cast<std::uint8_t>(1u << idx); }
    }
    return out;
}
}  // namespace

int main()
{
    // FP8 E5M2 adder (RNE rounding)
    // Format: [7]=sign, [6:2]=exp (5 bits), [1:0]=frac (2 bits)
    // Bias = 15
    decltype(auto) src = u8R"(
module fp8_e5m2_adder (
    input  logic [7:0] a,
    input  logic [7:0] b,
    output logic [7:0] y
);

    localparam int EXP_W = 5;
    localparam int FRAC_W = 2;
    localparam int BIAS   = 15;

    // We keep significand as: (hidden + frac) = 1+2 = 3 bits
    // plus GRS bits for rounding => 3 bits
    localparam int SIG_W  = 1 + FRAC_W; // 3
    localparam int GRS_W  = 3;          // guard, round, sticky
    localparam int EXT_W  = SIG_W + GRS_W + 2; // extra headroom for carry/normalization

    // -------- helper: classify --------
    function automatic logic is_nan(input logic [7:0] x);
        logic [4:0] e; logic [1:0] f;
        begin
            e = x[6:2]; f = x[1:0];
            is_nan = (e == 5'h1F) && (f != 2'b00);
        end
    endfunction

    function automatic logic is_inf(input logic [7:0] x);
        logic [4:0] e; logic [1:0] f;
        begin
            e = x[6:2]; f = x[1:0];
            is_inf = (e == 5'h1F) && (f == 2'b00);
        end
    endfunction

    function automatic logic is_zero(input logic [7:0] x);
        begin
            is_zero = (x[6:0] == 7'b0);
        end
    endfunction

    function automatic logic is_subnormal(input logic [7:0] x);
        logic [4:0] e; logic [1:0] f;
        begin
            e = x[6:2]; f = x[1:0];
            is_subnormal = (e == 5'b0) && (f != 2'b00);
        end
    endfunction

    // Canonical qNaN: sign=0, exp=all1, frac=01
    function automatic logic [7:0] qnan();
        qnan = 8'b0_11111_01;
    endfunction

    // -------- helper: shift right with sticky (for EXT_W bits) --------
    function automatic logic [EXT_W-1:0] shr_sticky(
        input logic [EXT_W-1:0] in,
        input int unsigned shamt
    );
        logic [EXT_W-1:0] tmp;
        logic sticky;
        int i;
        begin
            if (shamt == 0) begin
                shr_sticky = in;
            end else if (shamt >= EXT_W) begin
                // everything shifts out -> becomes 0, sticky = OR(all in)
                sticky = |in;
                tmp = '0;
                tmp[0] = sticky; // keep sticky in LSB
                shr_sticky = tmp;
            end else begin
                sticky = 1'b0;
                // bits that are shifted out contribute to sticky
                for (i = 0; i < shamt; i++) begin
                    sticky |= in[i];
                end
                tmp = (in >> shamt);
                tmp[0] |= sticky;
                shr_sticky = tmp;
            end
        end
    endfunction

    // -------- helper: RNE rounding --------
    function automatic logic [SIG_W-1:0] round_rne_sig(
        input logic [SIG_W-1:0] sig,  // kept bits (hidden+frac)
        input logic guard,
        input logic round,
        input logic sticky
    );
        logic inc;
        begin
            // inc when guard=1 and (round|sticky|lsb(sig))=1
            inc = guard & (round | sticky | sig[0]);
            round_rne_sig = sig + inc;
        end
    endfunction

    // -------- main combinational --------
    always_comb begin
        // unpack
        logic sa, sb;
        logic [4:0] ea, eb;
        logic [1:0] fa, fb;

        sa = a[7]; ea = a[6:2]; fa = a[1:0];
        sb = b[7]; eb = b[6:2]; fb = b[1:0];

        // Special cases
        if (is_nan(a) || is_nan(b)) begin
            y = qnan();
        end else if (is_inf(a) || is_inf(b)) begin
            if (is_inf(a) && is_inf(b) && (sa != sb)) begin
                // +inf + -inf = NaN
                y = qnan();
            end else if (is_inf(a)) begin
                y = {sa, 5'h1F, 2'b00};
            end else begin
                y = {sb, 5'h1F, 2'b00};
            end
        end else if (is_zero(a) && is_zero(b)) begin
            y = {(sa & sb), 5'b0, 2'b0};
        end else if (is_zero(a)) begin
            y = b;
        end else if (is_zero(b)) begin
            y = a;
        end else begin
            // -------- build effective exponent (unbiased) and significand --------
            int eua, eub; // unbiased exponents
            logic [SIG_W-1:0] siga, sigb; // hidden+frac (3 bits)

            // For normals: hidden=1, exp_unbiased = exp - BIAS
            // For subnormals: hidden=0, exp_unbiased = 1 - BIAS (per IEEE style)
            if (ea == 0) begin
                eua  = 1 - BIAS;
                siga = {1'b0, fa}; // 0.xx
            end else begin
                eua  = int'(ea) - BIAS;
                siga = {1'b1, fa}; // 1.xx
            end

            if (eb == 0) begin
                eub  = 1 - BIAS;
                sigb = {1'b0, fb};
            end else begin
                eub  = int'(eb) - BIAS;
                sigb = {1'b1, fb};
            end

            // Extend significands with GRS zeros at bottom, plus headroom at top
            logic [EXT_W-1:0] ma_ext, mb_ext;
            ma_ext = '0;
            mb_ext = '0;

            // Place sig in upper part, leave 3 bits for GRS in [2:0]
            // Also leave 2 extra bits at top for carry
            ma_ext[GRS_W + SIG_W - 1 : GRS_W] = siga; // bits [5:3] when SIG_W=3, GRS_W=3
            mb_ext[GRS_W + SIG_W - 1 : GRS_W] = sigb;

            // -------- align exponents --------
            int eu_big, eu_small;
            logic s_big, s_small;
            logic [EXT_W-1:0] m_big, m_small;
            int diff;

            if (eua > eub) begin
                eu_big   = eua;   eu_small = eub;
                s_big    = sa;    s_small  = sb;
                m_big    = ma_ext;
                m_small  = mb_ext;
            end else if (eub > eua) begin
                eu_big   = eub;   eu_small = eua;
                s_big    = sb;    s_small  = sa;
                m_big    = mb_ext;
                m_small  = ma_ext;
            end else begin
                // equal exponents: pick magnitude to decide subtract sign later
                eu_big   = eua; eu_small = eub;
                if (ma_ext >= mb_ext) begin
                    s_big   = sa; s_small = sb;
                    m_big   = ma_ext; m_small = mb_ext;
                end else begin
                    s_big   = sb; s_small = sa;
                    m_big   = mb_ext; m_small = ma_ext;
                end
            end

            diff = eu_big - eu_small;
            m_small = shr_sticky(m_small, (diff < 0) ? 0 : diff);

            // -------- add/sub based on signs --------
            logic sign_res;
            logic [EXT_W:0] sum_ext; // one more bit for carry/borrow

            if (s_big == s_small) begin
                sum_ext  = {1'b0, m_big} + {1'b0, m_small};
                sign_res = s_big;
            end else begin
                // magnitude: we already ensured m_big >= m_small when exponents equal,
                // but if exponents differ, m_big is from larger exp; still could be smaller after alignment?
                // (In practice larger exp dominates; keep safe with compare)
                if (m_big >= m_small) begin
                    sum_ext  = {1'b0, m_big} - {1'b0, m_small};
                    sign_res = s_big;
                end else begin
                    sum_ext  = {1'b0, m_small} - {1'b0, m_big};
                    sign_res = s_small;
                end
            end

            // If result is exactly zero
            if (sum_ext == '0) begin
                y = 8'b0; // +0
            end else begin
                // -------- normalization --------
                int eu_res;
                logic [EXT_W:0] norm; // keep the extra bit
                norm = sum_ext;
                eu_res = eu_big;

                // Case 1: addition could overflow into higher bit -> shift right 1, eu++
                // Detect if the bit above current hidden position is 1.
                int hidden_idx;
                hidden_idx = GRS_W + SIG_W - 1; // e.g. 5

                if (norm[hidden_idx+1] == 1'b1) begin
                    // shift right 1 with sticky
                    logic [EXT_W-1:0] tmp;
                    tmp = norm[EXT_W-1:0];
                    tmp = shr_sticky(tmp, 1);
                    norm = {1'b0, tmp};
                    eu_res = eu_res + 1;
                end else begin
                    // Case 2: subtraction may need left shifts until hidden bit becomes 1,
                    // but if exponent gets too small, we'll handle as subnormal later.
                    while ((norm[hidden_idx] == 1'b0) && (eu_res > (1 - BIAS - 8))) begin
                        norm = norm << 1;
                        eu_res = eu_res - 1;
                    end
                end

                // -------- rounding + pack --------
                // Extract G/R/S and kept sig( hidden+frac ) from norm
                logic guard, rnd, sticky;
                logic [SIG_W-1:0] sig_keep;
                logic [SIG_W-1:0] sig_rounded;
                int biased;

                // For convenience, operate on lower EXT_W bits
                logic [EXT_W-1:0] norm_lo;
                norm_lo = norm[EXT_W-1:0];

                guard  = norm_lo[2];
                rnd    = norm_lo[1];
                sticky = norm_lo[0];
                sig_keep = norm_lo[GRS_W + SIG_W - 1 : GRS_W]; // 3 bits

                sig_rounded = round_rne_sig(sig_keep, guard, rnd, sticky);

                // Rounding can carry out and require renormalization (e.g., 1.11 -> 10.00)
                if (sig_rounded[SIG_W-1] == 1'b0) begin
                    // shouldn't happen for normal path; ignore
                end
                if (sig_rounded == (1 << SIG_W)) begin
                    // not possible with fixed width; kept for clarity
                end

                // Detect carry beyond hidden bit: do rounding in 4 bits:
                begin
                    logic [SIG_W:0] sig4;
                    logic inc;
                    inc = guard & (rnd | sticky | sig_keep[0]);
                    sig4 = {1'b0, sig_keep} + inc;
                    if (sig4[SIG_W] == 1'b1) begin
                        // shift right 1
                        eu_res = eu_res + 1;
                        sig_rounded = sig4[SIG_W:1]; // take top 3 bits
                    end else begin
                        sig_rounded = sig4[SIG_W-1:0];
                    end
                end

                biased = eu_res + BIAS;

                // Overflow -> Inf
                if (biased >= 31) begin
                    y = {sign_res, 5'h1F, 2'b00};
                end
                // Underflow -> subnormal/zero
                else if (biased <= 0) begin
                    // Create subnormal by shifting right by (1 - biased)
                    int sh;
                    logic [EXT_W-1:0] sub_ext;
                    logic [SIG_W-1:0] sub_sig_keep;
                    logic sub_g, sub_r, sub_s;
                    logic [SIG_W:0] sub_sig4;

                    // Rebuild an extended mantissa from sig_rounded with zero GRS, then shift
                    sub_ext = '0;
                    sub_ext[GRS_W + SIG_W - 1 : GRS_W] = sig_rounded;
                    sh = (1 - biased); // how much to shift to make exponent 0

                    sub_ext = shr_sticky(sub_ext, (sh < 0) ? 0 : sh);

                    sub_g = sub_ext[2];
                    sub_r = sub_ext[1];
                    sub_s = sub_ext[0];
                    sub_sig_keep = sub_ext[GRS_W + SIG_W - 1 : GRS_W];

                    // RNE again for subnormal
                    begin
                        logic inc2;
                        inc2 = sub_g & (sub_r | sub_s | sub_sig_keep[0]);
                        sub_sig4 = {1'b0, sub_sig_keep} + inc2;
                        // For subnormal, hidden bit is NOT stored; exponent field is 0.
                        if (sub_sig4[SIG_W] == 1'b1) begin
                            // Promote to smallest normal: exp=1, frac=00
                            y = {sign_res, 5'd1, 2'b00};
                        end else begin
                            // exp=0, frac=lower 2 bits (drop hidden)
                            logic [1:0] frac_sub;
                            frac_sub = sub_sig4[1:0];
                            if (frac_sub == 2'b00) begin
                                y = 8'b0; // underflow to zero
                            end else begin
                                y = {sign_res, 5'b0, frac_sub};
                            end
                        end
                    end
                end
                // Normal
                else begin
                    // Store frac bits (without hidden)
                    y = {sign_res, biased[4:0], sig_rounded[FRAC_W-1:0]};
                end
            end
        end
    end

endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"fp8_e5m2_adder");
    if(top_mod == nullptr) { return 2; }

    auto top = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top.mod == nullptr) { return 3; }

    // Smoke: special cases (avoid depending on full arithmetic correctness here).
    set_u8(top, "a", 0x00);
    set_u8(top, "b", 0x40);
    ::phy_engine::verilog::digital::simulate(top, 0);
    if(get_u8(top, "y") != 0x40) { return 4; }

    set_u8(top, "a", 0x7C);  // +inf
    set_u8(top, "b", 0xFC);  // -inf
    ::phy_engine::verilog::digital::simulate(top, 1);
    if(get_u8(top, "y") != 0x7D) { return 5; }  // qNaN

    set_u8(top, "a", 0x7D);  // NaN
    set_u8(top, "b", 0x00);
    ::phy_engine::verilog::digital::simulate(top, 2);
    if(get_u8(top, "y") != 0x7D) { return 6; }  // qNaN

    return 0;
}
