#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
using u8sv = ::fast_io::u8string_view;

::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}

std::string read_file_text(std::filesystem::path const& path)
{
    std::ifstream ifs(path, std::ios::binary);
    if(!ifs.is_open()) { throw std::runtime_error("failed to open: " + path.string()); }
    std::string s;
    ifs.seekg(0, std::ios::end);
    auto const n = static_cast<std::size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    s.resize(n);
    if(n != 0) { ifs.read(s.data(), static_cast<std::streamsize>(n)); }
    return s;
}

struct include_ctx
{
    std::filesystem::path base_dir;
};

bool include_resolver_fs(void* user, ::fast_io::u8string_view path, ::fast_io::u8string& out_text) noexcept
{
    try
    {
        auto* ctx = static_cast<include_ctx*>(user);
        std::string rel(reinterpret_cast<char const*>(path.data()), path.size());
        auto p = ctx->base_dir / rel;
        auto s = read_file_text(p);
        out_text.assign(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(s.data()), s.size()});
        return true;
    }
    catch(...)
    {
        return false;
    }
}

inline float fp16_to_float(std::uint16_t h) noexcept
{
    std::uint32_t const sign = (h >> 15) & 1u;
    std::uint32_t const exp = (h >> 10) & 0x1Fu;
    std::uint32_t const frac = h & 0x3FFu;

    if(exp == 0)
    {
        if(frac == 0) { return sign ? -0.0f : 0.0f; }
        // subnormal: (-1)^s * 2^(1-bias) * (frac / 2^10)
        float const m = static_cast<float>(frac) / 1024.0f;
        float const v = std::ldexp(m, -14);
        return sign ? -v : v;
    }
    if(exp == 31)
    {
        if(frac == 0) { return sign ? -std::numeric_limits<float>::infinity() : std::numeric_limits<float>::infinity(); }
        return std::numeric_limits<float>::quiet_NaN();
    }
    float const m = 1.0f + static_cast<float>(frac) / 1024.0f;
    float const v = std::ldexp(m, static_cast<int>(exp) - 15);
    return sign ? -v : v;
}

inline std::uint16_t float_to_fp16(float f) noexcept
{
    // IEEE754 float32 -> float16, round-to-nearest-even, canonicalize NaN to 0x7E00.
    union
    {
        float f;
        std::uint32_t u;
    } v{f};

    std::uint32_t sign = (v.u >> 31) & 1u;
    std::uint32_t exp = (v.u >> 23) & 0xFFu;
    std::uint32_t frac = v.u & 0x7FFFFFu;

    if(exp == 0xFFu)
    {
        if(frac == 0) { return static_cast<std::uint16_t>((sign << 15) | 0x7C00u); }
        return 0x7E00u;
    }

    // Normalize subnormal float32.
    if(exp == 0)
    {
        if(frac == 0) { return static_cast<std::uint16_t>(sign << 15); }
        // shift until leading 1
        while((frac & 0x800000u) == 0)
        {
            frac <<= 1;
            --exp;
        }
        frac &= 0x7FFFFFu;
        ++exp;
    }

    int const exp_unbiased = static_cast<int>(exp) - 127;
    int const exp16 = exp_unbiased + 15;

    if(exp16 >= 31)
    {
        return static_cast<std::uint16_t>((sign << 15) | 0x7C00u);
    }

    if(exp16 <= 0)
    {
        // subnormal/zero in fp16
        if(exp16 < -10) { return static_cast<std::uint16_t>(sign << 15); }
        // add implicit leading 1
        std::uint32_t mant = frac | 0x800000u;
        int const shift = 14 - exp16;  // 1..10
        std::uint32_t const mant_shifted = mant >> shift;
        std::uint32_t const rem = mant & ((1u << shift) - 1u);
        std::uint32_t const halfway = 1u << (shift - 1);
        std::uint32_t const lsb = mant_shifted & 1u;
        std::uint32_t rounded = mant_shifted;
        if(rem > halfway || (rem == halfway && lsb)) { ++rounded; }
        return static_cast<std::uint16_t>((sign << 15) | (rounded & 0x3FFu));
    }

    // normal fp16
    std::uint32_t mant = frac;
    std::uint32_t const mant16 = mant >> 13;          // top 10 bits
    std::uint32_t const rem = mant & 0x1FFFu;         // remaining 13 bits
    std::uint32_t const halfway = 0x1000u;            // 1 << 12
    std::uint32_t const lsb = mant16 & 1u;
    std::uint32_t rounded = mant16;
    if(rem > halfway || (rem == halfway && lsb)) { ++rounded; }
    if(rounded == 0x400u)
    {
        // mantissa overflow => increment exponent
        rounded = 0;
        if(exp16 + 1 >= 31) { return static_cast<std::uint16_t>((sign << 15) | 0x7C00u); }
        return static_cast<std::uint16_t>((sign << 15) | (static_cast<std::uint16_t>(exp16 + 1) << 10));
    }
    return static_cast<std::uint16_t>((sign << 15) | (static_cast<std::uint16_t>(exp16) << 10) | static_cast<std::uint16_t>(rounded));
}

inline std::uint16_t fp16_ref_op(std::uint16_t a, std::uint16_t b, std::uint8_t op) noexcept
{
    float fa = fp16_to_float(a);
    float fb = fp16_to_float(b);
    float fr{};
    switch(op)
    {
        case 0: fr = fa + fb; break;
        case 1: fr = fa - fb; break;
        case 2: fr = fa * fb; break;
        default: fr = fa / fb; break;
    }
    return float_to_fp16(fr);
}

inline bool set_uN_vec_value(::phy_engine::verilog::digital::compiled_module const& m,
                             ::phy_engine::verilog::digital::instance_state& st,
                             u8sv base,
                             ::std::uint64_t value,
                             ::std::size_t nbits)
{
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end()) { return false; }
    auto const& vd{it->second};
    if(vd.bits.size() != nbits) { return false; }
    for(::std::size_t pos{}; pos < nbits; ++pos)
    {
        ::std::size_t const bit_from_lsb{nbits - 1 - pos};
        bool const b{bit_from_lsb < 64 ? (((value >> bit_from_lsb) & 1u) != 0u) : false};
        auto const sig{vd.bits.index_unchecked(pos)};
        if(sig >= st.state.values.size()) { return false; }
        st.state.values.index_unchecked(sig) = b ? ::phy_engine::verilog::digital::logic_t::true_state
                                                 : ::phy_engine::verilog::digital::logic_t::false_state;
    }
    return true;
}

inline bool read_uN_vec_binary(::phy_engine::verilog::digital::compiled_module const& m,
                               ::phy_engine::verilog::digital::instance_state const& st,
                               u8sv base,
                               ::std::uint64_t& out,
                               ::std::size_t nbits)
{
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end()) { return false; }
    auto const& vd{it->second};
    if(vd.bits.size() != nbits) { return false; }

    std::uint64_t v{};
    for(::std::size_t pos{}; pos < nbits; ++pos)
    {
        auto const sig{vd.bits.index_unchecked(pos)};
        if(sig >= st.state.values.size()) { return false; }
        auto const lv{st.state.values.index_unchecked(sig)};
        if(lv != ::phy_engine::verilog::digital::logic_t::false_state && lv != ::phy_engine::verilog::digital::logic_t::true_state)
        {
            return false;
        }
        bool const bit{lv == ::phy_engine::verilog::digital::logic_t::true_state};
        ::std::size_t const bit_from_lsb{nbits - 1 - pos};
        if(bit) { v |= (1ull << bit_from_lsb); }
    }
    out = v;
    return true;
}

struct rng64
{
    std::uint64_t s{0x243f6a8885a308d3ULL};
    std::uint64_t next() noexcept
    {
        std::uint64_t x = s;
        x ^= x << 13;
        x ^= x >> 7;
        x ^= x << 17;
        s = x;
        return x;
    }
};

std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
{
    if(!s.starts_with(base)) { return std::nullopt; }
    if(s.size() < base.size() + 3) { return std::nullopt; }  // "a[0]"
    if(s[base.size()] != '[') { return std::nullopt; }
    if(s.back() != ']') { return std::nullopt; }
    auto inner = s.substr(base.size() + 1, s.size() - (base.size() + 2));
    if(inner.empty()) { return std::nullopt; }
    std::size_t v{};
    for(char ch : inner)
    {
        if(ch < '0' || ch > '9') { return std::nullopt; }
        v = v * 10 + static_cast<std::size_t>(ch - '0');
    }
    return v;
}
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    static constexpr std::size_t kA = 16;
    static constexpr std::size_t kB = 16;
    static constexpr std::size_t kOP = 2;
    static constexpr std::size_t kY = 16;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "fp16_fpu.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    ::phy_engine::verilog::digital::compile_options copt{};
    include_ctx ictx{.base_dir = src_path.parent_path()};
    copt.preprocess.user = __builtin_addressof(ictx);
    copt.preprocess.include_resolver = include_resolver_fs;
    auto cr = ::phy_engine::verilog::digital::compile(src, copt);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"fp16_fpu_top");
    if(top_mod == nullptr) { return 2; }

    // Verilog-module simulation.
    auto st_vs = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(st_vs.mod == nullptr) { return 3; }

    // PE synthesis + PE simulation.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    auto st_pe = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(st_pe.mod == nullptr) { return 4; }

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(st_pe.mod->ports.size());
    for(std::size_t i{}; i < st_pe.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::array<::phy_engine::model::model_base*, kA> in_a{};
    std::array<::phy_engine::model::model_base*, kB> in_b{};
    std::array<::phy_engine::model::model_base*, kOP> in_op{};
    std::array<std::size_t, kY> out_y{};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    in_op.fill(nullptr);
    out_y.fill(static_cast<std::size_t>(-1));

    for(std::size_t pi{}; pi < st_pe.mod->ports.size(); ++pi)
    {
        auto const& p = st_pe.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 5; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 6; }

            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kA) { in_a[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kB) { in_b[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "op"); idx && *idx < kOP) { in_op[*idx] = m; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 7; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 8; }
            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kY) { out_y[*idx] = pi; }
        }
        else
        {
            return 9;
        }
    }

    for(auto* m : in_a) { if(m == nullptr) { return 10; } }
    for(auto* m : in_b) { if(m == nullptr) { return 11; } }
    for(auto* m : in_op) { if(m == nullptr) { return 12; } }
    for(auto pi : out_y) { if(pi == static_cast<std::size_t>(-1)) { return 13; } }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, st_pe, ports, &err, opt)) { return 14; }
    if(!c.analyze()) { return 15; }

    auto pe_set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto pe_set_inputs = [&](std::uint16_t a, std::uint16_t b, std::uint8_t opv) {
        for(std::size_t i{}; i < kA; ++i) { pe_set_in(in_a[i], ((a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kB; ++i) { pe_set_in(in_b[i], ((b >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kOP; ++i) { pe_set_in(in_op[i], ((opv >> i) & 1u) != 0); }
    };

    auto pe_settle = [&]() noexcept { c.digital_clk(); };

    auto pe_read_y = [&]() -> std::optional<std::uint16_t> {
        std::uint16_t y{};
        for(std::size_t i{}; i < kY; ++i)
        {
            auto const s = ports[out_y[i]]->node_information.dn.state;
            if(s != ::phy_engine::model::digital_node_statement_t::false_state &&
               s != ::phy_engine::model::digital_node_statement_t::true_state)
            {
                return std::nullopt;
            }
            if(s == ::phy_engine::model::digital_node_statement_t::true_state) { y |= static_cast<std::uint16_t>(1u << i); }
        }
        return y;
    };

    struct vec
    {
        std::uint16_t a;
        std::uint16_t b;
        std::uint8_t op;
    };

    std::vector<vec> cases_verilog{};
    cases_verilog.reserve(4096);

    std::vector<vec> cases_cross{};
    cases_cross.reserve(512);

    constexpr std::array<std::pair<std::uint16_t, std::uint16_t>, 20> cross_pairs{
        std::pair<std::uint16_t, std::uint16_t>{0x0000u, 0x0000u},
        {0x0000u, 0x8000u},
        {0x8000u, 0x0000u},
        {0x3C00u, 0x3C00u},  // 1, 1
        {0xBC00u, 0x3C00u},  // -1, 1
        {0x3C00u, 0xBC00u},  // 1, -1
        {0x0400u, 0x0400u},  // min normal
        {0x0001u, 0x0001u},  // min subnormal
        {0x7BFFu, 0x0001u},  // max, min sub
        {0x0001u, 0x7BFFu},
        {0x7BFFu, 0x7BFFu},  // max, max
        {0xFBFFu, 0x7BFFu},  // -max, max
        {0x7C00u, 0x3C00u},  // +inf, 1
        {0x3C00u, 0x7C00u},  // 1, +inf
        {0xFC00u, 0x3C00u},  // -inf, 1
        {0x3C00u, 0xFC00u},  // 1, -inf
        {0x7E00u, 0x3C00u},  // NaN, 1
        {0x3C00u, 0x7E00u},  // 1, NaN
        {0x7C00u, 0x0000u},  // inf, 0
        {0x0000u, 0x7C00u},  // 0, inf
    };

    // Verilog-only reference check (keep moderately sized; module-level sim is relatively slow here).
    for(auto const& [a, b] : cross_pairs)
    {
        for(std::uint8_t opv{}; opv < 4; ++opv) { cases_verilog.push_back({a, b, opv}); }
    }

    rng64 rng{};
    for(std::size_t i{}; i < 64; ++i)
    {
        std::uint16_t a = static_cast<std::uint16_t>(rng.next() & 0xFFFFu);
        std::uint16_t b = static_cast<std::uint16_t>((rng.next() >> 16) & 0xFFFFu);
        std::uint8_t opv = static_cast<std::uint8_t>(rng.next() & 3u);
        cases_verilog.push_back({a, b, opv});
    }

    // Cross-check PE vs Verilog vs reference on a smaller, high-value sample (PE sim is slower).
    for(auto const& [a, b] : cross_pairs)
    {
        for(std::uint8_t opv{}; opv < 4; ++opv) { cases_cross.push_back({a, b, opv}); }
    }
    for(std::size_t i{}; i < 32; ++i)
    {
        std::uint16_t a = static_cast<std::uint16_t>(rng.next() & 0xFFFFu);
        std::uint16_t b = static_cast<std::uint16_t>((rng.next() >> 16) & 0xFFFFu);
        std::uint8_t opv = static_cast<std::uint8_t>(rng.next() & 3u);
        cases_cross.push_back({a, b, opv});
    }

    // 1) Verify verilog::digital simulation matches reference.
    for(std::size_t ci{}; ci < cases_verilog.size(); ++ci)
    {
        auto const t = cases_verilog[ci];
        auto const ref = fp16_ref_op(t.a, t.b, t.op);

        // Verilog simulator (verilog::digital).
        (void)set_uN_vec_value(*top_mod, st_vs, u8sv{u8"a"}, static_cast<std::uint64_t>(t.a ^ 0xFFFFu), kA);
        (void)set_uN_vec_value(*top_mod, st_vs, u8sv{u8"b"}, static_cast<std::uint64_t>(t.b ^ 0xFFFFu), kB);
        (void)set_uN_vec_value(*top_mod, st_vs, u8sv{u8"op"}, static_cast<std::uint64_t>(t.op ^ 0x3u), kOP);
        st_vs.state.prev_values = st_vs.state.values;
        st_vs.state.comb_prev_values = st_vs.state.values;

        if(!set_uN_vec_value(*top_mod, st_vs, u8sv{u8"a"}, static_cast<std::uint64_t>(t.a), kA)) { return 20; }
        if(!set_uN_vec_value(*top_mod, st_vs, u8sv{u8"b"}, static_cast<std::uint64_t>(t.b), kB)) { return 21; }
        if(!set_uN_vec_value(*top_mod, st_vs, u8sv{u8"op"}, static_cast<std::uint64_t>(t.op), kOP)) { return 22; }
        simulate(st_vs, 2, false);

        std::uint64_t y_vs{};
        bool const ok_vs = read_uN_vec_binary(*top_mod, st_vs, u8sv{u8"y"}, y_vs, kY);
        if(!ok_vs)
        {
            std::fprintf(stderr, "verilog sim produced non-binary y at case=%zu\n", ci);
            return 23;
        }

        std::uint16_t const yv = static_cast<std::uint16_t>(y_vs);
        if(yv != ref)
        {
            std::fprintf(stderr,
                         "verilog mismatch case=%zu a=0x%04x b=0x%04x op=%u ref=0x%04x verilog=0x%04x\n",
                         ci,
                         t.a,
                         t.b,
                         static_cast<unsigned>(t.op),
                         ref,
                         yv);
            return 25;
        }
    }

    // 2) Cross-check PE vs verilog::digital vs reference.
    for(std::size_t ci{}; ci < cases_cross.size(); ++ci)
    {
        auto const t = cases_cross[ci];
        auto const ref = fp16_ref_op(t.a, t.b, t.op);

        // Verilog simulator (verilog::digital).
        (void)set_uN_vec_value(*top_mod, st_vs, u8sv{u8"a"}, static_cast<std::uint64_t>(t.a ^ 0xFFFFu), kA);
        (void)set_uN_vec_value(*top_mod, st_vs, u8sv{u8"b"}, static_cast<std::uint64_t>(t.b ^ 0xFFFFu), kB);
        (void)set_uN_vec_value(*top_mod, st_vs, u8sv{u8"op"}, static_cast<std::uint64_t>(t.op ^ 0x3u), kOP);
        st_vs.state.prev_values = st_vs.state.values;
        st_vs.state.comb_prev_values = st_vs.state.values;

        if(!set_uN_vec_value(*top_mod, st_vs, u8sv{u8"a"}, static_cast<std::uint64_t>(t.a), kA)) { return 30; }
        if(!set_uN_vec_value(*top_mod, st_vs, u8sv{u8"b"}, static_cast<std::uint64_t>(t.b), kB)) { return 31; }
        if(!set_uN_vec_value(*top_mod, st_vs, u8sv{u8"op"}, static_cast<std::uint64_t>(t.op), kOP)) { return 32; }
        simulate(st_vs, 2, false);

        std::uint64_t y_vs{};
        bool const ok_vs = read_uN_vec_binary(*top_mod, st_vs, u8sv{u8"y"}, y_vs, kY);
        if(!ok_vs)
        {
            std::fprintf(stderr, "verilog sim produced non-binary y at cross case=%zu\n", ci);
            return 33;
        }
        std::uint16_t const yv = static_cast<std::uint16_t>(y_vs);

        // PE sim.
        pe_set_inputs(t.a, t.b, t.op);
        pe_settle();
        auto y_pe = pe_read_y();
        if(!y_pe)
        {
            std::fprintf(stderr, "pe sim produced non-binary y at cross case=%zu\n", ci);
            return 34;
        }

        if(yv != ref || *y_pe != ref || *y_pe != yv)
        {
            std::fprintf(stderr,
                         "cross mismatch case=%zu a=0x%04x b=0x%04x op=%u ref=0x%04x verilog=0x%04x pe=0x%04x\n",
                         ci,
                         t.a,
                         t.b,
                         static_cast<unsigned>(t.op),
                         ref,
                         yv,
                         *y_pe);
            return 35;
        }
    }

    return 0;
}
