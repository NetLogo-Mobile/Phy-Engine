#include <phy_engine/verilog/digital/digital.h>

#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace
{
using namespace ::phy_engine::verilog::digital;
using u8sv = ::fast_io::u8string_view;

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

inline void commit_prev(instance_state& st) { st.state.prev_values = st.state.values; st.state.comb_prev_values = st.state.values; }

inline ::std::size_t find_signal(compiled_module const& m, u8sv name)
{
    auto it{m.signal_index.find(::fast_io::u8string{name})};
    if(it == m.signal_index.end()) { return SIZE_MAX; }
    return it->second;
}

inline bool set_scalar_value(compiled_module const& m, instance_state& st, u8sv name, logic_t v)
{
    auto const sig{find_signal(m, name)};
    if(sig == SIZE_MAX || sig >= st.state.values.size()) { return false; }
    st.state.values.index_unchecked(sig) = v;
    return true;
}

inline bool read_scalar(compiled_module const& m, instance_state const& st, u8sv name, logic_t& out)
{
    auto const sig{find_signal(m, name)};
    if(sig == SIZE_MAX || sig >= st.state.values.size()) { return false; }
    out = st.state.values.index_unchecked(sig);
    return true;
}

inline bool read_uN_vec(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint64_t& out, ::std::size_t nbits);

inline bool read_uN_vec_any(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint64_t& out, ::std::size_t nbits)
{
    if(nbits == 1)
    {
        logic_t v{};
        if(!read_scalar(m, st, base, v)) { return false; }
        out = (v == logic_t::true_state) ? 1u : 0u;
        return true;
    }
    return read_uN_vec(m, st, base, out, nbits);
}

inline bool set_uN_vec_value(compiled_module const& m, instance_state& st, u8sv base, ::std::uint64_t value, ::std::size_t nbits)
{
    if(nbits == 1)
    {
        return set_scalar_value(m, st, base, (value & 1u) ? logic_t::true_state : logic_t::false_state);
    }
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
        st.state.values.index_unchecked(sig) = b ? logic_t::true_state : logic_t::false_state;
    }
    return true;
}

inline bool read_uN_vec(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint64_t& out, ::std::size_t nbits)
{
    if(nbits == 1)
    {
        logic_t v{};
        if(!read_scalar(m, st, base, v)) { return false; }
        if(v != logic_t::false_state && v != logic_t::true_state) { return false; }
        out = (v == logic_t::true_state) ? 1u : 0u;
        return true;
    }
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end()) { return false; }
    auto const& vd{it->second};
    if(vd.bits.size() != nbits) { return false; }

    ::std::uint64_t v{};
    for(::std::size_t pos{}; pos < nbits; ++pos)
    {
        auto const sig{vd.bits.index_unchecked(pos)};
        if(sig >= st.state.values.size()) { return false; }
        auto const bit{st.state.values.index_unchecked(sig)};
        if(bit != logic_t::false_state && bit != logic_t::true_state)
        {
            std::fprintf(stderr, "read_uN_vec non-binary base=%.*s pos=%zu\n", static_cast<int>(base.size()), reinterpret_cast<char const*>(base.data()), pos);
            return false;
        }
        ::std::size_t const bit_from_lsb{nbits - 1 - pos};
        if(bit == logic_t::true_state && bit_from_lsb < 64) { v |= (1ull << bit_from_lsb); }
    }
    out = v;
    return true;
}
}  // namespace

int main()
{
    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "fp16_div.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    auto cr = compile(src);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            std::fprintf(stderr,
                         "compile error line=%u col=%u: %.*s\n",
                         static_cast<unsigned>(e.line),
                         static_cast<unsigned>(e.column),
                         static_cast<int>(e.message.size()),
                         reinterpret_cast<char const*>(e.message.data()));
        }
        return 1;
    }
    auto d = build_design(::std::move(cr));

    auto const* mod = find_module(d, u8sv{u8"fp16_div_unit"});
    if(mod == nullptr) { return 2; }
    auto st = elaborate(d, *mod);
    if(st.mod == nullptr) { return 3; }

    struct tc
    {
        std::uint16_t a;
        std::uint16_t b;
        std::uint16_t exp;
    };
    constexpr tc cases[] = {
        {0x4200u, 0x4000u, 0x3E00u},  // 3.0 / 2.0 = 1.5
        {0xC200u, 0x4000u, 0xBE00u},  // -3.0 / 2.0 = -1.5
    };

    for(auto const& t : cases)
    {
        if(!set_uN_vec_value(*mod, st, u8sv{u8"a"}, t.a ^ 0xFFFFu, 16)) { return 4; }
        if(!set_uN_vec_value(*mod, st, u8sv{u8"b"}, t.b ^ 0xFFFFu, 16)) { return 5; }
        commit_prev(st);

        if(!set_uN_vec_value(*mod, st, u8sv{u8"a"}, t.a, 16)) { return 6; }
        if(!set_uN_vec_value(*mod, st, u8sv{u8"b"}, t.b, 16)) { return 7; }
        simulate(st, 5, false);

        std::uint64_t y{};
        if(!read_uN_vec(*mod, st, u8sv{u8"y"}, y, 16)) { return 8; }
        auto const got = static_cast<std::uint16_t>(y);
        if(got != t.exp)
        {
            std::uint64_t exp_a_f{}, exp_b_f{}, frac_a{}, frac_b{}, exp_a_adj{}, exp_b_adj{}, exp_res{};
            std::uint64_t mant_a{}, mant_b{}, num{}, q{}, rem{}, mant_ext{};
            std::uint64_t zero_a{}, zero_b{}, nan_a{}, nan_b{}, inf_a{}, inf_b{};
            (void)read_uN_vec_any(*mod, st, u8sv{u8"exp_a_f"}, exp_a_f, 5);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"exp_b_f"}, exp_b_f, 5);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"frac_a"}, frac_a, 10);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"frac_b"}, frac_b, 10);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"exp_a_adj"}, exp_a_adj, 6);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"exp_b_adj"}, exp_b_adj, 6);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"exp_res"}, exp_res, 7);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"mant_a"}, mant_a, 11);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"mant_b"}, mant_b, 11);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"num"}, num, 24);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"q"}, q, 15);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"rem"}, rem, 11);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"mant_ext"}, mant_ext, 15);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"zero_a"}, zero_a, 1);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"zero_b"}, zero_b, 1);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"nan_a"}, nan_a, 1);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"nan_b"}, nan_b, 1);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"inf_a"}, inf_a, 1);
            (void)read_uN_vec_any(*mod, st, u8sv{u8"inf_b"}, inf_b, 1);

            std::fprintf(stderr,
                         "div mismatch a=0x%04x b=0x%04x got=0x%04x exp=0x%04x exp_a_f=%llu exp_b_f=%llu frac_a=0x%03llx frac_b=0x%03llx zero_a=%llu zero_b=%llu nan_a=%llu nan_b=%llu inf_a=%llu inf_b=%llu exp_a_adj=%llu exp_b_adj=%llu exp_res=%llu mant_a=0x%03llx mant_b=0x%03llx num=0x%06llx q=0x%04llx rem=0x%03llx mant_ext=0x%04llx\n",
                         t.a,
                         t.b,
                         got,
                         t.exp,
                         exp_a_f,
                         exp_b_f,
                         frac_a,
                         frac_b,
                         zero_a,
                         zero_b,
                         nan_a,
                         nan_b,
                         inf_a,
                         inf_b,
                         exp_a_adj,
                         exp_b_adj,
                         exp_res,
                         mant_a,
                         mant_b,
                         num,
                         q,
                         rem,
                         mant_ext);
            return 9;
        }
    }

    return 0;
}
