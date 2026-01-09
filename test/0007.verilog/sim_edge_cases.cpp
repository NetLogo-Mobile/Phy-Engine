#include <cstdint>

#include <phy_engine/verilog/digital/digital.h>

namespace
{
    using namespace ::phy_engine::verilog::digital;
    using u8sv = ::fast_io::u8string_view;

    inline ::std::size_t find_signal(compiled_module const& m, u8sv name)
    {
        auto it{m.signal_index.find(::fast_io::u8string{name})};
        if(it == m.signal_index.end()) { return SIZE_MAX; }
        return it->second;
    }

    inline vector_desc const* find_vec(compiled_module const& m, u8sv base)
    {
        auto it{m.vectors.find(::fast_io::u8string{base})};
        if(it == m.vectors.end()) { return nullptr; }
        return __builtin_addressof(it->second);
    }

    inline bool set_uN_vec_value(compiled_module const& m, instance_state& st, u8sv base, ::std::uint64_t value, ::std::size_t nbits)
    {
        auto const* vd{find_vec(m, base)};
        if(vd == nullptr || vd->bits.size() != nbits) { return false; }
        for(::std::size_t pos{}; pos < nbits; ++pos)
        {
            ::std::size_t const bit_from_lsb{nbits - 1 - pos};
            bool const b{bit_from_lsb < 64 ? (((value >> bit_from_lsb) & 1u) != 0u) : false};
            auto const sig{vd->bits.index_unchecked(pos)};
            if(sig >= st.state.values.size()) { return false; }
            st.state.values.index_unchecked(sig) = b ? logic_t::true_state : logic_t::false_state;
        }
        return true;
    }

    inline bool read_scalar(compiled_module const& m, instance_state const& st, u8sv name, logic_t& out)
    {
        auto const sig{find_signal(m, name)};
        if(sig == SIZE_MAX || sig >= st.state.values.size()) { return false; }
        out = st.state.values.index_unchecked(sig);
        return true;
    }

    inline bool read_vec_bit(compiled_module const& m, instance_state const& st, u8sv base, int idx, logic_t& out)
    {
        auto const* vd{find_vec(m, base)};
        if(vd == nullptr) { return false; }
        auto const pos{::phy_engine::verilog::digital::details::vector_pos(*vd, idx)};
        if(pos == SIZE_MAX || pos >= vd->bits.size()) { return false; }
        auto const sig{vd->bits.index_unchecked(pos)};
        if(sig >= st.state.values.size()) { return false; }
        out = st.state.values.index_unchecked(sig);
        return true;
    }
}  // namespace

int main()
{
    // Out-of-range selects should produce X in expressions (not Z).
    {
        decltype(auto) src = u8R"(
module oob(input [3:0] a, output y, output [1:0] z, output w);
  assign y = a[10];
  assign z = a[10:9];
  assign w = a[0] & 1'bz;
endmodule
)";

        ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
        auto cr{compile(sv)};
        if(!cr.errors.empty()) { ::std::fprintf(stderr, "oob: compile errors\n"); return 1; }
        auto d{build_design(::std::move(cr))};

        auto const* top{find_module(d, u8sv{u8"oob"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_uN_vec_value(*top, st, u8sv{u8"a"}, 0b1011u, 4)) { ::std::fprintf(stderr, "oob: set a failed\n"); return 1; }
        simulate(st, 1, false);

        logic_t y{};
        if(!read_scalar(*top, st, u8sv{u8"y"}, y)) { ::std::fprintf(stderr, "oob: read y failed\n"); return 1; }
        if(y != logic_t::indeterminate_state) { ::std::fprintf(stderr, "oob: y exp=X got=%d\n", static_cast<int>(y)); return 1; }

        logic_t z1{};
        logic_t z0{};
        if(!read_vec_bit(*top, st, u8sv{u8"z"}, 1, z1)) { ::std::fprintf(stderr, "oob: read z[1] failed\n"); return 1; }
        if(!read_vec_bit(*top, st, u8sv{u8"z"}, 0, z0)) { ::std::fprintf(stderr, "oob: read z[0] failed\n"); return 1; }
        if(z1 != logic_t::indeterminate_state || z0 != logic_t::indeterminate_state)
        {
            ::std::fprintf(stderr, "oob: z exp=XX got=%d%d\n", static_cast<int>(z1), static_cast<int>(z0));
            return 1;
        }

        logic_t w{};
        if(!read_scalar(*top, st, u8sv{u8"w"}, w)) { ::std::fprintf(stderr, "oob: read w failed\n"); return 1; }
        if(w != logic_t::indeterminate_state) { ::std::fprintf(stderr, "oob: w exp=X got=%d\n", static_cast<int>(w)); return 1; }
    }

    // Compile-time width limit for concat/replication should emit an error (but not crash).
    {
        decltype(auto) src = u8R"(
module big(output [4095:0] y);
  assign y = {4097{1'b0}};
endmodule
)";

        ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
        auto cr{compile(sv)};
        if(cr.errors.empty()) { ::std::fprintf(stderr, "big: expected compile error\n"); return 1; }
    }

    return 0;
}
