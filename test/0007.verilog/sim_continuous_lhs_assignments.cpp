#include <array>
#include <cstdint>
#include <cstdio>

#include <phy_engine/verilog/digital/digital.h>

namespace
{
    using namespace ::phy_engine::verilog::digital;

    using u8sv = ::fast_io::u8string_view;

    inline bool set_scalar(compiled_module const& m, instance_state& st, u8sv name, logic_t v)
    {
        auto it{m.signal_index.find(::fast_io::u8string{name})};
        if(it == m.signal_index.end()) { return false; }
        auto const sig{it->second};
        if(sig >= st.state.values.size()) { return false; }
        st.state.values.index_unchecked(sig) = v;
        return true;
    }

    inline vector_desc const* find_vec(compiled_module const& m, u8sv base)
    {
        auto it{m.vectors.find(::fast_io::u8string{base})};
        if(it == m.vectors.end()) { return nullptr; }
        return __builtin_addressof(it->second);
    }

    inline bool set_uN_vec(compiled_module const& m, instance_state& st, u8sv base, ::std::uint64_t value, ::std::size_t nbits)
    {
        auto const* vd{find_vec(m, base)};
        if(vd == nullptr || vd->bits.size() != nbits) { return false; }
        for(::std::size_t pos{}; pos < nbits; ++pos)
        {
            ::std::size_t const bit_from_lsb{nbits - 1 - pos};
            bool const bit{bit_from_lsb < 64 ? (((value >> bit_from_lsb) & 1u) != 0u) : false};
            auto const sig{vd->bits.index_unchecked(pos)};
            if(sig >= st.state.values.size()) { return false; }
            st.state.values.index_unchecked(sig) = bit ? logic_t::true_state : logic_t::false_state;
        }
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

    inline bool read_scalar(compiled_module const& m, instance_state const& st, u8sv name, logic_t& out)
    {
        auto it{m.signal_index.find(::fast_io::u8string{name})};
        if(it == m.signal_index.end()) { return false; }
        auto const sig{it->second};
        if(sig >= st.state.values.size()) { return false; }
        out = st.state.values.index_unchecked(sig);
        return true;
    }
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module dyn_lhs_bit(
  input a,
  input [1:0] idx,
  output [3:0] y
);
  assign y = 4'bzzzz;
  assign y[idx] = a;
endmodule

module dyn_lhs_ps(
  input [1:0] a,
  input [2:0] idx,
  output [3:0] y
);
  assign y = 4'bzzzz;
  assign y[idx+1:idx] = a;
endmodule

module md(
  input a,
  input b,
  output y
);
  assign y = a;
  assign y = b;
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
    auto cr{compile(sv)};
    if(!cr.errors.empty()) { return 1; }
    auto d{build_design(::std::move(cr))};

    // Continuous assign: dynamic LHS bit-select
    {
        auto const* top{find_module(d, u8sv{u8"dyn_lhs_bit"})};
        if(top == nullptr) { ::std::fprintf(stderr, "dyn_lhs_bit: missing module\n"); return 1; }
        auto st{elaborate(d, *top)};

        if(!set_scalar(*top, st, u8sv{u8"a"}, logic_t::true_state)) { ::std::fprintf(stderr, "dyn_lhs_bit: set a failed\n"); return 1; }

        for(::std::uint64_t idx{}; idx < 4; ++idx)
        {
            if(!set_uN_vec(*top, st, u8sv{u8"idx"}, idx, 2)) { ::std::fprintf(stderr, "dyn_lhs_bit: set idx failed\n"); return 1; }
            simulate(st, 1, false);

            for(int bit{}; bit < 4; ++bit)
            {
                logic_t v{};
                if(!read_vec_bit(*top, st, u8sv{u8"y"}, bit, v)) { ::std::fprintf(stderr, "dyn_lhs_bit: read y[%d] failed\n", bit); return 1; }
                if(static_cast<::std::uint64_t>(bit) == idx)
                {
                    if(v != logic_t::true_state)
                    {
                        ::std::fprintf(stderr, "dyn_lhs_bit: idx=%llu y[%d]=%d exp=1\n",
                                      static_cast<unsigned long long>(idx),
                                      bit,
                                      static_cast<int>(v));
                        return 1;
                    }
                }
                else
                {
                    if(v != logic_t::high_impedence_state)
                    {
                        ::std::fprintf(stderr, "dyn_lhs_bit: idx=%llu y[%d]=%d exp=z\n",
                                      static_cast<unsigned long long>(idx),
                                      bit,
                                      static_cast<int>(v));
                        return 1;
                    }
                }
            }
        }
    }

    // Continuous assign: dynamic part-select [idx+1:idx]
    {
        auto const* top{find_module(d, u8sv{u8"dyn_lhs_ps"})};
        if(top == nullptr) { ::std::fprintf(stderr, "dyn_lhs_ps: missing module\n"); return 1; }
        auto st{elaborate(d, *top)};

        // a = 2'b10
        if(!set_uN_vec(*top, st, u8sv{u8"a"}, 2, 2)) { ::std::fprintf(stderr, "dyn_lhs_ps: set a failed\n"); return 1; }

        // idx = 0..4 (idx>=3 => out-of-range => all Z)
        for(::std::uint64_t idx{}; idx < 5; ++idx)
        {
            if(!set_uN_vec(*top, st, u8sv{u8"idx"}, idx, 3)) { ::std::fprintf(stderr, "dyn_lhs_ps: set idx failed\n"); return 1; }
            simulate(st, 1, false);

            // Expected:
            // - idx=0 => y[1:0]=10
            // - idx=1 => y[2:1]=10
            // - idx=2 => y[3:2]=10
            // - idx>=3 => out-of-range => all Z
            for(int bit{}; bit < 4; ++bit)
            {
                logic_t v{};
                if(!read_vec_bit(*top, st, u8sv{u8"y"}, bit, v)) { ::std::fprintf(stderr, "dyn_lhs_ps: read y[%d] failed\n", bit); return 1; }

                if(idx <= 2 && bit >= static_cast<int>(idx) && bit <= static_cast<int>(idx + 1))
                {
                    bool const is_msb{bit == static_cast<int>(idx + 1)};
                    auto const exp{is_msb ? logic_t::true_state : logic_t::false_state};
                    if(v != exp)
                    {
                        ::std::fprintf(stderr, "dyn_lhs_ps: idx=%llu y[%d]=%d exp=%d\n",
                                      static_cast<unsigned long long>(idx),
                                      bit,
                                      static_cast<int>(v),
                                      static_cast<int>(exp));
                        return 1;
                    }
                }
                else
                {
                    if(v != logic_t::high_impedence_state)
                    {
                        ::std::fprintf(stderr, "dyn_lhs_ps: idx=%llu y[%d]=%d exp=z\n",
                                      static_cast<unsigned long long>(idx),
                                      bit,
                                      static_cast<int>(v));
                        return 1;
                    }
                }
            }
        }
    }

    // Multiple drivers: net resolution (0/1 conflict => X; Z ignored)
    {
        auto const* top{find_module(d, u8sv{u8"md"})};
        if(top == nullptr) { ::std::fprintf(stderr, "md: missing module\n"); return 1; }
        auto st{elaborate(d, *top)};

        auto check = [&](logic_t a, logic_t b, logic_t exp) -> bool
        {
            if(!set_scalar(*top, st, u8sv{u8"a"}, a)) { return false; }
            if(!set_scalar(*top, st, u8sv{u8"b"}, b)) { return false; }
            simulate(st, 1, false);

            logic_t y{};
            if(!read_scalar(*top, st, u8sv{u8"y"}, y)) { return false; }
            return y == exp;
        };

        if(!check(logic_t::false_state, logic_t::false_state, logic_t::false_state))
        {
            ::std::fprintf(stderr, "md: 0/0 => 0 failed\n");
            return 1;
        }
        if(!check(logic_t::true_state, logic_t::true_state, logic_t::true_state))
        {
            ::std::fprintf(stderr, "md: 1/1 => 1 failed\n");
            return 1;
        }
        if(!check(logic_t::false_state, logic_t::true_state, logic_t::indeterminate_state))
        {
            ::std::fprintf(stderr, "md: 0/1 => x failed\n");
            return 1;
        }
        if(!check(logic_t::true_state, logic_t::high_impedence_state, logic_t::true_state))
        {
            ::std::fprintf(stderr, "md: 1/z => 1 failed\n");
            return 1;
        }
    }

    return 0;
}
