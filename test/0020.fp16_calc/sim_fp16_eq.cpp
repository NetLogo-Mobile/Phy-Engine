#include <phy_engine/verilog/digital/digital.h>

#include <cstdint>
#include <cstdio>

namespace
{
using namespace ::phy_engine::verilog::digital;
using u8sv = ::fast_io::u8string_view;

inline void commit_prev(instance_state& st) { st.state.prev_values = st.state.values; st.state.comb_prev_values = st.state.values; }

inline bool set_uN_vec_value(compiled_module const& m, instance_state& st, u8sv base, ::std::uint64_t value, ::std::size_t nbits)
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
        st.state.values.index_unchecked(sig) = b ? logic_t::true_state : logic_t::false_state;
    }
    return true;
}

inline ::std::size_t find_signal(compiled_module const& m, u8sv name)
{
    auto it{m.signal_index.find(::fast_io::u8string{name})};
    if(it == m.signal_index.end()) { return SIZE_MAX; }
    return it->second;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module top(input [15:0] a, output reg y);
  reg [4:0] e;
  always @* begin
    e = a[14:10];
    if(e == 0) y = 1'b1;
    else y = 1'b0;
  end
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
    auto cr{compile(sv)};
    if(!cr.errors.empty()) { return 1; }
    auto d{build_design(::std::move(cr))};
    auto const* m{find_module(d, u8sv{u8"top"})};
    if(m == nullptr) { return 2; }
    auto st{elaborate(d, *m)};

    if(!set_uN_vec_value(*m, st, u8sv{u8"a"}, 0x0000u, 16)) { return 3; }
    commit_prev(st);
    if(!set_uN_vec_value(*m, st, u8sv{u8"a"}, 0x3E00u, 16)) { return 4; }
    simulate(st, 2, false);

    auto const sig = find_signal(*m, u8sv{u8"y"});
    if(sig == SIZE_MAX) { return 7; }
    auto const v = st.state.values.index_unchecked(sig);
    if(v != logic_t::false_state && v != logic_t::true_state) { return 5; }
    std::fprintf(stderr, "y=%u\n", v == logic_t::true_state ? 1u : 0u);
    return (v == logic_t::false_state) ? 0 : 6;
}
