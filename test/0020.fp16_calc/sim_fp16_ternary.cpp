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

inline bool read_uN_vec(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint64_t& out, ::std::size_t nbits)
{
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
        if(bit != logic_t::false_state && bit != logic_t::true_state) { return false; }
        ::std::size_t const bit_from_lsb{nbits - 1 - pos};
        if(bit == logic_t::true_state && bit_from_lsb < 64) { v |= (1ull << bit_from_lsb); }
    }
    out = v;
    return true;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module top(input [4:0] e, output reg [5:0] adj);
  always @* begin
    adj = (e == 0) ? 6'd1 : {1'b0, e};
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

    // e=15 => adj=15
    if(!set_uN_vec_value(*m, st, u8sv{u8"e"}, 0u, 5)) { return 3; }
    commit_prev(st);
    if(!set_uN_vec_value(*m, st, u8sv{u8"e"}, 15u, 5)) { return 4; }
    simulate(st, 2, false);

    std::uint64_t adj{};
    if(!read_uN_vec(*m, st, u8sv{u8"adj"}, adj, 6)) { return 5; }

    std::fprintf(stderr, "adj=%llu\n", static_cast<unsigned long long>(adj));
    return (adj == 15u) ? 0 : 6;
}

