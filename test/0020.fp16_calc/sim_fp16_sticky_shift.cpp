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
module top(input [14:0] in15, input [5:0] diff, output reg [14:0] out15);
  reg [14:0] tmp;
  reg sticky;
  reg [5:0] i;
  always @* begin
    tmp = in15;
    sticky = 1'b0;
    for(i = 0; i < 30; i = i + 1) begin
      if(i < diff) begin
        sticky = sticky | tmp[0];
        tmp = (tmp >> 1);
        tmp[0] = tmp[0] | sticky;
      end
    end
    out15 = tmp;
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

    if(!set_uN_vec_value(*m, st, u8sv{u8"in15"}, 0u, 15)) { return 3; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"diff"}, 0u, 6)) { return 4; }
    commit_prev(st);
    if(!set_uN_vec_value(*m, st, u8sv{u8"in15"}, 0x3000u, 15)) { return 5; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"diff"}, 1u, 6)) { return 6; }

    simulate(st, 2, false);

    std::uint64_t out{};
    if(!read_uN_vec(*m, st, u8sv{u8"out15"}, out, 15)) { return 7; }
    std::fprintf(stderr, "out15=0x%04llx\n", static_cast<unsigned long long>(out));
    return (out == 0x1800u) ? 0 : 8;
}

