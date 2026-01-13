#include <phy_engine/verilog/digital/digital.h>

#include <cstdint>

namespace
{
using namespace ::phy_engine::verilog::digital;
using u8sv = ::fast_io::u8string_view;

inline void commit_prev(instance_state& st) { st.state.prev_values = st.state.values; st.state.comb_prev_values = st.state.values; }

inline bool set_uN_vec(compiled_module const& m, instance_state& st, u8sv base, ::std::uint64_t value, ::std::size_t nbits)
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
module top(
  input  [14:0] in15,
  input  [5:0]  diff,
  output reg [14:0] out_shr1,
  output reg [14:0] out_shl1,
  output reg [14:0] out_loop
);
  reg [14:0] tmp;
  reg [5:0]  i;

  always @* begin
    tmp = in15;
    tmp = (tmp >> 1);
    out_shr1 = tmp;

    tmp = in15;
    tmp = (tmp << 1);
    out_shl1 = tmp;

    tmp = in15;
    for(i = 0; i < 30; i = i + 1) begin
      if(i < diff) begin
        tmp = (tmp >> 1);
      end
    end
    out_loop = tmp;
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
    if(st.mod == nullptr) { return 3; }

    if(!set_uN_vec(*m, st, u8sv{u8"in15"}, 0u, 15)) { return 4; }
    if(!set_uN_vec(*m, st, u8sv{u8"diff"}, 0u, 6)) { return 5; }
    commit_prev(st);

    if(!set_uN_vec(*m, st, u8sv{u8"in15"}, 0x3000u, 15)) { return 6; }
    if(!set_uN_vec(*m, st, u8sv{u8"diff"}, 2u, 6)) { return 7; }
    simulate(st, 2, false);

    ::std::uint64_t out_shr1{};
    ::std::uint64_t out_shl1{};
    ::std::uint64_t out_loop{};
    if(!read_uN_vec(*m, st, u8sv{u8"out_shr1"}, out_shr1, 15)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"out_shl1"}, out_shl1, 15)) { return 9; }
    if(!read_uN_vec(*m, st, u8sv{u8"out_loop"}, out_loop, 15)) { return 10; }

    // 0x3000 >> 1 = 0x1800; 0x3000 << 1 = 0x6000; loop diff=2 -> shift twice => 0x0C00.
    if(out_shr1 != 0x1800u) { return 11; }
    if(out_shl1 != 0x6000u) { return 12; }
    if(out_loop != 0x0C00u) { return 13; }
    return 0;
}

