#include <phy_engine/verilog/digital/digital.h>

#include <cstdint>
#include <cstdio>

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

inline bool read_uN_vec(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint64_t& out, ::std::size_t nbits)
{
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end())
    {
        std::fprintf(stderr, "missing vector %.*s\n", static_cast<int>(base.size()), reinterpret_cast<char const*>(base.data()));
        return false;
    }
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
            std::fprintf(stderr, "non-binary at pos=%zu\n", pos);
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
    decltype(auto) src = u8R"(
module top(input trig, output reg [14:0] out);
  reg [14:0] tmp;
  reg [5:0] diff;
  reg sticky;
  reg [5:0] i;
  always @* begin
    if(trig) begin end
    tmp = 15'h3000;
    diff = 6'd1;
    sticky = 1'b0;
    for(i = 0; i < 30; i = i + 1) begin
      if(i < diff) begin
        sticky = sticky | tmp[0];
        tmp = (tmp >> 1);
        tmp[0] = tmp[0] | sticky;
      end
    end
    out = tmp;
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

    // Trigger combinational block.
    auto const trig_sig = find_signal(*m, u8sv{u8"trig"});
    if(trig_sig == SIZE_MAX || trig_sig >= st.state.values.size()) { return 20; }
    st.state.values.index_unchecked(trig_sig) = logic_t::false_state;
    st.state.prev_values = st.state.values;
    st.state.comb_prev_values = st.state.values;
    st.state.values.index_unchecked(trig_sig) = logic_t::true_state;
    simulate(st, 2, false);

    std::uint64_t out{};
    if(!read_uN_vec(*m, st, u8sv{u8"out"}, out, 15)) { return 3; }
    std::fprintf(stderr, "out=0x%04llx\n", static_cast<unsigned long long>(out));
    return (out == 0x1800u) ? 0 : 4;
}
