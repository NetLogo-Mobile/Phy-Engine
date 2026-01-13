#include <phy_engine/verilog/digital/digital.h>

#include <cstdint>
#include <cstdio>

namespace
{
using namespace ::phy_engine::verilog::digital;
using u8sv = ::fast_io::u8string_view;

inline void commit_prev(instance_state& st) { st.state.prev_values = st.state.values; st.state.comb_prev_values = st.state.values; }

inline ::std::size_t find_signal(compiled_module const& m, u8sv name)
{
    auto it{m.signal_index.find(::fast_io::u8string{name})};
    if(it == m.signal_index.end()) { return SIZE_MAX; }
    return it->second;
}

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
module top(
  input  [15:0] a,
  output [4:0]  e_assign,
  output reg [4:0] e_reg,
  output [9:0]  f_assign,
  output reg [9:0] f_reg,
  output reg    msb_reg
);
  assign e_assign = a[14:10];
  assign f_assign = a[9:0];
  always @* begin
    e_reg = a[14:10];
    f_reg = a[9:0];
    msb_reg = a[15];
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

    // a=0x3E00 => sign=0, exp=0x0F, frac=0x200
    if(!set_uN_vec_value(*m, st, u8sv{u8"a"}, 0x0000u, 16)) { return 3; }
    commit_prev(st);
    if(!set_uN_vec_value(*m, st, u8sv{u8"a"}, 0x3E00u, 16)) { return 4; }
    simulate(st, 5, false);

    std::uint64_t e0{}, e1{}, f0{}, f1{};
    if(!read_uN_vec(*m, st, u8sv{u8"e_assign"}, e0, 5)) { return 5; }
    if(!read_uN_vec(*m, st, u8sv{u8"e_reg"}, e1, 5)) { return 6; }
    if(!read_uN_vec(*m, st, u8sv{u8"f_assign"}, f0, 10)) { return 7; }
    if(!read_uN_vec(*m, st, u8sv{u8"f_reg"}, f1, 10)) { return 8; }

    auto const msb_sig = find_signal(*m, u8sv{u8"msb_reg"});
    if(msb_sig == SIZE_MAX) { return 9; }
    auto const msb = st.state.values.index_unchecked(msb_sig);
    if(msb != logic_t::false_state && msb != logic_t::true_state) { return 10; }

    std::fprintf(stderr,
                 "e_assign=%llu e_reg=%llu f_assign=%llu f_reg=%llu msb=%u\n",
                 static_cast<unsigned long long>(e0),
                 static_cast<unsigned long long>(e1),
                 static_cast<unsigned long long>(f0),
                 static_cast<unsigned long long>(f1),
                 msb == logic_t::true_state ? 1u : 0u);

    if(static_cast<std::uint8_t>(e0) != 0x0F || static_cast<std::uint8_t>(e1) != 0x0F) { return 11; }
    if(static_cast<std::uint16_t>(f0) != 0x200 || static_cast<std::uint16_t>(f1) != 0x200) { return 12; }
    if(msb != logic_t::false_state) { return 13; }

    return 0;
}

