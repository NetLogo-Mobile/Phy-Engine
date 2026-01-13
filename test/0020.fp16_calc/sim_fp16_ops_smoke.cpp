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
module top(input [14:0] in15,
           input [5:0]  sh,
           input [10:0] ma,
           input [10:0] mb,
           output [14:0] shr_c,
           output reg [14:0] shr_a,
           output [14:0] shr_dyn_c,
           output reg [14:0] shr_dyn_a,
           output [15:0] mul_c,
           output reg [15:0] mul_a,
           output [21:0] mul_dyn_c,
           output reg [21:0] mul_dyn_a,
           output [11:0] add_dyn_c,
           output reg [11:0] add_dyn_a,
           output [11:0] add_const_c,
           output [11:0] add_vc_c,
           output [11:0] add_cv_c,
           output [11:0] sub_dyn_c,
           output reg [11:0] sub_dyn_a);
  assign shr_c = in15 >> 1;
  assign shr_dyn_c = in15 >> sh;
  assign mul_c = 8'd3 * 8'd2;
  assign mul_dyn_c = ma * mb;
  assign add_dyn_c = ma + mb;
  assign add_const_c = 12'd1536 + 12'd1024;
  assign add_vc_c = ma + 12'd1;
  assign add_cv_c = 12'd1 + mb;
  assign sub_dyn_c = ma - mb;
  always @* begin
    shr_a = in15 >> 1;
    shr_dyn_a = in15 >> sh;
    mul_a = 8'd3 * 8'd2;
    mul_dyn_a = ma * mb;
    add_dyn_a = ma + mb;
    sub_dyn_a = ma - mb;
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

    if(!set_uN_vec_value(*m, st, u8sv{u8"in15"}, 0x0000u, 15)) { return 3; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"sh"}, 0u, 6)) { return 3; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"ma"}, 0u, 11)) { return 3; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"mb"}, 0u, 11)) { return 3; }
    commit_prev(st);
    if(!set_uN_vec_value(*m, st, u8sv{u8"in15"}, 0x3000u, 15)) { return 4; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"sh"}, 1u, 6)) { return 4; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"ma"}, 0x600u, 11)) { return 4; }
    if(!set_uN_vec_value(*m, st, u8sv{u8"mb"}, 0x400u, 11)) { return 4; }
    simulate(st, 2, false);

    std::uint64_t shr_c{}, shr_a{}, shr_dyn_c{}, shr_dyn_a{}, mul_c{}, mul_a{}, mul_dyn_c{}, mul_dyn_a{}, add_dyn_c{}, add_dyn_a{}, add_const_c{}, add_vc_c{}, add_cv_c{}, sub_dyn_c{}, sub_dyn_a{};
    if(!read_uN_vec(*m, st, u8sv{u8"shr_c"}, shr_c, 15)) { return 5; }
    if(!read_uN_vec(*m, st, u8sv{u8"shr_a"}, shr_a, 15)) { return 6; }
    if(!read_uN_vec(*m, st, u8sv{u8"shr_dyn_c"}, shr_dyn_c, 15)) { return 6; }
    if(!read_uN_vec(*m, st, u8sv{u8"shr_dyn_a"}, shr_dyn_a, 15)) { return 6; }
    if(!read_uN_vec(*m, st, u8sv{u8"mul_c"}, mul_c, 16)) { return 7; }
    if(!read_uN_vec(*m, st, u8sv{u8"mul_a"}, mul_a, 16)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"mul_dyn_c"}, mul_dyn_c, 22)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"mul_dyn_a"}, mul_dyn_a, 22)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"add_dyn_c"}, add_dyn_c, 12)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"add_dyn_a"}, add_dyn_a, 12)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"add_const_c"}, add_const_c, 12)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"add_vc_c"}, add_vc_c, 12)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"add_cv_c"}, add_cv_c, 12)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"sub_dyn_c"}, sub_dyn_c, 12)) { return 8; }
    if(!read_uN_vec(*m, st, u8sv{u8"sub_dyn_a"}, sub_dyn_a, 12)) { return 8; }

    std::fprintf(stderr,
                 "shr_c=0x%04llx shr_a=0x%04llx shr_dyn_c=0x%04llx shr_dyn_a=0x%04llx mul_c=%llu mul_a=%llu mul_dyn_c=0x%06llx mul_dyn_a=0x%06llx add_vv_c=%llu add_vv_a=%llu add_cc=%llu add_vc=%llu add_cv=%llu sub_vv_c=%llu sub_vv_a=%llu\n",
                 static_cast<unsigned long long>(shr_c),
                 static_cast<unsigned long long>(shr_a),
                 static_cast<unsigned long long>(shr_dyn_c),
                 static_cast<unsigned long long>(shr_dyn_a),
                 static_cast<unsigned long long>(mul_c),
                 static_cast<unsigned long long>(mul_a),
                 static_cast<unsigned long long>(mul_dyn_c),
                 static_cast<unsigned long long>(mul_dyn_a),
                 static_cast<unsigned long long>(add_dyn_c),
                 static_cast<unsigned long long>(add_dyn_a),
                 static_cast<unsigned long long>(add_const_c),
                 static_cast<unsigned long long>(add_vc_c),
                 static_cast<unsigned long long>(add_cv_c),
                 static_cast<unsigned long long>(sub_dyn_c),
                 static_cast<unsigned long long>(sub_dyn_a));

    if(shr_c != 0x1800u) { return 9; }
    if(shr_a != 0x1800u) { return 10; }
    if(shr_dyn_c != 0x1800u) { return 10; }
    if(shr_dyn_a != 0x1800u) { return 10; }
    if(mul_c != 6u) { return 11; }
    if(mul_a != 6u) { return 12; }
    if(mul_dyn_c != (0x600u * 0x400u)) { return 13; }
    if(mul_dyn_a != (0x600u * 0x400u)) { return 14; }
    // Verilog sizing: '+' result width is max operand width (no carry-out bit).
    if(add_dyn_c != ((0x600u + 0x400u) & 0x7FFu)) { return 15; }
    if(add_dyn_a != ((0x600u + 0x400u) & 0x7FFu)) { return 16; }
    if(add_const_c != (1536u + 1024u)) { return 19; }
    if(add_vc_c != (0x600u + 1u)) { return 20; }
    if(add_cv_c != (0x400u + 1u)) { return 21; }
    if(sub_dyn_c != (0x600u - 0x400u)) { return 17; }
    if(sub_dyn_a != (0x600u - 0x400u)) { return 18; }

    return 0;
}
