#include <cstddef>
#include <cstdint>

#include <phy_engine/verilog/digital/digital.h>

namespace
{
using namespace ::phy_engine::verilog::digital;
using u8sv = ::fast_io::u8string_view;

inline vector_desc const* find_vec(compiled_module const& m, u8sv base)
{
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end()) { return nullptr; }
    return __builtin_addressof(it->second);
}

inline bool set_u4_vec(compiled_module const& m, instance_state& st, u8sv base, ::std::uint8_t value)
{
    auto const* vd{find_vec(m, base)};
    if(vd == nullptr || vd->bits.size() != 4) { return false; }
    for(::std::size_t i{}; i < 4; ++i)
    {
        bool const bit{((value >> (3 - i)) & 1u) != 0u};
        auto const sig{vd->bits.index_unchecked(i)};
        if(sig >= st.state.values.size()) { return false; }
        st.state.values.index_unchecked(sig) = bit ? logic_t::true_state : logic_t::false_state;
    }
    return true;
}

inline bool set_vec_bit(compiled_module const& m, instance_state& st, u8sv base, ::std::size_t pos_msb_to_lsb, logic_t v)
{
    auto const* vd{find_vec(m, base)};
    if(vd == nullptr || pos_msb_to_lsb >= vd->bits.size()) { return false; }
    auto const sig{vd->bits.index_unchecked(pos_msb_to_lsb)};
    if(sig >= st.state.values.size()) { return false; }
    st.state.values.index_unchecked(sig) = v;
    return true;
}

inline bool read_u4_vec(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint8_t& out)
{
    auto const* vd{find_vec(m, base)};
    if(vd == nullptr || vd->bits.size() != 4) { return false; }

    ::std::uint8_t v{};
    for(::std::size_t i{}; i < 4; ++i)
    {
        auto const sig{vd->bits.index_unchecked(i)};
        if(sig >= st.state.values.size()) { return false; }
        auto const bit{st.state.values.index_unchecked(sig)};
        if(bit != logic_t::false_state && bit != logic_t::true_state) { return false; }
        if(bit == logic_t::true_state) { v |= static_cast<::std::uint8_t>(1u << (3 - i)); }
    }
    out = v;
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
module xnor_test(
  input  [3:0] a,
  input  [3:0] b,
  output [3:0] y_tildecaret,
  output [3:0] y_carettilde,
  output       y_red_tildecaret,
  output       y_red_carettilde
);
  assign y_tildecaret = a ~^ b;
  assign y_carettilde = a ^~ b;
  assign y_red_tildecaret = ~^a;
  assign y_red_carettilde = ^~a;
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
    auto cr{compile(sv)};
    if(!cr.errors.empty()) { return 1; }

    auto d{build_design(::std::move(cr))};
    auto const* top{find_module(d, u8sv{u8"xnor_test"})};
    if(top == nullptr) { return 1; }

    auto st{elaborate(d, *top)};

    // Known inputs.
    if(!set_u4_vec(*top, st, u8sv{u8"a"}, 0b1010)) { return 1; }
    if(!set_u4_vec(*top, st, u8sv{u8"b"}, 0b1100)) { return 1; }
    simulate(st, 1, true);

    {
        ::std::uint8_t v{};
        if(!read_u4_vec(*top, st, u8sv{u8"y_tildecaret"}, v) || v != 0b1001) { return 1; }
        if(!read_u4_vec(*top, st, u8sv{u8"y_carettilde"}, v) || v != 0b1001) { return 1; }

        logic_t s{};
        if(!read_scalar(*top, st, u8sv{u8"y_red_tildecaret"}, s) || s != logic_t::true_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_red_carettilde"}, s) || s != logic_t::true_state) { return 1; }
    }

    // Unknown bit in a => reduction xnor becomes X, and the matching bit in bitwise xnor becomes X.
    if(!set_vec_bit(*top, st, u8sv{u8"a"}, 1, logic_t::indeterminate_state)) { return 1; }  // a[2] (msb->lsb)
    simulate(st, 2, true);

    {
        logic_t s{};
        if(!read_scalar(*top, st, u8sv{u8"y_red_tildecaret"}, s) || s != logic_t::indeterminate_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_red_carettilde"}, s) || s != logic_t::indeterminate_state) { return 1; }
    }

    return 0;
}

