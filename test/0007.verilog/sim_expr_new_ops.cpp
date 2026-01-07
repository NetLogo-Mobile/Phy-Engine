#include <array>
#include <cstdint>

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

    inline bool set_u8_vec(compiled_module const& m, instance_state& st, u8sv base, ::std::uint8_t value)
    {
        auto const* vd{find_vec(m, base)};
        if(vd == nullptr || vd->bits.size() != 8) { return false; }
        for(::std::size_t i{}; i < 8; ++i)
        {
            bool const bit{((value >> (7 - i)) & 1u) != 0u};
            auto const sig{vd->bits.index_unchecked(i)};
            if(sig >= st.state.values.size()) { return false; }
            st.state.values.index_unchecked(sig) = bit ? logic_t::true_state : logic_t::false_state;
        }
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

    inline bool read_u8_vec(compiled_module const& m, instance_state const& st, u8sv base, ::std::uint8_t& out)
    {
        auto const* vd{find_vec(m, base)};
        if(vd == nullptr || vd->bits.size() != 8) { return false; }

        ::std::uint8_t v{};
        for(::std::size_t i{}; i < 8; ++i)
        {
            auto const sig{vd->bits.index_unchecked(i)};
            if(sig >= st.state.values.size()) { return false; }
            auto const bit{st.state.values.index_unchecked(sig)};
            if(bit != logic_t::false_state && bit != logic_t::true_state) { return false; }
            if(bit == logic_t::true_state) { v |= static_cast<::std::uint8_t>(1u << (7 - i)); }
        }
        out = v;
        return true;
    }

    inline bool read_u8_all_x(compiled_module const& m, instance_state const& st, u8sv base)
    {
        auto const* vd{find_vec(m, base)};
        if(vd == nullptr || vd->bits.size() != 8) { return false; }
        for(::std::size_t i{}; i < 8; ++i)
        {
            auto const sig{vd->bits.index_unchecked(i)};
            if(sig >= st.state.values.size()) { return false; }
            if(st.state.values.index_unchecked(sig) != logic_t::indeterminate_state) { return false; }
        }
        return true;
    }
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module expr_new_ops(
  input dyn_bit,
  input sh_x,
  input ax_x,
  input signed [7:0] a_s,
  input [7:0] a_u,
  output [7:0] y_mod_const,
  output [7:0] y_mod_div0,
  output [7:0] y_mod_dyn,
  output [7:0] y_pow_const,
  output [7:0] y_pow_wrap,
  output [7:0] y_pow_dyn,
  output y_case_eq_x,
  output y_case_neq_x,
  output y_eq_x,
  output y_case_eq_z,
  output y_eq_z,
  output y_red_and,
  output y_red_or,
  output y_red_xor,
  output y_red_xor_x,
  output y_lit_signed_lt,
  output y_lit_unsigned_lt,
  output y_port_signed_lt,
  output y_port_unsigned_lt,
  output [7:0] y_signed_ext,
  output [7:0] y_unsized_hex,
  output [7:0] y_unsized_signed,
  output [7:0] y_underscore_hex,
  output [7:0] y_xarith,
  output [7:0] y_xshift
);
  wire [7:0] dyn_vec;
  wire [7:0] sh_vec;
  wire [7:0] ax_vec;
  assign dyn_vec = {7'd0, dyn_bit};
  assign sh_vec  = {7'd0, sh_x};
  assign ax_vec  = {7'd0, ax_x};

  // Modulo / Power
  assign y_mod_const = 8'd7 % 8'd3;
  assign y_mod_div0  = 8'd7 % 8'd0;
  assign y_mod_dyn   = dyn_vec % 8'd3;

  assign y_pow_const = 8'd3 ** 8'd2;  // 9
  assign y_pow_wrap  = 8'd2 ** 8'd8;  // 256 -> 0 in 8-bit
  assign y_pow_dyn   = dyn_vec ** 8'd2;

  // Case equality vs normal equality
  assign y_case_eq_x  = (1'bx === 1'bx);
  assign y_case_neq_x = (1'bx !== 1'bx);
  assign y_eq_x       = (1'bx ==  1'bx); // should be x
  assign y_case_eq_z  = (1'bz === 1'bz);
  assign y_eq_z       = (1'bz ==  1'bz); // should be x

  // Reduction operators
  assign y_red_and   = &4'b1111;
  assign y_red_or    = |4'b0000;
  assign y_red_xor   = ^4'b1010;
  assign y_red_xor_x = ^4'b10x1;

  // Signed arithmetic/comparisons
  assign y_lit_signed_lt   = (8'sd-1 < 8'sd1);
  assign y_lit_unsigned_lt = (8'd255 < 8'd1);

  assign y_port_signed_lt   = (a_s < 8'sd1);
  assign y_port_unsigned_lt = (a_u < 8'd1);

  // Sign extension on assignment (4-bit signed -> 8-bit)
  assign y_signed_ext = 4'sd-1;

  // Numeric literals
  assign y_unsized_hex     = 'hFF;
  assign y_unsized_signed  = 'sd-1;
  assign y_underscore_hex  = 8'hF_F;

  // Better 4-state semantics for arithmetic/shift (x in operand => all x)
  assign y_xarith = ax_vec + 8'd1;
  assign y_xshift = 8'd1 << sh_vec;
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
    auto cr{compile(sv)};
    if(!cr.errors.empty()) { return 1; }

    auto d{build_design(::std::move(cr))};
    auto const* top{find_module(d, u8sv{u8"expr_new_ops"})};
    if(top == nullptr) { return 1; }

    auto st{elaborate(d, *top)};

    // Drive inputs:
    if(!set_scalar(*top, st, u8sv{u8"dyn_bit"}, logic_t::true_state)) { return 1; }
    if(!set_scalar(*top, st, u8sv{u8"sh_x"}, logic_t::indeterminate_state)) { return 1; }
    if(!set_scalar(*top, st, u8sv{u8"ax_x"}, logic_t::indeterminate_state)) { return 1; }
    if(!set_u8_vec(*top, st, u8sv{u8"a_s"}, 0xFF)) { return 1; }
    if(!set_u8_vec(*top, st, u8sv{u8"a_u"}, 0xFF)) { return 1; }

    simulate(st, 1, true);

    // Modulo / Power (const)
    {
        ::std::uint8_t v{};
        if(!read_u8_vec(*top, st, u8sv{u8"y_mod_const"}, v) || v != 0x01) { return 1; }
        if(!read_u8_all_x(*top, st, u8sv{u8"y_mod_div0"})) { return 1; }
        if(!read_u8_all_x(*top, st, u8sv{u8"y_mod_dyn"})) { return 1; }

        if(!read_u8_vec(*top, st, u8sv{u8"y_pow_const"}, v) || v != 0x09) { return 1; }
        if(!read_u8_vec(*top, st, u8sv{u8"y_pow_wrap"}, v) || v != 0x00) { return 1; }
        if(!read_u8_all_x(*top, st, u8sv{u8"y_pow_dyn"})) { return 1; }
    }

    // Case equality / equality
    {
        logic_t v{};
        if(!read_scalar(*top, st, u8sv{u8"y_case_eq_x"}, v) || v != logic_t::true_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_case_neq_x"}, v) || v != logic_t::false_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_eq_x"}, v) || v != logic_t::indeterminate_state) { return 1; }

        if(!read_scalar(*top, st, u8sv{u8"y_case_eq_z"}, v) || v != logic_t::true_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_eq_z"}, v) || v != logic_t::indeterminate_state) { return 1; }
    }

    // Reduction
    {
        logic_t v{};
        if(!read_scalar(*top, st, u8sv{u8"y_red_and"}, v) || v != logic_t::true_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_red_or"}, v) || v != logic_t::false_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_red_xor"}, v) || v != logic_t::false_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_red_xor_x"}, v) || v != logic_t::indeterminate_state) { return 1; }
    }

    // Signed compare
    {
        logic_t v{};
        if(!read_scalar(*top, st, u8sv{u8"y_lit_signed_lt"}, v) || v != logic_t::true_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_lit_unsigned_lt"}, v) || v != logic_t::false_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_port_signed_lt"}, v) || v != logic_t::true_state) { return 1; }
        if(!read_scalar(*top, st, u8sv{u8"y_port_unsigned_lt"}, v) || v != logic_t::false_state) { return 1; }
    }

    // Signed extension + numeric literal parsing
    {
        ::std::uint8_t v{};
        if(!read_u8_vec(*top, st, u8sv{u8"y_signed_ext"}, v) || v != 0xFF) { return 1; }
        if(!read_u8_vec(*top, st, u8sv{u8"y_unsized_hex"}, v) || v != 0xFF) { return 1; }
        if(!read_u8_vec(*top, st, u8sv{u8"y_unsized_signed"}, v) || v != 0xFF) { return 1; }
        if(!read_u8_vec(*top, st, u8sv{u8"y_underscore_hex"}, v) || v != 0xFF) { return 1; }
    }

    // 4-state arithmetic / shift
    if(!read_u8_all_x(*top, st, u8sv{u8"y_xarith"})) { return 1; }
    if(!read_u8_all_x(*top, st, u8sv{u8"y_xshift"})) { return 1; }

    return 0;
}
