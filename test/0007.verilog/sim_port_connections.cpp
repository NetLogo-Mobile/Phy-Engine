#include <cstdint>
#include <cstdio>

#include <phy_engine/verilog/digital/digital.h>

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

    inline bool set_scalar_value(compiled_module const& m, instance_state& st, u8sv name, logic_t v)
    {
        auto const sig{find_signal(m, name)};
        if(sig == SIZE_MAX || sig >= st.state.values.size()) { return false; }
        st.state.values.index_unchecked(sig) = v;
        return true;
    }
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module child4(input [3:0] a, output [3:0] y);
  assign y = a;
endmodule

module top_expr_conn(input [3:0] x, input [3:0] y, output [3:0] z);
  child4 u(.a(x + y), .y(z));
endmodule

module child8s(input signed [7:0] a, output [7:0] y);
  assign y = a;
endmodule

module top_signext_in(input signed [3:0] s, output [7:0] y);
  child8s u(.a(s), .y(y));
endmodule

module child8u(input [7:0] a, output [7:0] y);
  assign y = a;
endmodule

module top_zeroext_in(input [3:0] u, output [7:0] y);
  child8u u0(.a(u), .y(y));
endmodule

module top_trunc_in(input [7:0] x, output [3:0] y);
  child4 u(.a(x), .y(y));
endmodule

module child_out4s(output signed [3:0] y);
  assign y = 4'sd-1;
endmodule

module top_signext_out(output [7:0] out);
  child_out4s u(.y(out));
endmodule

module child_out8(output [7:0] y);
  assign y = 8'b10100101;
endmodule

module top_trunc_out(output [3:0] out);
  child_out8 u(.y(out));
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
    auto cr{compile(sv)};
    if(!cr.errors.empty()) { return 1; }
    auto d{build_design(::std::move(cr))};

    // General expression connection: .a(x + y)
    {
        auto const* top{find_module(d, u8sv{u8"top_expr_conn"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_uN_vec_value(*top, st, u8sv{u8"x"}, 3u, 4)) { return 1; }
        if(!set_uN_vec_value(*top, st, u8sv{u8"y"}, 5u, 4)) { return 1; }
        simulate(st, 1, false);
        ::std::uint64_t z{};
        if(!read_uN_vec(*top, st, u8sv{u8"z"}, z, 4)) { return 1; }
        if(z != 8u) { ::std::fprintf(stderr, "expr_conn: 3+5 => %llu\n", static_cast<unsigned long long>(z)); return 1; }

        if(!set_uN_vec_value(*top, st, u8sv{u8"x"}, 12u, 4)) { return 1; }
        if(!set_uN_vec_value(*top, st, u8sv{u8"y"}, 7u, 4)) { return 1; }
        simulate(st, 2, false);
        if(!read_uN_vec(*top, st, u8sv{u8"z"}, z, 4)) { return 1; }
        if(z != 3u) { ::std::fprintf(stderr, "expr_conn: 12+7 => %llu\n", static_cast<unsigned long long>(z)); return 1; }
    }

    // Width coercion (input): sign extension
    {
        auto const* top{find_module(d, u8sv{u8"top_signext_in"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_uN_vec_value(*top, st, u8sv{u8"s"}, 0b1111u, 4)) { return 1; }  // -1 in 4-bit signed
        simulate(st, 1, false);
        ::std::uint64_t y{};
        if(!read_uN_vec(*top, st, u8sv{u8"y"}, y, 8)) { return 1; }
        if(y != 0xFFu) { ::std::fprintf(stderr, "signext_in: -1 => 0x%llx\n", static_cast<unsigned long long>(y)); return 1; }

        if(!set_uN_vec_value(*top, st, u8sv{u8"s"}, 0b0111u, 4)) { return 1; }
        simulate(st, 2, false);
        if(!read_uN_vec(*top, st, u8sv{u8"y"}, y, 8)) { return 1; }
        if(y != 0x07u) { ::std::fprintf(stderr, "signext_in: 7 => 0x%llx\n", static_cast<unsigned long long>(y)); return 1; }
    }

    // Width coercion (input): zero extension
    {
        auto const* top{find_module(d, u8sv{u8"top_zeroext_in"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_uN_vec_value(*top, st, u8sv{u8"u"}, 0b1111u, 4)) { return 1; }
        simulate(st, 1, false);
        ::std::uint64_t y{};
        if(!read_uN_vec(*top, st, u8sv{u8"y"}, y, 8)) { return 1; }
        if(y != 0x0Fu) { ::std::fprintf(stderr, "zeroext_in: 15 => 0x%llx\n", static_cast<unsigned long long>(y)); return 1; }
    }

    // Width coercion (input): truncation keeps LSBs
    {
        auto const* top{find_module(d, u8sv{u8"top_trunc_in"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_uN_vec_value(*top, st, u8sv{u8"x"}, 0b10100101u, 8)) { return 1; }
        simulate(st, 1, false);
        ::std::uint64_t y{};
        if(!read_uN_vec(*top, st, u8sv{u8"y"}, y, 4)) { return 1; }
        if(y != 0b0101u) { ::std::fprintf(stderr, "trunc_in: exp=5 got=%llu\n", static_cast<unsigned long long>(y)); return 1; }
    }

    // Width coercion (output): sign extension uses port signedness
    {
        auto const* top{find_module(d, u8sv{u8"top_signext_out"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        simulate(st, 1, false);
        ::std::uint64_t out{};
        if(!read_uN_vec(*top, st, u8sv{u8"out"}, out, 8)) { return 1; }
        if(out != 0xFFu) { ::std::fprintf(stderr, "signext_out: exp=0xFF got=0x%llx\n", static_cast<unsigned long long>(out)); return 1; }
    }

    // Width coercion (output): truncation keeps LSBs
    {
        auto const* top{find_module(d, u8sv{u8"top_trunc_out"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        simulate(st, 1, false);
        ::std::uint64_t out{};
        if(!read_uN_vec(*top, st, u8sv{u8"out"}, out, 4)) { return 1; }
        if(out != 0b0101u) { ::std::fprintf(stderr, "trunc_out: exp=5 got=%llu\n", static_cast<unsigned long long>(out)); return 1; }
    }

    // Smoke: scalar ports (exercise direct bit binding path)
    {
        auto const* top{find_module(d, u8sv{u8"child_out4s"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};
        simulate(st, 1, false);
        logic_t y{};
        auto const sig{find_signal(*top, u8sv{u8"y[3]"})};
        if(sig == SIZE_MAX || sig >= st.state.values.size()) { return 1; }
        y = st.state.values.index_unchecked(sig);
        if(y != logic_t::true_state) { return 1; }
    }
    return 0;
}

