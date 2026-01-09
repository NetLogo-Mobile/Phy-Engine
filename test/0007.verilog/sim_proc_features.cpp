#include <cstdint>

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

    inline bool set_scalar_value(compiled_module const& m, instance_state& st, u8sv name, logic_t v)
    {
        auto const sig{find_signal(m, name)};
        if(sig == SIZE_MAX || sig >= st.state.values.size()) { return false; }
        st.state.values.index_unchecked(sig) = v;
        return true;
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

    inline bool read_scalar(compiled_module const& m, instance_state const& st, u8sv name, logic_t& out)
    {
        auto const sig{find_signal(m, name)};
        if(sig == SIZE_MAX || sig >= st.state.values.size()) { return false; }
        out = st.state.values.index_unchecked(sig);
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

    inline void commit_prev(instance_state& st) { st.state.prev_values = st.state.values; st.state.comb_prev_values = st.state.values; }
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module loop_for(input [3:0] a, output reg [3:0] y);
  reg [2:0] i;
  always @* begin
    y = 4'b0000;
    for(i = 0; i < 4; i = i + 1) begin
      y[i] = a[i];
    end
  end
endmodule

module loop_while(input [3:0] a, output reg [3:0] y);
  reg [2:0] i;
  always @* begin
    y = 4'b0000;
    i = 0;
    while(i < 4) begin
      y[i] = a[i];
      i = i + 1;
    end
  end
endmodule

module loop_repeat(input [3:0] a, output reg [3:0] y);
  reg [2:0] i;
  always @* begin
    y = 4'b0000;
    i = 0;
    repeat(4) begin
      y[i] = a[i];
      i = i + 1;
    end
  end
endmodule

module casez2(input [1:0] s, output reg y);
  always @* begin
    casez(s)
      2'b0z: y = 1'b0;
      2'b1z: y = 1'b1;
      default: y = 1'bx;
    endcase
  end
endmodule

module casex2(input [1:0] s, output reg y);
  always @* begin
    casex(s)
      2'bx1: y = 1'b1;
      default: y = 1'b0;
    endcase
  end
endmodule

module kw_always_comb(input a, input b, output reg y);
  always_comb begin
    y = a & b;
  end
endmodule

module kw_always_ff(input clk, input rst_n, input d, output reg q);
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) q <= 1'b0;
    else q <= d;
  end
endmodule

module sens_nba(input trig, output reg x, output reg y);
  always @(trig) begin
    x <= 1'b0;
    y = x;
  end
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};
    auto cr{compile(sv)};
    if(!cr.errors.empty()) { return 1; }
    auto d{build_design(::std::move(cr))};

    // Loop tests
    for(auto const top_name: {u8sv{u8"loop_for"}, u8sv{u8"loop_while"}, u8sv{u8"loop_repeat"}})
    {
        auto const* top{find_module(d, top_name)};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_uN_vec_value(*top, st, u8sv{u8"a"}, 0u, 4)) { return 1; }
        commit_prev(st);
        if(!set_uN_vec_value(*top, st, u8sv{u8"a"}, 0b1011u, 4)) { return 1; }
        simulate(st, 1, false);

        ::std::uint64_t y{};
        if(!read_uN_vec(*top, st, u8sv{u8"y"}, y, 4)) { return 1; }
        if(y != 0b1011u) { return 1; }
    }

    // casez
    {
        auto const* top{find_module(d, u8sv{u8"casez2"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        auto run_one = [&](::std::uint64_t s, logic_t exp) -> bool
        {
            ::std::uint64_t const base{s == 0 ? 1u : 0u};
            if(!set_uN_vec_value(*top, st, u8sv{u8"s"}, base, 2)) { return false; }
            commit_prev(st);
            if(!set_uN_vec_value(*top, st, u8sv{u8"s"}, s, 2)) { return false; }
            simulate(st, 1, false);

            logic_t y{};
            return read_scalar(*top, st, u8sv{u8"y"}, y) && y == exp;
        };

        if(!run_one(0b00u, logic_t::false_state)) { return 1; }
        if(!run_one(0b01u, logic_t::false_state)) { return 1; }
        if(!run_one(0b10u, logic_t::true_state)) { return 1; }
        if(!run_one(0b11u, logic_t::true_state)) { return 1; }
    }

    // casex
    {
        auto const* top{find_module(d, u8sv{u8"casex2"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        auto run_one = [&](::std::uint64_t s, logic_t exp) -> bool
        {
            ::std::uint64_t const base{s == 0 ? 1u : 0u};
            if(!set_uN_vec_value(*top, st, u8sv{u8"s"}, base, 2)) { return false; }
            commit_prev(st);
            if(!set_uN_vec_value(*top, st, u8sv{u8"s"}, s, 2)) { return false; }
            simulate(st, 1, false);
            logic_t y{};
            return read_scalar(*top, st, u8sv{u8"y"}, y) && y == exp;
        };

        if(!run_one(0b00u, logic_t::false_state)) { return 1; }
        if(!run_one(0b01u, logic_t::true_state)) { return 1; }
        if(!run_one(0b10u, logic_t::false_state)) { return 1; }
        if(!run_one(0b11u, logic_t::true_state)) { return 1; }
    }

    // always_comb keyword
    {
        auto const* top{find_module(d, u8sv{u8"kw_always_comb"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_scalar_value(*top, st, u8sv{u8"a"}, logic_t::false_state)) { return 1; }
        if(!set_scalar_value(*top, st, u8sv{u8"b"}, logic_t::true_state)) { return 1; }
        commit_prev(st);
        if(!set_scalar_value(*top, st, u8sv{u8"a"}, logic_t::true_state)) { return 1; }
        simulate(st, 1, false);
        logic_t y{};
        if(!read_scalar(*top, st, u8sv{u8"y"}, y) || y != logic_t::true_state) { return 1; }
    }

    // always_ff keyword + multi-event control (posedge clk or negedge rst_n)
    {
        auto const* top{find_module(d, u8sv{u8"kw_always_ff"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_scalar_value(*top, st, u8sv{u8"clk"}, logic_t::false_state)) { return 1; }
        if(!set_scalar_value(*top, st, u8sv{u8"rst_n"}, logic_t::true_state)) { return 1; }
        if(!set_scalar_value(*top, st, u8sv{u8"d"}, logic_t::false_state)) { return 1; }
        commit_prev(st);

        // posedge clk captures d=1
        if(!set_scalar_value(*top, st, u8sv{u8"d"}, logic_t::true_state)) { return 1; }
        if(!set_scalar_value(*top, st, u8sv{u8"clk"}, logic_t::true_state)) { return 1; }
        simulate(st, 1, true);
        logic_t q{};
        if(!read_scalar(*top, st, u8sv{u8"q"}, q) || q != logic_t::true_state) { return 1; }

        // negedge rst_n resets
        commit_prev(st);
        if(!set_scalar_value(*top, st, u8sv{u8"rst_n"}, logic_t::false_state)) { return 1; }
        simulate(st, 2, true);
        if(!read_scalar(*top, st, u8sv{u8"q"}, q) || q != logic_t::false_state) { return 1; }
    }

    // Explicit sensitivity scheduling + nonblocking vs blocking ordering
    {
        auto const* top{find_module(d, u8sv{u8"sens_nba"})};
        if(top == nullptr) { return 1; }
        auto st{elaborate(d, *top)};

        if(!set_scalar_value(*top, st, u8sv{u8"trig"}, logic_t::false_state)) { return 1; }
        if(!set_scalar_value(*top, st, u8sv{u8"x"}, logic_t::true_state)) { return 1; }
        if(!set_scalar_value(*top, st, u8sv{u8"y"}, logic_t::false_state)) { return 1; }
        commit_prev(st);

        // Trigger the always @(trig) block.
        if(!set_scalar_value(*top, st, u8sv{u8"trig"}, logic_t::true_state)) { return 1; }
        simulate(st, 1, false);

        logic_t x{};
        logic_t y{};
        if(!read_scalar(*top, st, u8sv{u8"x"}, x) || !read_scalar(*top, st, u8sv{u8"y"}, y)) { return 1; }
        if(x != logic_t::false_state) { return 1; }
        if(y != logic_t::true_state) { return 1; }
    }

    return 0;
}
