#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include <phy_engine/verilog/digital/digital.h>

int main()
{
    // `$urandom`/`$random` (subset): ensure lexer parses '$', the interpreter updates `$urandom[3:0]`,
    // and reset_n forces outputs to 0.
    decltype(auto) src = u8R"(
module top(input clk, input rst_n, output [3:0] out);
  assign out = $urandom;
endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"top");
    if(mod == nullptr) { return 2; }
    auto top = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top.mod == nullptr) { return 3; }

    auto port_signal = [&](std::string_view name) noexcept -> std::size_t {
        for(std::size_t i{}; i < top.mod->ports.size(); ++i)
        {
            auto const& p = top.mod->ports.index_unchecked(i);
            std::string_view pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
            if(pn == name) { return p.signal; }
        }
        return SIZE_MAX;
    };

    auto const sig_clk = port_signal("clk");
    auto const sig_rstn = port_signal("rst_n");
    if(sig_clk == SIZE_MAX || sig_rstn == SIZE_MAX) { return 4; }

    std::array<std::size_t, 4> sig_out{};
    for(std::size_t i{}; i < 4; ++i)
    {
        sig_out[i] = port_signal("out[" + std::to_string(i) + "]");
        if(sig_out[i] == SIZE_MAX) { return 5; }
    }

    auto set_sig = [&](std::size_t sig, bool v) noexcept {
        if(sig >= top.state.values.size()) { return; }
        top.state.values.index_unchecked(sig) = v ? ::phy_engine::verilog::digital::logic_t::true_state
                                                  : ::phy_engine::verilog::digital::logic_t::false_state;
    };

    auto get_out = [&]() noexcept -> std::uint8_t {
        std::uint8_t v{};
        for(std::size_t i{}; i < 4; ++i)
        {
            if(sig_out[i] >= top.state.values.size()) { continue; }
            auto const b = top.state.values.index_unchecked(sig_out[i]);
            if(b == ::phy_engine::verilog::digital::logic_t::true_state) { v |= static_cast<std::uint8_t>(1u << i); }
            else if(b != ::phy_engine::verilog::digital::logic_t::false_state) { return 0xFFu; }
        }
        return v;
    };

    std::uint64_t tick{};

    // Reset asserted.
    set_sig(sig_rstn, false);
    set_sig(sig_clk, false);
    ::phy_engine::verilog::digital::simulate(top, tick++);
    set_sig(sig_clk, true);
    ::phy_engine::verilog::digital::simulate(top, tick++);
    if(get_out() != 0u) { return 6; }

    // Release reset and expect output to change at least once across a few clocks.
    set_sig(sig_rstn, true);
    std::uint8_t first{};
    bool first_set{};
    bool changed{};
    for(int i = 0; i < 16; ++i)
    {
        set_sig(sig_clk, false);
        ::phy_engine::verilog::digital::simulate(top, tick++);
        set_sig(sig_clk, true);
        ::phy_engine::verilog::digital::simulate(top, tick++);
        auto const now = get_out();
        if(now == 0xFFu) { return 7; }
        if(!first_set)
        {
            first = now;
            first_set = true;
        }
        else if(now != first)
        {
            changed = true;
            break;
        }
    }
    if(!changed) { return 8; }

    return 0;
}
