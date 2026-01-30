#include <cstdint>
#include <string>

#include <phy_engine/verilog/digital/digital.h>

int main()
{
    using namespace phy_engine::verilog::digital;

    decltype(auto) src = u8R"(
module top(input logic a, input logic b, output logic y);
  function automatic logic f_and(input logic x, input logic y0);
    return x & y0;
  endfunction

  always_comb begin
    y = f_and(a, b);
  end
endmodule
)";

    auto cr = compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }

    auto design = build_design(::std::move(cr));
    auto const* top_mod = find_module(design, u8"top");
    if(top_mod == nullptr) { return 2; }
    auto inst = elaborate(design, *top_mod);
    if(inst.mod == nullptr) { return 3; }

    auto set_in = [&](::fast_io::u8string_view nm, bool v) noexcept
    {
        for(auto const& p: inst.mod->ports)
        {
            if(p.dir != port_dir::input) { continue; }
            if(p.name != nm) { continue; }
            inst.state.values.index_unchecked(p.signal) = v ? logic_t::true_state : logic_t::false_state;
        }
    };

    auto read_out = [&](::fast_io::u8string_view nm, logic_t& out) noexcept -> bool
    {
        for(auto const& p: inst.mod->ports)
        {
            if(p.dir != port_dir::output) { continue; }
            if(p.name != nm) { continue; }
            out = inst.state.values.index_unchecked(p.signal);
            return true;
        }
        return false;
    };

    for(unsigned mask = 0; mask < 4; ++mask)
    {
        bool const a = (mask & 1u) != 0u;
        bool const b = (mask & 2u) != 0u;
        set_in(u8"a", a);
        set_in(u8"b", b);
        simulate(inst, mask);  // tick value irrelevant; ensure always_comb runs

        logic_t y{};
        if(!read_out(u8"y", y)) { return 4; }
        auto const exp = (a && b) ? logic_t::true_state : logic_t::false_state;
        if(y != exp) { return 5; }
    }

    return 0;
}

