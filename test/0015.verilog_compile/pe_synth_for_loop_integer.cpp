#include <array>
#include <cstddef>
#include <cstdio>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
inline ::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module top(input [7:0] b, output reg y);
  integer i;
  always @* begin
    y = 1'b0;
    for(i = 0; i < 8; i = i + 1) begin
      if (b[i]) y = 1'b1;
    end
  end
endmodule
)";

    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;
    auto& nl = cct.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"top");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    ::std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    auto port_idx = [&](::fast_io::u8string_view name) noexcept -> ::std::size_t
    {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            if(p.name == name) { return i; }
        }
        return SIZE_MAX;
    };

    ::std::array<::std::size_t, 8> b_idx{
        port_idx(u8"b[7]"),
        port_idx(u8"b[6]"),
        port_idx(u8"b[5]"),
        port_idx(u8"b[4]"),
        port_idx(u8"b[3]"),
        port_idx(u8"b[2]"),
        port_idx(u8"b[1]"),
        port_idx(u8"b[0]"),
    };
    for(auto v : b_idx)
    {
        if(v == SIZE_MAX) { return 4; }
    }

    auto const y_idx = port_idx(u8"y");
    if(y_idx == SIZE_MAX) { return 5; }

    ::std::array<::phy_engine::model::model_base*, 8> in_b{};
    for(auto& p : in_b)
    {
        auto [m, pos] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr) { return 6; }
        p = m;
    }

    auto [out_y, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)pos;
    if(out_y == nullptr) { return 7; }

    for(::std::size_t i{}; i < 8; ++i)
    {
        if(!::phy_engine::netlist::add_to_node(nl, *in_b[i], 0, *ports[b_idx[i]])) { return 8; }
    }
    if(!::phy_engine::netlist::add_to_node(nl, *out_y, 0, *ports[y_idx])) { return 9; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = true,
        .allow_multi_driver = true,
        .assume_binary_inputs = true,
        .optimize_wires = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 10; }

    std::size_t models{};
    for(auto const& blk : nl.models) { models += static_cast<std::size_t>(blk.curr - blk.begin); }
    if(models > 5000)
    {
        std::fprintf(stderr, "unexpected model blow-up: models=%zu\n", models);
        return 11;
    }

    if(!cct.analyze()) { return 12; }

    auto settle = [&]() noexcept
    {
        cct.digital_clk();
        cct.digital_clk();
    };

    auto set_in = [&](::phy_engine::model::model_base* in, bool bit) noexcept
    {
        (void)in->ptr->set_attribute(0, dv(bit ? ::phy_engine::model::digital_node_statement_t::true_state
                                               : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto all_zero = [&]() noexcept
    {
        for(auto* in : in_b) { set_in(in, false); }
    };

    // b[7]=1 should still be observed (catches incorrect integer loop handling).
    all_zero();
    set_in(in_b[0], true);  // b[7]
    settle();
    if(ports[y_idx]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 13; }

    // b[0]=1
    all_zero();
    set_in(in_b[7], true);  // b[0]
    settle();
    if(ports[y_idx]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 14; }

    // all zero => y=0
    all_zero();
    settle();
    if(ports[y_idx]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 15; }

    return 0;
}

