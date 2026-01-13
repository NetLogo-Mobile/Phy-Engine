#include <cstddef>

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
    // Regression: child output ports drive parent nets via instance_state::output_drives.
    decltype(auto) src = u8R"(
module child(input a, output y);
  assign y = a;
endmodule
module top(input a, output y);
  child u(.a(a), .y(y));
endmodule
)";

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;

    auto& nl = c.get_netlist();

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

    auto port_idx = [&](::fast_io::u8string_view name) noexcept -> ::std::size_t {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            if(p.name == name) { return i; }
        }
        return SIZE_MAX;
    };

    auto const i_a = port_idx(u8"a");
    auto const i_y = port_idx(u8"y");
    if(i_a == SIZE_MAX || i_y == SIZE_MAX) { return 4; }

    auto [in_a, p0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [out_y, p1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)p0;
    (void)p1;
    if(in_a == nullptr || out_y == nullptr) { return 5; }

    if(!::phy_engine::netlist::add_to_node(nl, *in_a, 0, *ports[i_a])) { return 6; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_y, 0, *ports[i_y])) { return 7; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 8; }
    if(!c.analyze()) { return 9; }

    (void)in_a->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state));
    c.digital_clk();
    c.digital_clk();

    if(ports[i_y]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 10; }
    return 0;
}

