#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>

#include <cstddef>
#include <vector>

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    // Intentionally create a pure combinational cycle. O3 passes should never crash here.
    auto const src = ::fast_io::u8string_view{u8R"(
module comb_cycle(
    input  wire a,
    output wire y
);
    wire w1;
    wire w2;
    assign w1 = ~w2;
    assign w2 = ~w1;
    assign y = w1;
endmodule
)"};

    compile_options copt{};
    auto cr = compile(src, copt);
    if(!cr.errors.empty()) { return 1; }
    if(cr.modules.empty()) { return 2; }

    auto design = build_design(::std::move(cr));
    auto const* top_mod = find_module(design, u8"comb_cycle");
    if(top_mod == nullptr) { return 3; }

    auto top_inst = elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 4; }

    phy_engine::circult c{};
    c.set_analyze_type(phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    std::vector<phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 5; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 6; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 7; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 8; }
        }
        else
        {
            return 9;
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = 3,
        .optimize_wires = true,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 10; }
    return 0;
}
