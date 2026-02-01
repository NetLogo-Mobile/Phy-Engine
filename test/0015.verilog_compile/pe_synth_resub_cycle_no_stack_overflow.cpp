#include <cstddef>
#include <cstdint>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/pe_synth.h>

int main()
{
    using namespace phy_engine;

    // Regression: resub/sweep cone builders must not recurse infinitely on cyclic gate graphs.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& nl = c.get_netlist();

    auto& n0_ref = ::phy_engine::netlist::create_node(nl);
    auto& n1_ref = ::phy_engine::netlist::create_node(nl);
    auto* n0 = __builtin_addressof(n0_ref);
    auto* n1 = __builtin_addressof(n1_ref);

    // Create a small combinational loop: n0 = ~n1, n1 = ~n0.
    {
        auto [m0, pos0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
        (void)pos0;
        if(m0 == nullptr) { return 1; }
        if(!::phy_engine::netlist::add_to_node(nl, *m0, 0, *n1) || !::phy_engine::netlist::add_to_node(nl, *m0, 1, *n0)) { return 2; }
    }
    {
        auto [m1, pos1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
        (void)pos1;
        if(m1 == nullptr) { return 3; }
        if(!::phy_engine::netlist::add_to_node(nl, *m1, 0, *n0) || !::phy_engine::netlist::add_to_node(nl, *m1, 1, *n1)) { return 4; }
    }

    // Keep the loop reachable.
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr || !::phy_engine::netlist::add_to_node(nl, *m, 0, *n0)) { return 5; }
    }

    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = true;
    opt.resub_max_vars = 6;
    opt.resub_max_gates = 64;
    opt.sweep_max_vars = 6;
    opt.sweep_max_gates = 64;

    std::vector<::phy_engine::model::node_t*> prot{};

    (void)::phy_engine::verilog::digital::details::optimize_bounded_resubstitute_in_pe_netlist(nl, prot, opt);
    (void)::phy_engine::verilog::digital::details::optimize_bounded_sweep_in_pe_netlist(nl, prot, opt);

    return 0;
}

