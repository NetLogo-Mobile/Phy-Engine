#include <phy_engine/model/models/digital/logical/and.h>
#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>

int main()
{
    using namespace phy_engine;

    netlist::netlist nl{};

    auto& n_a = netlist::create_node(nl);
    auto& n_y = netlist::create_node(nl);

    auto [in_a, p0] = netlist::add_model(nl, model::INPUT{.outputA = model::digital_node_statement_t::false_state});
    auto [g_and, p1] = netlist::add_model(nl, model::AND{});
    auto [out_y, p2] = netlist::add_model(nl, model::OUTPUT{});
    (void)p0;
    (void)p1;
    (void)p2;
    if(in_a == nullptr || g_and == nullptr || out_y == nullptr) { return 1; }

    // INPUT(o0) -> AND(a0), AND(o2) -> OUTPUT(i0)
    if(!netlist::add_to_node(nl, *in_a, 0, n_a)) { return 2; }
    if(!netlist::add_to_node(nl, *g_and, 0, n_a)) { return 3; }
    if(!netlist::add_to_node(nl, *g_and, 2, n_y)) { return 4; }
    if(!netlist::add_to_node(nl, *out_y, 0, n_y)) { return 5; }

    auto r = phy_lab_wrapper::pe_to_pl::convert(nl);

    assert(r.ex.type() == phy_lab_wrapper::experiment_type::circuit);
    assert(r.ex.wires().empty());
    assert(r.ex.elements().size() == 3);

    bool has_logic_input{};
    bool has_and_gate{};
    bool has_logic_output{};
    for(auto const& e : r.ex.elements())
    {
        auto const mid = e.data().value("ModelID", "");
        if(mid == "Logic Input") { has_logic_input = true; }
        if(mid == "And Gate") { has_and_gate = true; }
        if(mid == "Logic Output") { has_logic_output = true; }
        auto const p = e.element_position();
        assert(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
    }
    assert(has_logic_input && has_and_gate && has_logic_output);

    // We should have 2 nets with 2 endpoints each.
    std::size_t nets2{};
    for(auto const& n : r.nets)
    {
        if(n.endpoints.size() == 2) { ++nets2; }
    }
    assert(nets2 >= 2);

    return 0;
}
