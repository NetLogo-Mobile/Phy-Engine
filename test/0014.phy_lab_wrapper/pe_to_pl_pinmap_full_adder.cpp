#include <phy_engine/model/models/digital/combinational/full_adder.h>
#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>
#include <string>

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::phy_lab_wrapper;

    netlist::netlist nl{};

    auto& na = netlist::create_node(nl);
    auto& nb = netlist::create_node(nl);
    auto& ncin = netlist::create_node(nl);
    auto& ns = netlist::create_node(nl);
    auto& ncout = netlist::create_node(nl);

    auto [in_a, in_a_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_a_pos;
    auto [in_b, in_b_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_b_pos;
    auto [in_cin, in_cin_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_cin_pos;
    auto [out_s, out_s_pos] = netlist::add_model(nl, model::OUTPUT{});
    (void)out_s_pos;
    auto [out_cout, out_cout_pos] = netlist::add_model(nl, model::OUTPUT{});
    (void)out_cout_pos;

    auto [fa, fa_pos] = netlist::add_model(nl, model::FULL_ADDER{});
    (void)fa_pos;

    // PE(FULL_ADDER): ia(A), ib(B), cin(Cin), s(S), cout(Cout)
    assert(netlist::add_to_node(nl, *in_a, 0, na));
    assert(netlist::add_to_node(nl, *in_b, 0, nb));
    assert(netlist::add_to_node(nl, *in_cin, 0, ncin));
    assert(netlist::add_to_node(nl, *out_s, 0, ns));
    assert(netlist::add_to_node(nl, *out_cout, 0, ncout));

    assert(netlist::add_to_node(nl, *fa, 0, na));
    assert(netlist::add_to_node(nl, *fa, 1, nb));
    assert(netlist::add_to_node(nl, *fa, 2, ncin));
    assert(netlist::add_to_node(nl, *fa, 3, ns));
    assert(netlist::add_to_node(nl, *fa, 4, ncout));

    auto r = pe_to_pl::convert(nl);

    // Find PL Full Adder element identifier.
    std::string pl_fa_id{};
    for(auto const& e : r.ex.elements())
    {
        if(e.data().value("ModelID", "") == "Full Adder")
        {
            pl_fa_id = e.identifier();
            break;
        }
    }
    assert(!pl_fa_id.empty());

    auto assert_has_endpoint = [&](model::node_t const* pe_node, int expected_pl_pin) {
        bool ok{};
        for(auto const& net : r.nets)
        {
            if(net.pe_node != pe_node) { continue; }
            for(auto const& ep : net.endpoints)
            {
                if(ep.element_identifier == pl_fa_id && ep.pin == expected_pl_pin)
                {
                    ok = true;
                    break;
                }
            }
        }
        assert(ok);
    };

    // physicsLab(Full Adder) pin order:
    //   outputs: 0=S, 1=Cout
    //   inputs : 2=B, 3=Cin, 4=A
    assert_has_endpoint(&na, 4);     // A -> pin4
    assert_has_endpoint(&nb, 2);     // B -> pin2
    assert_has_endpoint(&ncin, 3);   // Cin -> pin3
    assert_has_endpoint(&ns, 0);     // S -> pin0
    assert_has_endpoint(&ncout, 1);  // Cout -> pin1

    return 0;
}

