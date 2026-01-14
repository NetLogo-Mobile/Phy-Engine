#include <phy_engine/model/models/digital/combinational/mul2.h>
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

    auto& na0 = netlist::create_node(nl);
    auto& na1 = netlist::create_node(nl);
    auto& nb0 = netlist::create_node(nl);
    auto& nb1 = netlist::create_node(nl);
    auto& np0 = netlist::create_node(nl);
    auto& np1 = netlist::create_node(nl);
    auto& np2 = netlist::create_node(nl);
    auto& np3 = netlist::create_node(nl);

    auto [in_a0, in_a0_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_a0_pos;
    auto [in_a1, in_a1_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_a1_pos;
    auto [in_b0, in_b0_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_b0_pos;
    auto [in_b1, in_b1_pos] = netlist::add_model(nl, model::INPUT{});
    (void)in_b1_pos;

    auto [out_p0, out_p0_pos] = netlist::add_model(nl, model::OUTPUT{});
    (void)out_p0_pos;
    auto [out_p1, out_p1_pos] = netlist::add_model(nl, model::OUTPUT{});
    (void)out_p1_pos;
    auto [out_p2, out_p2_pos] = netlist::add_model(nl, model::OUTPUT{});
    (void)out_p2_pos;
    auto [out_p3, out_p3_pos] = netlist::add_model(nl, model::OUTPUT{});
    (void)out_p3_pos;

    auto [mul2, mul2_pos] = netlist::add_model(nl, model::MUL2{});
    (void)mul2_pos;

    // PE(MUL2): a0, a1, b0, b1, p0, p1, p2, p3
    assert(netlist::add_to_node(nl, *in_a0, 0, na0));
    assert(netlist::add_to_node(nl, *in_a1, 0, na1));
    assert(netlist::add_to_node(nl, *in_b0, 0, nb0));
    assert(netlist::add_to_node(nl, *in_b1, 0, nb1));

    assert(netlist::add_to_node(nl, *out_p0, 0, np0));
    assert(netlist::add_to_node(nl, *out_p1, 0, np1));
    assert(netlist::add_to_node(nl, *out_p2, 0, np2));
    assert(netlist::add_to_node(nl, *out_p3, 0, np3));

    assert(netlist::add_to_node(nl, *mul2, 0, na0));
    assert(netlist::add_to_node(nl, *mul2, 1, na1));
    assert(netlist::add_to_node(nl, *mul2, 2, nb0));
    assert(netlist::add_to_node(nl, *mul2, 3, nb1));
    assert(netlist::add_to_node(nl, *mul2, 4, np0));
    assert(netlist::add_to_node(nl, *mul2, 5, np1));
    assert(netlist::add_to_node(nl, *mul2, 6, np2));
    assert(netlist::add_to_node(nl, *mul2, 7, np3));

    auto r = pe_to_pl::convert(nl);

    // Find PL Multiplier element identifier.
    std::string pl_mul_id{};
    for(auto const& e : r.ex.elements())
    {
        if(e.data().value("ModelID", "") == "Multiplier")
        {
            pl_mul_id = e.identifier();
            break;
        }
    }
    assert(!pl_mul_id.empty());

    auto assert_has_endpoint = [&](model::node_t const* pe_node, int expected_pl_pin) {
        bool ok{};
        for(auto const& net : r.nets)
        {
            if(net.pe_node != pe_node) { continue; }
            for(auto const& ep : net.endpoints)
            {
                if(ep.element_identifier == pl_mul_id && ep.pin == expected_pl_pin)
                {
                    ok = true;
                    break;
                }
            }
        }
        assert(ok);
    };

    // PL(Multiplier) pin order (PhysicsLab):
    //   outputs: 0=Q1, 1=Q2, 2=Q3, 3=Q4
    //   inputs : 4=B1, 5=B2, 6=A1, 7=A2
    //
    // Wrapper fix expectation:
    //   A1<->A2, B1<->B2, Q1<->Q4, Q2<->Q3.
    assert_has_endpoint(&na0, 7);
    assert_has_endpoint(&na1, 6);
    assert_has_endpoint(&nb0, 5);
    assert_has_endpoint(&nb1, 4);

    assert_has_endpoint(&np0, 3);
    assert_has_endpoint(&np1, 2);
    assert_has_endpoint(&np2, 1);
    assert_has_endpoint(&np3, 0);

    return 0;
}

