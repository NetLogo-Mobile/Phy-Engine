#include <complex>
#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/circuits/circuit.h>

int main()
{
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::AC);

        ::phy_engine::model::resistance r{.r = 10.0};
        ::phy_engine::model::VAC vac{.m_Vp{3.0}, .m_omega{50.0 * (2.0 * ::std::numbers::pi)}};
        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, r)};
        auto [R2, R2_pos]{add_model(nl, ::std::move(r))};
        auto [VDC, VDC_pos]{add_model(nl, ::std::move(vac))};
        auto& node1{create_node(nl)};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *R2, 0, node1);
        auto& node2{create_node(nl)};
        add_to_node(nl, *VDC, 0, node2);
        add_to_node(nl, *R1, 0, node2);
        auto& node3{nl.ground_node};  // ground
        add_to_node(nl, *VDC, 1, node3);
        add_to_node(nl, *R2, 1, node3);

        c.analyze();

        auto const r1_pin_view{R1->ptr->generate_pin_view()};
        ::fast_io::io::println("R1: VA=",
                               ::fast_io::mnp::fixed(r1_pin_view.pins[0].nodes->node_information.an.voltage),
                               ", VB=",
                               ::fast_io::mnp::fixed(r1_pin_view.pins[1].nodes->node_information.an.voltage));

        auto const r2_pin_view{R2->ptr->generate_pin_view()};
        ::fast_io::io::println("R2: VA=",
                               ::fast_io::mnp::fixed(r2_pin_view.pins[0].nodes->node_information.an.voltage),
                               ", VB=",
                               ::fast_io::mnp::fixed(r2_pin_view.pins[1].nodes->node_information.an.voltage));

        auto const VDC_branch_view{VDC->ptr->generate_branch_view()};
        ::fast_io::io::println("VDC: current=", ::fast_io::mnp::fixed(-VDC_branch_view.branches[0].current));
    }
}
