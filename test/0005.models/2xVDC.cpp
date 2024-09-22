#include <complex>
#include <fast_io/fast_io.h>
#include <phy_engine/phy_engine.h>

int main()
{

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl{c.get_netlist()};

    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 2.0})};
    auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 4.0})};
    auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 8.0})};

    auto [VDC1, VDC1_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 32.0})};
    auto [VDC2, VDC2_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 20.0})};
    
    auto& node1{create_node(nl)};
    auto& node2{create_node(nl)};
    auto& node3{create_node(nl)};
    auto& node4{nl.ground_node}; 
    add_to_node(nl, *VDC1, 0, node4);
    add_to_node(nl, *VDC1, 1, node1);

    add_to_node(nl, *R1, 0, node1);
    add_to_node(nl, *R1, 1, node2);

    add_to_node(nl, *R2, 0, node2);
    add_to_node(nl, *R2, 1, node3);

    add_to_node(nl, *VDC2, 0, node4);
    add_to_node(nl, *VDC2, 1, node3);

    add_to_node(nl, *R3, 0, node2);
    add_to_node(nl, *R3, 1, node4);

    if(!c.analyze()) { ::fast_io::io::perr("analyze error\n"); }
    else
    {
        auto const r1_pin_view{R1->ptr->generate_pin_view()};
        auto const r1_V1{r1_pin_view.pins[0].nodes->node_information.an.voltage};
        auto const r1_V2{r1_pin_view.pins[1].nodes->node_information.an.voltage};

        auto const r1_R{R1->ptr->get_attribute(0).d};
        ::fast_io::io::println("R1: UA=",
                               ::fast_io::mnp::fixed(r1_V1),
                               "V, UB=",
                               ::fast_io::mnp::fixed(r1_V2),
                               "V, U=",
                               r1_V1 - r1_V2,
                               "V, R=",
                               r1_R,
                               "ohm, I=",
                               (r1_V1 - r1_V2) / r1_R);

        auto const VDC1_branch_view{VDC1->ptr->generate_branch_view()};
        ::fast_io::io::print("VDC1: current=", ::fast_io::mnp::fixed(-VDC1_branch_view.branches[0].current), "A, U=", VDC1->ptr->get_attribute(0).d, "V\n");
        ::fast_io::io::print("=======================\n");

        auto const VDC_branch_view{VDC2->ptr->generate_branch_view()};
        ::fast_io::io::print("VDC2: current=", ::fast_io::mnp::fixed(-VDC_branch_view.branches[0].current), "A, U=", VDC2->ptr->get_attribute(0).d, "V\n");
        ::fast_io::io::print("=======================\n");
    }
}
