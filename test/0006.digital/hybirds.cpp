#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c.get_analyze_setting()};
    setting.tr.t_step = 0.0001;
    setting.tr.t_stop = 0.0001;

    auto& nl{c.get_netlist()};

    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
    auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
    auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 5.0})};
    auto [O1, O1_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};

    auto& node1{create_node(nl)};
    add_to_node(nl, *R1, 1, node1);
    add_to_node(nl, *R2, 0, node1);
    auto& node2{create_node(nl)};
    add_to_node(nl, *VDC, 0, node2);  // +
    add_to_node(nl, *O1, 0, node2);   // +
    add_to_node(nl, *R1, 0, node2);
    auto& node3{nl.ground_node};  // ground
    add_to_node(nl, *VDC, 1, node3);
    add_to_node(nl, *R2, 1, node3);
    for(size_t i = 0; i < 10; i++)
    {
        ::fast_io::io::print("\nSTEP(", i, "):\n");

        if(!c.analyze())
        {
            ::fast_io::io::perr("error\n");
            return -1;
        }
        c.digital_clk();

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

        ::fast_io::io::println("O=", O1->ptr->get_attribute(0).digital);
    }
}
