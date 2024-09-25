#include <complex>
#include <fast_io/fast_io.h>
#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c.get_analyze_setting()};

    setting.tr.t_step = 0.0001;
    setting.tr.t_stop = 0.001;

    auto& nl{c.get_netlist()};

    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
    auto [C1, C1_pos]{add_model(nl, ::phy_engine::model::inductor{.m_kZimag{0.1}})};
    auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};

    auto& node1{create_node(nl)};
    add_to_node(nl, *R1, 1, node1);
    add_to_node(nl, *C1, 0, node1);
    auto& node2{create_node(nl)};
    add_to_node(nl, *VDC, 0, node2);
    add_to_node(nl, *R1, 0, node2);
    auto& node3{get_ground_node(nl)};  // ground
    add_to_node(nl, *VDC, 1, node3);
    add_to_node(nl, *C1, 1, node3);

    double start{};
    for(::std::size_t i{}; i < 50; i++)
    {
        ::fast_io::io::print(
            "========================\n" "start = ",
            start,
            "s, stop = ",
            start + 0.001,
            "s, step = 0.0001s\n" "========================\n");
        if(!c.analyze()) { ::fast_io::io::perr("analyze error\n"); }
        else
        {
            auto const r1_pin_view{R1->ptr->generate_pin_view()};
            auto const r1_V1{r1_pin_view.pins[0].nodes->node_information.an.voltage};
            auto const r1_V2{r1_pin_view.pins[1].nodes->node_information.an.voltage};

            auto const r1_R{R1->ptr->get_attribute(0).d};
            ::fast_io::io::println("R1: U=",
                                   ::fast_io::mnp::fixed(r1_V1 - r1_V2),
                                   "V, R=",
                                   ::fast_io::mnp::fixed(r1_R),
                                   "ohm, I=",
                                   ::fast_io::mnp::fixed((r1_V1 - r1_V2) / r1_R));

            auto const c1_pin_view{C1->ptr->generate_pin_view()};
            auto const c1_V1{c1_pin_view.pins[0].nodes->node_information.an.voltage};
            auto const c1_V2{c1_pin_view.pins[1].nodes->node_information.an.voltage};

            ::fast_io::io::print("I1: U=", ::fast_io::mnp::fixed(c1_V1 - c1_V2), "V\n");

            auto const VDC_branch_view{VDC->ptr->generate_branch_view()};
            ::fast_io::io::print("VDC: current=",
                                 ::fast_io::mnp::fixed(-VDC_branch_view.branches[0].current),
                                 "A, U=",
                                 ::fast_io::mnp::fixed(VDC->ptr->get_attribute(0).d),
                                 "V\n");
            ::fast_io::io::print("=======================\n");
        }
        start += 0.001;
    }
}
