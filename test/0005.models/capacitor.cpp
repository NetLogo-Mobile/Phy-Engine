#include <complex>
#include <fast_io/fast_io.h>
#include <phy_engine/phy_engine.h>

int main()
{
    {
        {
            ::phy_engine::circult c{};
            c.set_analyze_type(::phy_engine::analyze_type::DC);

            auto& nl{c.get_netlist()};

            auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
            auto [C1, C1_pos]{add_model(nl, ::phy_engine::model::capacitor{.m_kZimag{1e-5}})};
            auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};

            auto& node1{create_node(nl)};
            add_to_node(nl, *R1, 1, node1);
            add_to_node(nl, *C1, 0, node1);
            auto& node2{create_node(nl)};
            add_to_node(nl, *VDC, 0, node2);
            add_to_node(nl, *R1, 0, node2);
            auto& node3{nl.ground_node};  // ground
            add_to_node(nl, *VDC, 1, node3);
            add_to_node(nl, *C1, 1, node3);

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

                auto const c1_pin_view{C1->ptr->generate_pin_view()};
                auto const c1_V1{c1_pin_view.pins[0].nodes->node_information.an.voltage};
                auto const c1_V2{c1_pin_view.pins[1].nodes->node_information.an.voltage};

                ::fast_io::io::println("C1: VA=", ::fast_io::mnp::fixed(c1_V1), "V, VB=", ::fast_io::mnp::fixed(c1_V2), "V, V=", c1_V1 - c1_V2);

                auto const VDC_branch_view{VDC->ptr->generate_branch_view()};
                ::fast_io::io::print("VDC: current=",
                                     ::fast_io::mnp::fixed(-VDC_branch_view.branches[0].current),
                                     "A, U=",
                                     VDC->ptr->get_attribute(0).d,
                                     "V\n");
                ::fast_io::io::print("=======================\n");
            }
        }

        {
            ::phy_engine::circult c{};
            c.set_analyze_type(::phy_engine::analyze_type::AC);

            auto& nl{c.get_netlist()};

            auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
            auto [C1, C1_pos]{add_model(nl, ::phy_engine::model::capacitor{.m_kZimag{1e-5}})};
            auto [VAC, VAC_pos]{add_model(nl, ::phy_engine::model::VAC{.m_Vp = 3.0})};

            VAC->ptr->set_attribute(1, {.d=50, .type=::phy_engine::model::variant_type::d});
            VAC->ptr->set_attribute(2, {.d=15, .type=::phy_engine::model::variant_type::d});

            auto& node1{create_node(nl)};
            add_to_node(nl, *R1, 1, node1);
            add_to_node(nl, *C1, 0, node1);
            auto& node2{create_node(nl)};
            add_to_node(nl, *VAC, 0, node2);
            add_to_node(nl, *R1, 0, node2);
            auto& node3{nl.ground_node};  // ground
            add_to_node(nl, *VAC, 1, node3);
            add_to_node(nl, *C1, 1, node3);

            if(!c.analyze()) { ::fast_io::io::perr("analyze error\n"); }
            else
            {
                auto const r1_pin_view{R1->ptr->generate_pin_view()};
                auto const r1_V1{r1_pin_view.pins[0].nodes->node_information.an.voltage};
                auto const r1_V2{r1_pin_view.pins[1].nodes->node_information.an.voltage};

                auto const r1_R{R1->ptr->get_attribute(0).d};
                ::fast_io::io::println("R1: VA=",
                                       ::fast_io::mnp::fixed(r1_V1),
                                       "V, VB=",
                                       ::fast_io::mnp::fixed(r1_V2),
                                       "V, V=",
                                       r1_V1 - r1_V2,
                                       "V, R=",
                                       r1_R,
                                       "ohm, I=",
                                       (r1_V1 - r1_V2) / r1_R);

                auto const c1_pin_view{C1->ptr->generate_pin_view()};
                auto const c1_V1{c1_pin_view.pins[0].nodes->node_information.an.voltage};
                auto const c1_V2{c1_pin_view.pins[1].nodes->node_information.an.voltage};

                ::fast_io::io::println("C1: VA=", ::fast_io::mnp::fixed(c1_V1), "V, VB=", ::fast_io::mnp::fixed(c1_V2), "V, V=", c1_V1 - c1_V2);

                auto const VAC_branch_view{VAC->ptr->generate_branch_view()};
                ::fast_io::io::println("VAC: current=",
                                       ::fast_io::mnp::fixed(-VAC_branch_view.branches[0].current),
                                       "A, U=",
                                       VAC->ptr->get_attribute(0).d,
                                       "V, freq=",
                                       VAC->ptr->get_attribute(1).d,
                                       ", phase=",
                                       VAC->ptr->get_attribute(2).d);
                ::fast_io::io::print("=======================\n");
            }
        }
    }
}
