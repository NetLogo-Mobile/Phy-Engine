#include <complex>
#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/controller/switch.h>
#include <phy_engine/circuits/circuit.h>

int main()
{
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);

        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
        auto [SPS, SPS_pos]{add_model(nl, ::phy_engine::model::single_pole_switch{.cut_through{false}})};
        auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};

        auto& node1{create_node(nl)};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *SPS, 0, node1);
        auto& node2{nl.ground_node};
        add_to_node(nl, *VDC, 0, node2);
        add_to_node(nl, *R1, 0, node2);
        auto& node3{create_node(nl)};  // ground
        add_to_node(nl, *VDC, 1, node3);
        add_to_node(nl, *SPS, 1, node3);

        {
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

                auto const r2_pin_view{SPS->ptr->generate_pin_view()};
                ::fast_io::io::println("Switch: Cut Through=", ::fast_io::mnp::boolalpha(SPS->ptr->get_attribute(0).boolean));

                auto const VDC_branch_view{VDC->ptr->generate_branch_view()};
                ::fast_io::io::println("VDC: current=", ::fast_io::mnp::fixed(-VDC_branch_view.branches[0].current));
                ::fast_io::io::print("=======================\n");
            }
        }

        // set cut through
        SPS->ptr->set_attribute(0, {.boolean{true}, .type{::phy_engine::model::variant_type::boolean}});

        {
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

                auto const r2_pin_view{SPS->ptr->generate_pin_view()};
                ::fast_io::io::println("Switch: Cut Through=", ::fast_io::mnp::boolalpha(SPS->ptr->get_attribute(0).boolean));

                auto const VDC_branch_view{VDC->ptr->generate_branch_view()};
                ::fast_io::io::println("VDC: current=", ::fast_io::mnp::fixed(-VDC_branch_view.branches[0].current));
                ::fast_io::io::print("=======================\n");
            }
        }
    }
}
