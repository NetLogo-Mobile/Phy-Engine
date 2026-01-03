#include <phy_engine/circuits/floating_subnet/detect.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    ::phy_engine::netlist::netlist nl{};

    // A subnet not connected to ground.
    ::phy_engine::model::resistance r{};
    auto [model_p, model_pos]{add_model(nl, r)};
    auto& n1{create_node(nl)};
    auto& n2{create_node(nl)};
    add_to_node(nl, model_pos, 0, n1);
    add_to_node(nl, model_pos, 1, n2);

    auto floating{::phy_engine::floating_subnet::detect(nl)};
    if(floating.size() != 1) { return 1; }
    if(floating.front_unchecked().size() != 2) { return 2; }

    // Tie the subnet to ground.
    ::phy_engine::model::resistance r_gnd{};
    auto [model_p2, model_pos2]{add_model(nl, r_gnd)};
    add_to_node(nl, model_pos2, 0, n1);
    add_to_node(nl, model_pos2, 1, get_ground_node(nl));

    auto floating2{::phy_engine::floating_subnet::detect(nl)};
    if(!floating2.empty()) { return 3; }

    return 0;
}

