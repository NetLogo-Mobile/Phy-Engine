#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/resistance.h>

int main()
{

    {
        ::phy_engine::netlist::netlist nl{};
        ::phy_engine::model::resistance r{.r = 0.5};

        auto [model_p, model_pos]{add_model(nl, r)};
        auto [model_p2, model_pos2]{add_model(nl, r)};
        auto& node{create_node(nl)};
        add_to_node(nl, model_pos, 0, node);
        add_to_node(nl, model_pos2, 0, node);
        auto& node2{create_node(nl)};
        add_to_node(nl, model_pos, 1, node2);
        add_to_node(nl, model_pos2, 1, node2);
        // delete node
        delete_node(nl, node);
        delete_node(nl, node2);
    }

    {
        ::phy_engine::netlist::netlist nl{};
        ::phy_engine::model::resistance r{.r = 0.5};
        auto [model_p, model_pos]{add_model(nl, r)};
        auto [model_p2, model_pos2]{add_model(nl, r)};
        auto& node{create_node(nl)};
        add_to_node(nl, model_pos, 0, node);
        add_to_node(nl, model_pos2, 0, node);
        auto& node2{create_node(nl)};
        add_to_node(nl, model_pos, 1, node2);
        add_to_node(nl, model_pos2, 1, node2);
        // delete model
        delete_model(nl, model_pos);
        delete_model(nl, model_pos2);
    }
}
