#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/capacitor.h>
#include <phy_engine/circuits/circuit.h>

int main()
{
    {
        ::phy_engine::circult c{};
        ::phy_engine::model::resistance r{.r = 0.5};
        ::phy_engine::model::capacitor cap{.m_kZimag = 0.001};
        auto& nl{c.nl};

        auto [model_p, model_pos]{add_model(nl, r)};
        auto [model_p2, model_pos2]{add_model(nl, cap)};
        auto& node{create_node(nl)};
        add_to_node(nl, *model_p, 0, node);
        add_to_node(nl, model_pos2, 0, node);
        auto& node2{create_node(nl)};
        add_to_node(nl, model_pos, 1, node2);
        add_to_node(nl, *model_p2, 1, node2);

        // c.t_step = 0.001;
        // c.at = ::phy_engine::analyze_type::DC;

        // c.prepare();
        // c.solve();
    }
}
