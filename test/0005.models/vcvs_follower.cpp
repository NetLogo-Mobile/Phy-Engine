#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/VCVS.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};

    constexpr double Vin = 1.0;
    constexpr double mu = 1e6;
    constexpr double Rload = 1000.0;

    auto [vin, vin_pos]{add_model(nl, ::phy_engine::model::VDC{.V = Vin})};
    auto [e, e_pos]{add_model(nl, ::phy_engine::model::VCVS{.m_mu = mu})};
    auto [rl, rl_pos]{add_model(nl, ::phy_engine::model::resistance{.r = Rload})};

    auto& node_in{create_node(nl)};
    auto& node_out{create_node(nl)};
    auto& gnd{nl.ground_node};

    add_to_node(nl, *vin, 0, node_in);
    add_to_node(nl, *vin, 1, gnd);

    // VCVS pins: S,T are output; P,Q are control.
    // Output: node_out - gnd; Control: node_in - node_out.
    add_to_node(nl, *e, 0, node_out); // S
    add_to_node(nl, *e, 1, gnd);      // T
    add_to_node(nl, *e, 2, node_in);  // P
    add_to_node(nl, *e, 3, node_out); // Q

    add_to_node(nl, *rl, 0, node_out);
    add_to_node(nl, *rl, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("vcvs_follower: analyze failed\n");
        return 1;
    }

    double const vout{node_out.node_information.an.voltage.real()};
    double const vexp{mu / (1.0 + mu) * Vin};
    if(!(std::abs(vout - vexp) < 1e-5))
    {
        ::fast_io::io::perr("vcvs_follower: unexpected output voltage vout=", vout, " vexp=", vexp, "\n");
        return 1;
    }

    return 0;
}

