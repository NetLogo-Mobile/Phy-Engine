#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VCCS.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    inline bool near(double a, double b, double tol) noexcept { return std::abs(a - b) <= tol; }
}  // namespace

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};

    constexpr double vctrl = 5.0;
    constexpr double g = 2e-3;        // 2 mS
    constexpr double rload = 1000.0;  // 1k

    auto [v, v_pos]{add_model(nl, ::phy_engine::model::VDC{.V = vctrl})};
    auto [gm, gm_pos]{add_model(nl, ::phy_engine::model::VCCS{.m_g = g})};
    auto [rl, rl_pos]{add_model(nl, ::phy_engine::model::resistance{.r = rload})};

    auto& node_ctrl{create_node(nl)};
    auto& node_out{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Control voltage: node_ctrl - gnd = vctrl
    add_to_node(nl, *v, 0, node_ctrl);
    add_to_node(nl, *v, 1, gnd);

    // VCCS: current from S->T = g*(V(P)-V(Q))
    add_to_node(nl, *gm, 0, node_out);  // S
    add_to_node(nl, *gm, 1, gnd);       // T
    add_to_node(nl, *gm, 2, node_ctrl); // P
    add_to_node(nl, *gm, 3, gnd);       // Q

    // Load to ground
    add_to_node(nl, *rl, 0, node_out);
    add_to_node(nl, *rl, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("vccs_dc: analyze failed\n");
        return 1;
    }

    double const vout{node_out.node_information.an.voltage.real()};
    double const i_load{vout / rload};

    // With only a controlled current sink and a resistor to ground:
    // i_load + g*vctrl = 0 => vout = -g*vctrl*rload
    double const vexp{-g * vctrl * rload};
    double const iexp{-g * vctrl};

    if(!near(vout, vexp, 1e-9))
    {
        ::fast_io::io::perr("vccs_dc: vout mismatch vout=", vout, " vexp=", vexp, "\n");
        return 1;
    }
    if(!near(i_load, iexp, 1e-12))
    {
        ::fast_io::io::perr("vccs_dc: i_load mismatch i=", i_load, " iexp=", iexp, "\n");
        return 1;
    }

    return 0;
}

