#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/CCCS.h>
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

    constexpr double vin = 5.0;
    constexpr double rin = 1000.0;
    constexpr double alpha = 3.0;
    constexpr double rload = 1000.0;

    auto [v, v_pos]{add_model(nl, ::phy_engine::model::VDC{.V = vin})};
    auto [rm, rm_pos]{add_model(nl, ::phy_engine::model::resistance{.r = rin})};
    auto [f, f_pos]{add_model(nl, ::phy_engine::model::CCCS{.m_alpha = alpha})};
    auto [rl, rl_pos]{add_model(nl, ::phy_engine::model::resistance{.r = rload})};

    auto& node_src{create_node(nl)};
    auto& node_ctrl{create_node(nl)};
    auto& node_out{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Drive: node_src - gnd = vin
    add_to_node(nl, *v, 0, node_src);
    add_to_node(nl, *v, 1, gnd);

    // Rin from node_src to node_ctrl
    add_to_node(nl, *rm, 0, node_src);
    add_to_node(nl, *rm, 1, node_ctrl);

    // CCCS control branch is a 0V source between P-Q (measures current from P->Q)
    add_to_node(nl, *f, 2, node_ctrl); // P
    add_to_node(nl, *f, 3, gnd);      // Q

    // CCCS output is a current source between S-T controlled by that branch current
    add_to_node(nl, *f, 0, node_out); // S
    add_to_node(nl, *f, 1, gnd);      // T

    // Load to ground
    add_to_node(nl, *rl, 0, node_out);
    add_to_node(nl, *rl, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("cccs_dc: analyze failed\n");
        return 1;
    }

    auto const bv{f->ptr->generate_branch_view()};
    if(bv.size != 1)
    {
        ::fast_io::io::perr("cccs_dc: unexpected branch view size\n");
        return 1;
    }

    double const ictrl{bv.branches[0].current.real()};
    double const ictrl_exp{vin / rin};

    double const vout{node_out.node_information.an.voltage.real()};
    double const vexp{-alpha * ictrl_exp * rload};

    if(!near(ictrl, ictrl_exp, 1e-12))
    {
        ::fast_io::io::perr("cccs_dc: ictrl mismatch ictrl=", ictrl, " exp=", ictrl_exp, "\n");
        return 1;
    }
    if(!near(vout, vexp, 1e-9))
    {
        ::fast_io::io::perr("cccs_dc: vout mismatch vout=", vout, " vexp=", vexp, "\n");
        return 1;
    }

    return 0;
}

