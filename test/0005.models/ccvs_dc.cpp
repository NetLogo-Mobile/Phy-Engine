#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/CCVS.h>
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
    constexpr double r_ccvs = 2000.0;  // transresistance
    constexpr double rload = 1000.0;

    auto [v, v_pos]{add_model(nl, ::phy_engine::model::VDC{.V = vin})};
    auto [rm, rm_pos]{add_model(nl, ::phy_engine::model::resistance{.r = rin})};
    auto [h, h_pos]{add_model(nl, ::phy_engine::model::CCVS{.m_r = r_ccvs})};
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

    // CCVS control branch is a 0V source between P-Q (measures current from P->Q)
    add_to_node(nl, *h, 2, node_ctrl); // P
    add_to_node(nl, *h, 3, gnd);      // Q

    // CCVS output is a dependent voltage source between S-T controlled by that branch current
    add_to_node(nl, *h, 0, node_out); // S
    add_to_node(nl, *h, 1, gnd);      // T

    // Load to ground
    add_to_node(nl, *rl, 0, node_out);
    add_to_node(nl, *rl, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("ccvs_dc: analyze failed\n");
        return 1;
    }

    auto const bv{h->ptr->generate_branch_view()};
    if(bv.size != 2)
    {
        ::fast_io::io::perr("ccvs_dc: unexpected branch view size\n");
        return 1;
    }

    double const ictrl{bv.branches[1].current.real()};
    double const ictrl_exp{vin / rin};
    double const vout{node_out.node_information.an.voltage.real()};
    double const vexp{r_ccvs * ictrl_exp};

    if(!near(ictrl, ictrl_exp, 1e-12))
    {
        ::fast_io::io::perr("ccvs_dc: ictrl mismatch ictrl=", ictrl, " exp=", ictrl_exp, "\n");
        return 1;
    }
    if(!near(vout, vexp, 1e-9))
    {
        ::fast_io::io::perr("ccvs_dc: vout mismatch vout=", vout, " vexp=", vexp, "\n");
        return 1;
    }

    return 0;
}

