#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/inductor.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    inline bool near(double a, double b, double tol) noexcept { return std::abs(a - b) <= tol; }
}  // namespace

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c.get_analyze_setting()};

    constexpr double vstep = 1.0;
    constexpr double r = 10.0;
    constexpr double L = 1e-5;
    constexpr double tau = L / r;  // 1e-6

    setting.tr.t_step = tau / 100.0;  // 1e-8
    setting.tr.t_stop = tau;          // 1e-6 (100 steps)

    auto& nl{c.get_netlist()};

    auto [v, v_pos]{add_model(nl, ::phy_engine::model::VDC{.V = vstep})};
    auto [rr, rr_pos]{add_model(nl, ::phy_engine::model::resistance{.r = r})};
    auto [ll, ll_pos]{add_model(nl, ::phy_engine::model::inductor{.m_kZimag = L})};

    auto& node_src{create_node(nl)};
    auto& node_mid{create_node(nl)};
    auto& gnd{nl.ground_node};

    // VDC: node_src - gnd = vstep
    add_to_node(nl, *v, 0, node_src);
    add_to_node(nl, *v, 1, gnd);

    // R: node_src -> node_mid
    add_to_node(nl, *rr, 0, node_src);
    add_to_node(nl, *rr, 1, node_mid);

    // L: node_mid -> gnd
    add_to_node(nl, *ll, 0, node_mid);
    add_to_node(nl, *ll, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("rl_step_tr: analyze failed\n");
        return 1;
    }

    auto const bv{ll->ptr->generate_branch_view()};
    if(bv.size != 1)
    {
        ::fast_io::io::perr("rl_step_tr: unexpected branch view size\n");
        return 1;
    }

    double const iL{bv.branches[0].current.real()};
    double const ifinal{vstep / r};
    double const iexp{ifinal * (1.0 - std::exp(-1.0))};  // at t=tau

    if(!near(iL, iexp, 5e-3))
    {
        ::fast_io::io::perr("rl_step_tr: iL mismatch iL=", iL, " iexp=", iexp, "\n");
        return 1;
    }

    return 0;
}

