#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/capacitor.h>
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
    constexpr double r = 1000.0;
    constexpr double cap = 1e-9;
    constexpr double tau = r * cap;  // 1e-6

    setting.tr.t_step = tau / 100.0;  // 1e-8
    setting.tr.t_stop = tau;          // 1e-6 (100 steps)

    auto& nl{c.get_netlist()};

    auto [v, v_pos]{add_model(nl, ::phy_engine::model::VDC{.V = vstep})};
    auto [rr, rr_pos]{add_model(nl, ::phy_engine::model::resistance{.r = r})};
    auto [cc, cc_pos]{add_model(nl, ::phy_engine::model::capacitor{.m_kZimag = cap})};

    auto& node_src{create_node(nl)};
    auto& node_out{create_node(nl)};
    auto& gnd{nl.ground_node};

    // VDC: node_src - gnd = vstep
    add_to_node(nl, *v, 0, node_src);
    add_to_node(nl, *v, 1, gnd);

    // R: node_src -> node_out
    add_to_node(nl, *rr, 0, node_src);
    add_to_node(nl, *rr, 1, node_out);

    // C: node_out -> gnd
    add_to_node(nl, *cc, 0, node_out);
    add_to_node(nl, *cc, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("rc_step_tr: analyze failed\n");
        return 1;
    }

    double const vout{node_out.node_information.an.voltage.real()};
    double const vexp{vstep * (1.0 - std::exp(-1.0))};  // at t=tau

    if(!near(vout, vexp, 5e-3))
    {
        ::fast_io::io::perr("rc_step_tr: vout mismatch vout=", vout, " vexp=", vexp, "\n");
        return 1;
    }

    return 0;
}

