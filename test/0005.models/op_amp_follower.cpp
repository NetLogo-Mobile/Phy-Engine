#include <cmath>
#include <limits>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/op_amp.h>
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
    auto [oa, oa_pos]{add_model(nl, ::phy_engine::model::op_amp{.mu = mu})};
    auto [rl, rl_pos]{add_model(nl, ::phy_engine::model::resistance{.r = Rload})};

    auto& node_in{create_node(nl)};
    auto& node_out{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Vin: node_in - gnd
    add_to_node(nl, *vin, 0, node_in);
    add_to_node(nl, *vin, 1, gnd);

    // Op-amp as a voltage follower:
    // (+) = node_in, (-) = node_out, OUT+ = node_out, OUT- = gnd.
    add_to_node(nl, *oa, 0, node_in);
    add_to_node(nl, *oa, 1, node_out);
    add_to_node(nl, *oa, 2, node_out);
    add_to_node(nl, *oa, 3, gnd);

    // Load on output.
    add_to_node(nl, *rl, 0, node_out);
    add_to_node(nl, *rl, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("op_amp_follower: analyze failed\n");
        return 1;
    }

    double const vin_v{node_in.node_information.an.voltage.real()};
    double const vout{node_out.node_information.an.voltage.real()};
    auto const mu_vi = oa->ptr->get_attribute(0);
    double mu_read = ::std::numeric_limits<double>::quiet_NaN();
    if(mu_vi.type == ::phy_engine::model::variant_type::d) { mu_read = mu_vi.d; }

    double const vexp{mu / (1.0 + mu) * Vin};
    if(!(std::abs(vout - vexp) < 1e-5))
    {
        ::fast_io::io::perr("op_amp_follower: unexpected output voltage vin=", vin_v, " vout=", vout, " vexp=", vexp, " mu=", mu_read, "\n");
        return 1;
    }

    // Branch current should match load current magnitude.
    auto const bv = oa->ptr->generate_branch_view();
    if(bv.size != 1)
    {
        ::fast_io::io::perr("op_amp_follower: unexpected branch view size\n");
        return 1;
    }
    double const i_branch{bv.branches[0].current.real()};
    double const i_load{vout / Rload};
    if(!(std::abs(std::abs(i_branch) - std::abs(i_load)) < 1e-6))
    {
        ::fast_io::io::perr("op_amp_follower: unexpected branch current\n");
        return 1;
    }

    return 0;
}
