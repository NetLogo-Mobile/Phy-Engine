#include <cmath>
#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/coupled_inductors.h>
#include <phy_engine/circuits/circuit.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-5;
    c.get_analyze_setting().tr.t_stop = 1e-4;  // about 1 time-constant for L=1e-3, R=10

    auto& nl{c.get_netlist()};

    auto [vsrc, vsrc_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 1.0})};
    auto [r1, r1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
    auto [kl, kl_pos]{add_model(nl, ::phy_engine::model::coupled_inductors{.L1 = 1e-3, .L2 = 1e-3, .k = 0.0})};
    auto [r2, r2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};

    auto& node_p{create_node(nl)};
    auto& node_r{create_node(nl)};
    auto& node_sr{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Primary loop: Vsrc -> L1 -> R1 -> gnd
    add_to_node(nl, *vsrc, 0, node_p);
    add_to_node(nl, *vsrc, 1, gnd);

    add_to_node(nl, *kl, 0, node_p);  // P1
    add_to_node(nl, *kl, 1, node_r);  // P2
    add_to_node(nl, *r1, 0, node_r);
    add_to_node(nl, *r1, 1, gnd);

    // Secondary loop (k=0 => should remain ~0 current): L2 between gnd and node_sr, R2 to gnd
    add_to_node(nl, *kl, 2, gnd);      // S1
    add_to_node(nl, *kl, 3, node_sr);  // S2
    add_to_node(nl, *r2, 0, node_sr);
    add_to_node(nl, *r2, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("coupled_inductors_TR: analyze failed\n");
        return 1;
    }

    auto const kl_branch_view{kl->ptr->generate_branch_view()};
    double const i1{kl_branch_view.branches[0].current.real()};
    double const i2{kl_branch_view.branches[1].current.real()};

    // If the inductor were incorrectly treated as an ideal short in TR, current would jump near 0.1A immediately.
    if(!(i1 < 0.095))
    {
        ::fast_io::io::perr("coupled_inductors_TR: primary current too large (likely stamped as short)\n");
        return 1;
    }

    if(!(std::abs(i2) < 1e-6))
    {
        ::fast_io::io::perr("coupled_inductors_TR: unexpected secondary current with k=0\n");
        return 1;
    }

    return 0;
}

