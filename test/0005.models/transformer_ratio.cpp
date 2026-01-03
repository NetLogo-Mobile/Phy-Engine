#include <cmath>
#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/transformer.h>
#include <phy_engine/circuits/circuit.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl{c.get_netlist()};

    auto [vsrc, vsrc_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 4.0})};
    auto [tx, tx_pos]{add_model(nl, ::phy_engine::model::transformer{.n = 2.0})};
    auto [rload, rload_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 100.0})};

    auto& node_p{create_node(nl)};
    auto& node_s{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Vsrc: node_p - gnd = 4V
    add_to_node(nl, *vsrc, 0, node_p);
    add_to_node(nl, *vsrc, 1, gnd);

    // Transformer: primary node_p-gnd, secondary node_s-gnd
    add_to_node(nl, *tx, 0, node_p);  // P
    add_to_node(nl, *tx, 1, gnd);     // Q
    add_to_node(nl, *tx, 2, node_s);  // S
    add_to_node(nl, *tx, 3, gnd);     // T

    // Load on secondary
    add_to_node(nl, *rload, 0, node_s);
    add_to_node(nl, *rload, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("transformer_ratio: analyze failed\n");
        return 1;
    }

    double const vp{node_p.node_information.an.voltage.real()};
    double const vs{node_s.node_information.an.voltage.real()};
    if(!(std::abs(vp - 4.0) < 1e-6))
    {
        ::fast_io::io::perr("transformer_ratio: unexpected primary voltage\n");
        return 1;
    }
    if(!(std::abs(vs - 2.0) < 1e-3))
    {
        ::fast_io::io::perr("transformer_ratio: unexpected secondary voltage\n");
        return 1;
    }

    auto const tx_branch_view{tx->ptr->generate_branch_view()};
    double const ip{tx_branch_view.branches[0].current.real()};
    double const is{tx_branch_view.branches[1].current.real()};

    double const expected_load_current{vs / 100.0};
    if(!(std::abs(std::abs(is) - expected_load_current) < 5e-3))
    {
        ::fast_io::io::perr("transformer_ratio: unexpected secondary current magnitude\n");
        return 1;
    }

    // Dot convention used by the model: Is + n*Ip = 0
    if(!(std::abs(is + 2.0 * ip) < 5e-3))
    {
        ::fast_io::io::perr("transformer_ratio: current constraint violated\n");
        return 1;
    }

    return 0;
}

