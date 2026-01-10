#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/transformer_center_tap.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    void dump_mna(::phy_engine::circult const& c) noexcept
    {
        auto const& mna{c.mna};
        auto const row_size{mna.node_size + mna.branch_size};
        ::fast_io::io::perr("MNA: node_size=", mna.node_size, " branch_size=", mna.branch_size, " row_size=", row_size, "\n");
        for(::std::size_t r{}; r < row_size; ++r)
        {
            ::fast_io::io::perr("A[", r, "]:");
            for(auto const& [col, v]: mna.A[r])
            {
                ::fast_io::io::perr(" (", col, ")=", v.real());
                if(v.imag() != 0.0) { ::fast_io::io::perr("+j", v.imag()); }
            }
            ::fast_io::io::perr("\n");
        }
        ::fast_io::io::perr("Z:");
        for(auto const& [idx, v]: mna.Z)
        {
            ::fast_io::io::perr(" (", idx, ")=", v.real());
            if(v.imag() != 0.0) { ::fast_io::io::perr("+j", v.imag()); }
        }
        ::fast_io::io::perr("\n");
    }
}  // namespace

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};

    constexpr double Vp = 4.0;
    constexpr double n_total = 2.0;  // Vp / V(S1-S2)
    constexpr double R = 100.0;

    auto [vsrc, vsrc_pos]{add_model(nl, ::phy_engine::model::VDC{.V = Vp})};
    auto [tx, tx_pos]{add_model(nl, ::phy_engine::model::transformer_center_tap{.n_total = n_total})};
    auto [r1, r1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = R})};
    auto [r2, r2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = R})};

    auto& node_p{create_node(nl)};
    auto& node_s1{create_node(nl)};
    auto& node_s2{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Primary excitation: node_p - gnd = Vp
    add_to_node(nl, *vsrc, 0, node_p);
    add_to_node(nl, *vsrc, 1, gnd);

    // Transformer: P-Q, S1-CT, CT-S2; tie Q and CT to ground.
    add_to_node(nl, *tx, 0, node_p);  // P
    add_to_node(nl, *tx, 1, gnd);     // Q
    add_to_node(nl, *tx, 2, node_s1); // S1
    add_to_node(nl, *tx, 3, gnd);     // CT
    add_to_node(nl, *tx, 4, node_s2); // S2

    // Loads on each half-secondary.
    add_to_node(nl, *r1, 0, node_s1);
    add_to_node(nl, *r1, 1, gnd);
    add_to_node(nl, *r2, 0, node_s2);
    add_to_node(nl, *r2, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("transformer_center_tap_ratio: analyze failed\n");
        dump_mna(c);
        return 1;
    }

    double const vp{node_p.node_information.an.voltage.real()};
    double const vs1{node_s1.node_information.an.voltage.real()};
    double const vs2{node_s2.node_information.an.voltage.real()};

    if(!(std::abs(vp - Vp) < 1e-6))
    {
        ::fast_io::io::perr("transformer_center_tap_ratio: unexpected primary voltage\n");
        return 1;
    }

    double const vhalf_exp = Vp / (2.0 * n_total); // V(S1-CT) magnitude
    if(!(std::abs(vs1 - vhalf_exp) < 2e-3))
    {
        ::fast_io::io::perr("transformer_center_tap_ratio: unexpected S1 voltage\n");
        return 1;
    }
    if(!(std::abs(vs2 + vhalf_exp) < 2e-3))
    {
        ::fast_io::io::perr("transformer_center_tap_ratio: unexpected S2 voltage\n");
        return 1;
    }

    // Check the current relation: I_p + (I_h1 + I_h2)/n_half ~= 0 (from the model stamping).
    auto const bv = tx->ptr->generate_branch_view();
    if(bv.size != 3)
    {
        ::fast_io::io::perr("transformer_center_tap_ratio: unexpected branch view size\n");
        return 1;
    }

    double const ip{bv.branches[0].current.real()};
    double const ih1{bv.branches[1].current.real()};
    double const ih2{bv.branches[2].current.real()};
    double const n_half = 2.0 * n_total;

    if(!(std::abs(ip + (ih1 + ih2) / n_half) < 5e-3))
    {
        ::fast_io::io::perr("transformer_center_tap_ratio: current constraint violated\n");
        return 1;
    }

    return 0;
}
