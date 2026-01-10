#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/VCVS.h>
#include <phy_engine/model/models/linear/resistance.h>
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

    constexpr double Vin = 1.0;
    constexpr double mu = 2.0;
    constexpr double Rload = 1000.0;

    auto [vin, vin_pos]{add_model(nl, ::phy_engine::model::VDC{.V = Vin})};
    auto [e, e_pos]{add_model(nl, ::phy_engine::model::VCVS{.m_mu = mu})};
    auto [rl, rl_pos]{add_model(nl, ::phy_engine::model::resistance{.r = Rload})};

    auto& node_in{create_node(nl)};
    auto& node_out{create_node(nl)};
    auto& gnd{nl.ground_node};

    add_to_node(nl, *vin, 0, node_in);
    add_to_node(nl, *vin, 1, gnd);

    // Output: node_out - gnd; Control: node_in - gnd.
    add_to_node(nl, *e, 0, node_out); // S
    add_to_node(nl, *e, 1, gnd);      // T
    add_to_node(nl, *e, 2, node_in);  // P
    add_to_node(nl, *e, 3, gnd);      // Q

    add_to_node(nl, *rl, 0, node_out);
    add_to_node(nl, *rl, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("vcvs_gain: analyze failed\n");
        return 1;
    }

    double const vin_v{node_in.node_information.an.voltage.real()};
    double const vout{node_out.node_information.an.voltage.real()};
    auto const mu_vi = e->ptr->get_attribute(0);
    double mu_read{};
    if(mu_vi.type == ::phy_engine::model::variant_type::d) { mu_read = mu_vi.d; }
    double const vexp{mu * Vin};
    if(!(std::abs(vout - vexp) < 1e-6))
    {
        auto const bv_vin = vin->ptr->generate_branch_view();
        auto const bv_e = e->ptr->generate_branch_view();
        ::std::size_t vin_k = (bv_vin.size == 1) ? bv_vin.branches[0].index : SIZE_MAX;
        ::std::size_t e_k = (bv_e.size == 1) ? bv_e.branches[0].index : SIZE_MAX;
        ::fast_io::io::perr("vcvs_gain: unexpected output voltage vin=", vin_v,
                            " vout=", vout,
                            " vexp=", vexp,
                            " mu=", mu_read,
                            " node_in_idx=", node_in.node_index,
                            " node_out_idx=", node_out.node_index,
                            " k_vin=", vin_k,
                            " k_e=", e_k,
                            "\n");
        dump_mna(c);
        return 1;
    }

    return 0;
}
