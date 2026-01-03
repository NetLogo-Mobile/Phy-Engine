#include <cmath>
#include <limits>
#include <fast_io/fast_io.h>
#include <phy_engine/netlist/impl.h>
#include <phy_engine/model/models/controller/switch.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/circuits/circuit.h>

int main()
{
    auto run_case = [](double r_open) -> double {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        c.get_environment().r_open = r_open;

        auto& nl{c.get_netlist()};

        auto [sw, sw_pos]{add_model(nl, ::phy_engine::model::single_pole_switch{.cut_through{false}})};
        auto [rload, rload_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 1000.0})};
        auto [vsrc, vsrc_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 1.0})};

        auto& node_a{create_node(nl)};
        auto& gnd{nl.ground_node};

        // Vsrc: node_a - gnd = 1V
        add_to_node(nl, *vsrc, 0, node_a);
        add_to_node(nl, *vsrc, 1, gnd);

        // Switch (open): node_a -- node_b
        auto& node_b{create_node(nl)};
        add_to_node(nl, *sw, 0, node_a);
        add_to_node(nl, *sw, 1, node_b);

        // Load: node_b -- gnd
        add_to_node(nl, *rload, 0, node_b);
        add_to_node(nl, *rload, 1, gnd);

        if(!c.analyze())
        {
            ::fast_io::io::perr("switch_r_open: analyze failed\n");
            return std::numeric_limits<double>::quiet_NaN();
        }

        // Leakage current through the open switch equals load current magnitude (series path).
        auto const rload_v{node_b.node_information.an.voltage.real()};
        return rload_v / 1000.0;
    };

    double const i1{run_case(1e6)};
    double const i2{run_case(1e9)};

    if(!std::isfinite(i1) || !std::isfinite(i2))
    {
        ::fast_io::io::perr("switch_r_open: non-finite current\n");
        return 1;
    }

    // Larger r_open should reduce leakage current.
    if(!(i2 < i1))
    {
        ::fast_io::io::perr("switch_r_open: r_open has no effect\n");
        return 1;
    }

    // Rough magnitude sanity: i ≈ 1/(r_open + 1000)
    if(!(std::abs(i1 - (1.0 / (1e6 + 1000.0))) < 5e-10))
    {
        ::fast_io::io::perr("switch_r_open: unexpected current for r_open=1e6\n");
        return 1;
    }

    return 0;
}
