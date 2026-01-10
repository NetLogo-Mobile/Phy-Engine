#include <cmath>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/controller/relay.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    bool set_vdc(::phy_engine::netlist::netlist& nl, ::phy_engine::netlist::model_pos pos, double v)
    {
        auto* m = get_model(nl, pos);
        if(m == nullptr || m->ptr == nullptr) { return false; }
        ::phy_engine::model::variant vi{};
        vi.d = v;
        vi.type = ::phy_engine::model::variant_type::d;
        return m->ptr->set_attribute(0, vi);
    }
}  // namespace

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};

    constexpr double Vsrc = 1.0;
    constexpr double R = 100.0;

    auto [vsrc, vsrc_pos]{add_model(nl, ::phy_engine::model::VDC{.V = Vsrc})};
    auto [vctrl, vctrl_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 0.0})};
    auto [rload, rload_pos]{add_model(nl, ::phy_engine::model::resistance{.r = R})};

    ::phy_engine::model::relay r{};
    r.Von = 5.0;
    r.Voff = 3.0;
    auto [relay, relay_pos]{add_model(nl, ::std::move(r))};

    auto& node_a{create_node(nl)};
    auto& node_b{create_node(nl)};
    auto& node_ctrl{create_node(nl)};
    auto& gnd{nl.ground_node};

    // Vsrc drives node_a.
    add_to_node(nl, *vsrc, 0, node_a);
    add_to_node(nl, *vsrc, 1, gnd);

    // Load from node_b to ground.
    add_to_node(nl, *rload, 0, node_b);
    add_to_node(nl, *rload, 1, gnd);

    // Control source drives node_ctrl.
    add_to_node(nl, *vctrl, 0, node_ctrl);
    add_to_node(nl, *vctrl, 1, gnd);

    // Relay: coil C+ C-, contact A B
    add_to_node(nl, *relay, 0, node_ctrl); // C+
    add_to_node(nl, *relay, 1, gnd);      // C-
    add_to_node(nl, *relay, 2, node_a);   // A
    add_to_node(nl, *relay, 3, node_b);   // B

    auto check_open = [&]() -> bool
    {
        double const vb{node_b.node_information.an.voltage.real()};
        if(!(std::abs(vb) < 1e-6)) { return false; }
        auto const bv = vsrc->ptr->generate_branch_view();
        // VDC has 1 branch; when open, supply current should be near zero.
        double const isrc = bv.branches[0].current.real();
        return std::abs(isrc) < 1e-9;
    };

    auto check_closed = [&]() -> bool
    {
        double const vb{node_b.node_information.an.voltage.real()};
        if(!(std::abs(vb - Vsrc) < 1e-6)) { return false; }
        double const i_load = vb / R;
        auto const bv = vsrc->ptr->generate_branch_view();
        double const isrc = bv.branches[0].current.real();
        return std::abs(std::abs(isrc) - std::abs(i_load)) < 1e-6;
    };

    // 1) Vctrl = 0V -> open
    if(!c.analyze())
    {
        ::fast_io::io::perr("relay_hysteresis: analyze failed at 0V\n");
        return 1;
    }
    if(!check_open())
    {
        ::fast_io::io::perr("relay_hysteresis: expected open at 0V\n");
        return 1;
    }

    // 2) Raise above Von -> engage/closed
    if(!set_vdc(nl, vctrl_pos, 6.0))
    {
        ::fast_io::io::perr("relay_hysteresis: failed to set control voltage\n");
        return 1;
    }
    if(!c.analyze())
    {
        ::fast_io::io::perr("relay_hysteresis: analyze failed at 6V\n");
        return 1;
    }
    if(!check_closed())
    {
        auto const vb{node_b.node_information.an.voltage.real()};
        auto const isrc{vsrc->ptr->generate_branch_view().branches[0].current.real()};
        auto const engaged_v{relay->ptr->get_attribute(2)};
        bool engaged{};
        if(engaged_v.type == ::phy_engine::model::variant_type::boolean) { engaged = engaged_v.boolean; }
        ::fast_io::io::perr("relay_hysteresis: expected closed at 6V, vb=", vb, " isrc=", isrc, " engaged=", engaged, "\n");
        return 1;
    }

    // 3) Drop below Voff -> disengage/open
    if(!set_vdc(nl, vctrl_pos, 2.0))
    {
        ::fast_io::io::perr("relay_hysteresis: failed to set control voltage\n");
        return 1;
    }
    if(!c.analyze())
    {
        ::fast_io::io::perr("relay_hysteresis: analyze failed at 2V\n");
        return 1;
    }
    if(!check_open())
    {
        ::fast_io::io::perr("relay_hysteresis: expected open at 2V\n");
        return 1;
    }

    return 0;
}
