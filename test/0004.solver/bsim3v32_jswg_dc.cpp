#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static std::size_t find_attr(::phy_engine::model::model_base* m, char const* name) noexcept
{
    auto const ascii_ieq = [](char a, char b) noexcept
    {
        auto const ua = static_cast<unsigned char>(a);
        auto const ub = static_cast<unsigned char>(b);
        auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
        auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
        return la == lb;
    };

    auto const eq_name = [&](::fast_io::u8string_view n) noexcept
    {
        if(n.empty()) { return false; }
        auto const* p = reinterpret_cast<char const*>(n.data());
        std::size_t i{};
        for(; name[i] != '\0'; ++i)
        {
            if(i >= n.size()) { return false; }
            if(!ascii_ieq(p[i], name[i])) { return false; }
        }
        return i == n.size();
    };

    constexpr std::size_t kMaxScan{512};
    for(std::size_t idx{}; idx < kMaxScan; ++idx)
    {
        auto const n = m->ptr->get_attribute_name(idx);
        if(!eq_name(n)) { continue; }
        return idx;
    }
    return SIZE_MAX;
}

static double run_case(double jswg_a_per_m) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Bulk bias = 0.7V to forward-bias the B-D diode for NMOS (anode at B, cathode at D).
    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.7});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    // Gate at 0V so channel is off.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Drain load resistor to ground.
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_drain);
    add_to_node(nl, *rload, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, n_bulk);

    auto const idx_jswg = find_attr(m1, "jswg");
    auto const idx_is = find_attr(m1, "diode_Is");
    auto const idx_w = find_attr(m1, "W");
    auto const idx_js = find_attr(m1, "js");
    auto const idx_jsw = find_attr(m1, "jsw");
    if(idx_jswg == SIZE_MAX || idx_is == SIZE_MAX || idx_w == SIZE_MAX || idx_js == SIZE_MAX || idx_jsw == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Isolate jswg contribution.
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-14}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_js, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jsw, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_w, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jswg, {.d{jswg_a_per_m}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    // Larger jswg => larger effective Is => more forward diode current => drain rises closer to bulk.
    double const vd0 = run_case(0.0);
    double const vd1 = run_case(1e-4);

    if(!std::isfinite(vd0) || !std::isfinite(vd1)) { return 1; }
    if(!(vd1 > vd0 + 1e-3)) { return 2; }
    return 0;
}

