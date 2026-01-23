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

static double run_case(double lref_m) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // VDD = 3V
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // VG = 3V
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Load resistor from VDD to drain
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_vdd);
    add_to_node(nl, *rload, 1, n_drain);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_l = find_attr(m1, "L");
    auto const idx_lref = find_attr(m1, "lref");
    auto const idx_wref = find_attr(m1, "wref");
    auto const idx_ua = find_attr(m1, "ua");
    auto const idx_lua = find_attr(m1, "lua");
    auto const idx_kp = find_attr(m1, "Kp");
    if(idx_l == SIZE_MAX || idx_lref == SIZE_MAX || idx_wref == SIZE_MAX || idx_ua == SIZE_MAX || idx_lua == SIZE_MAX || idx_kp == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Fix L, vary only lref so that dl = (L - lref) changes and isolates L-scaling effect.
    (void)m1->ptr->set_attribute(idx_l, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_lref, {.d{lref_m}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_wref, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_ua, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    // lua*(L-lref): with lua=0.01 and delta=0.5um => ua_eff=5e-9, which increases mobility degradation.
    (void)m1->ptr->set_attribute(idx_lua, {.d{1e-2}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_kp, {.d{5e-4}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    // Smaller lref => larger dl => larger ua_eff => lower mobility => less current => higher drain voltage.
    double const vd_lref1u = run_case(1e-6);
    double const vd_lref0p5u = run_case(0.5e-6);

    if(!std::isfinite(vd_lref1u) || !std::isfinite(vd_lref0p5u)) { return 1; }
    if(!(vd_lref0p5u > vd_lref1u + 1e-3)) { return 2; }
    return 0;
}

