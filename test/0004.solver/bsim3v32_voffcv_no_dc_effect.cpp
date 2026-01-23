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

static double run_case(double voffcv) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // VDD = 3V
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // VG = 1.2V
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 1.2});
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

    auto const idx_w = find_attr(m1, "W");
    auto const idx_l = find_attr(m1, "L");
    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_voff = find_attr(m1, "voff");
    auto const idx_voffcv = find_attr(m1, "voffcv");
    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_kp == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX || idx_voff == SIZE_MAX ||
       idx_voffcv == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_w, {.d{10e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_kp, {.d{2e-4}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vth0, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_phi, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});

    // Keep DC voff fixed; voffcv should affect only C/V, not DC.
    (void)m1->ptr->set_attribute(idx_voff, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_voffcv, {.d{voffcv}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    double const vd0 = run_case(0.0);
    double const vd2 = run_case(0.2);
    if(!std::isfinite(vd0) || !std::isfinite(vd2)) { return 1; }

    // voffcv should not change DC operating point noticeably.
    if(!(std::abs(vd2 - vd0) < 1e-6)) { return 2; }
    return 0;
}

