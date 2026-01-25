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

static double run_case(bool enable_cjswd) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c.get_analyze_setting()};
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;  // single step: capacitive Norton conductance must affect the solution

    auto& nl = c.get_netlist();

    // VDD = 3V.
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // Drain load resistor from VDD to drain.
    auto [rd, rd_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 10'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rd, 0, n_vdd);
    add_to_node(nl, *rd, 1, n_drain);

    // Gate at 0V (off).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // NMOS: D=drain, G=0V, S=0V, B=0V.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_diode_is = find_attr(m1, "diode_Is");
    auto const idx_pd = find_attr(m1, "pd");
    auto const idx_cjswd = find_attr(m1, "cjswd");
    auto const idx_pb = find_attr(m1, "pb");
    if(idx_kp == SIZE_MAX || idx_diode_is == SIZE_MAX || idx_pd == SIZE_MAX || idx_cjswd == SIZE_MAX || idx_pb == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Suppress channel + junction conduction so only depletion C matters.
    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_diode_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});

    // Enable a sizeable drain-sidewall depletion capacitance using only the per-junction param cjswd.
    (void)m1->ptr->set_attribute(idx_pb, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pd, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});  // perimeter in meters
    (void)m1->ptr->set_attribute(idx_cjswd, {.d{enable_cjswd ? 2e-6 : 0.0}, .type{::phy_engine::model::variant_type::d}});  // F/m

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    double const vd_no = run_case(false);
    double const vd_on = run_case(true);
    if(!std::isfinite(vd_no) || !std::isfinite(vd_on)) { return 1; }

    // With cjswd disabled, the drain is essentially pulled up to VDD in one step.
    if(!(vd_no > 2.8)) { return 2; }
    // With cjswd enabled, the depletion cap must appear in TR (as a Norton conductance), pulling the drain down.
    if(!(vd_on < 1.5)) { return 3; }
    if(!(vd_no - vd_on > 1.0)) { return 4; }
    return 0;
}

