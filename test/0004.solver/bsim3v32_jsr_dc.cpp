#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
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

static double run_case(bool enable_jsr) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl = c.get_netlist();

    // Force drain to -0.7V to forward-bias the drain-body diode (NMOS, bulk at 0V).
    auto [vd_src, vd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -0.7});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vd_src, 0, n_drain);
    add_to_node(nl, *vd_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, nl.ground_node);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_diode_is = find_attr(m1, "diode_Is");
    auto const idx_diode_isr = find_attr(m1, "diode_Isr");
    auto const idx_diode_nr = find_attr(m1, "diode_Nr");
    auto const idx_jsr = find_attr(m1, "jsr");
    auto const idx_ad = find_attr(m1, "ad");
    if(idx_kp == SIZE_MAX || idx_diode_is == SIZE_MAX || idx_diode_isr == SIZE_MAX || idx_diode_nr == SIZE_MAX || idx_jsr == SIZE_MAX || idx_ad == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Suppress channel current and diffusion Is so the recombination current dominates.
    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_diode_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_diode_isr, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_diode_nr, {.d{2.0}, .type{::phy_engine::model::variant_type::d}});

    // Use a non-zero drain junction area so jsr is observable.
    (void)m1->ptr->set_attribute(idx_ad, {.d{1e-12}, .type{::phy_engine::model::variant_type::d}});

    // Geometry-based recombination current density (A/m^2).
    // With ad=1e-12 m^2 => Isr ~ 1e-9 A at TNOM when enabled.
    (void)m1->ptr->set_attribute(idx_jsr, {.d{enable_jsr ? 1e3 : 0.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    auto const bv = vd_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_off = run_case(false);
    double const i_on = run_case(true);
    if(!std::isfinite(i_off) || !std::isfinite(i_on)) { return 1; }

    // Enabling jsr must significantly increase diode current.
    if(!(i_off < 1e-9)) { return 2; }
    if(!(i_on > 1e-6)) { return 3; }
    if(!(i_on > 1e3 * i_off)) { return 4; }
    return 0;
}

