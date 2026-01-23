#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
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

static double run_case(double capmod) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{2.0 * 3.14159265358979323846};  // 1 Hz
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Drain at 0V (AC-grounded), source/bulk at 0V.
    auto [vd_src, vd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vd_src, 0, n_drain);
    add_to_node(nl, *vd_src, 1, nl.ground_node);

    // Gate has 1V AC excitation around a DC bias.
    auto [vg_dc, vgdc_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 1.0});
    auto& n_gate_dc = create_node(nl);
    add_to_node(nl, *vg_dc, 0, n_gate_dc);
    add_to_node(nl, *vg_dc, 1, nl.ground_node);

    auto [vg_ac, vgac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_ac, 0, n_gate);
    add_to_node(nl, *vg_ac, 1, n_gate_dc);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_capmod = find_attr(m1, "capMod");
    auto const idx_cgs = find_attr(m1, "Cgs");
    auto const idx_cgd = find_attr(m1, "Cgd");
    auto const idx_cgb = find_attr(m1, "Cgb");
    auto const idx_cgso = find_attr(m1, "cgso");
    auto const idx_cgdo = find_attr(m1, "cgdo");
    auto const idx_cgbo = find_attr(m1, "cgbo");
    if(idx_capmod == SIZE_MAX || idx_cgs == SIZE_MAX || idx_cgd == SIZE_MAX || idx_cgb == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Make fixed caps dominant/observable; if capMod=1 still used C-matrix (old behavior), this would get double-counted
    // because intrinsic caps would also be added from cmat_ac.
    (void)m1->ptr->set_attribute(idx_capmod, {.d{capmod}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgs, {.d{1e-12}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgb, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    if(idx_cgso != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgso, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgdo != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgdo, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgbo != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgbo, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    // Gate current equals displacement current through Cgs at 1Hz: |Ig| = omega*C*Vac
    auto const bv = vg_ac->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.imag());  // imag part is quadrature (j*omega*C)
}

int main()
{
    double const ig_cap0 = run_case(0.0);
    double const ig_cap1 = run_case(1.0);
    double const ig_cap3 = run_case(3.0);
    if(!std::isfinite(ig_cap0) || !std::isfinite(ig_cap1) || !std::isfinite(ig_cap3)) { return 1; }

    // Behavior alignment target (ngspice-like): capMod=0/1/2 use Meyer-style intrinsic C model.
    // Regression target: capMod=1 must behave like capMod=0 here (previously capMod!=0 could wrongly route 1/2 into C-matrix).
    if(!(std::abs(ig_cap1 - ig_cap0) <= 1e-12 + 1e-6 * std::max(ig_cap0, ig_cap1))) { return 2; }
    return 0;
}
