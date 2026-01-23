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

static double run_case(double xpart) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{2.0 * 3.14159265358979323846 * 1e6};  // 1 MHz
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Source at 0V (AC-grounded)
    auto [vs_src, vs_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_source = create_node(nl);
    add_to_node(nl, *vs_src, 0, n_source);
    add_to_node(nl, *vs_src, 1, nl.ground_node);

    // Drain at 2V DC (AC-grounded), measure its branch current.
    auto [vd_src, vd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vd_src, 0, n_drain);
    add_to_node(nl, *vd_src, 1, nl.ground_node);

    // Gate: DC bias + AC excitation.
    auto [vg_dc, vgdc_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
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
    add_to_node(nl, *m1, 2, n_source);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_w = find_attr(m1, "W");
    auto const idx_l = find_attr(m1, "L");
    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_toxm = find_attr(m1, "toxm");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_capmod = find_attr(m1, "capMod");
    auto const idx_xpart = find_attr(m1, "xpart");
    auto const idx_cgs = find_attr(m1, "Cgs");
    auto const idx_cgd = find_attr(m1, "Cgd");
    auto const idx_cgb = find_attr(m1, "Cgb");
    auto const idx_cgso = find_attr(m1, "cgso");
    auto const idx_cgdo = find_attr(m1, "cgdo");
    auto const idx_cgbo = find_attr(m1, "cgbo");
    auto const idx_acm = find_attr(m1, "acm");

    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_tox == SIZE_MAX || idx_toxm == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX ||
       idx_capmod == SIZE_MAX || idx_xpart == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double const tox{1e-8};
    (void)m1->ptr->set_attribute(idx_w, {.d{10e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_tox, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_toxm, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vth0, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_phi, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_capmod, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_xpart, {.d{xpart}, .type{::phy_engine::model::variant_type::d}});

    // Remove fixed/overlap caps so the partition effect dominates.
    if(idx_cgs != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgs, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgd != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgb != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgb, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgso != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgso, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgdo != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgdo, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_cgbo != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_cgbo, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }
    if(idx_acm != SIZE_MAX) { (void)m1->ptr->set_attribute(idx_acm, {.d{0.0}, .type{::phy_engine::model::variant_type::d}}); }

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vd_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.imag());
}

int main()
{
    // In saturation, xpart controls how inversion charge is partitioned between drain/source.
    // Expect the drain displacement current due to gate excitation to increase when xpart selects more drain charge.
    double const id_x0 = run_case(0.0);   // 0/100 -> minimal drain partition
    double const id_x1 = run_case(1.0);   // 40/60  -> intermediate
    double const id_x05 = run_case(0.5);  // 50/50  -> more drain charge than 40/60
    if(!std::isfinite(id_x0) || !std::isfinite(id_x1) || !std::isfinite(id_x05)) { return 1; }

    if(!(id_x0 > 0.0)) { return 2; }
    if(!(id_x1 > id_x0 * 1.05)) { return 3; }
    if(!(id_x05 > id_x1 * 1.02)) { return 4; }
    return 0;
}

