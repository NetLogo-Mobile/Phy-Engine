#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
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

static double run_case(double acm) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Gate driven by 1V AC source, referenced to ground.
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_gate);
    add_to_node(nl, *vac_src, 1, nl.ground_node);

    // NMOS tied off (D,S,B grounded).
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, nl.ground_node);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_w = find_attr(m1, "W");
    auto const idx_l = find_attr(m1, "L");
    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_toxm = find_attr(m1, "toxm");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_capmod = find_attr(m1, "capMod");
    auto const idx_acm = find_attr(m1, "acm");
    auto const idx_cgso = find_attr(m1, "cgso");
    auto const idx_cgdo = find_attr(m1, "cgdo");
    auto const idx_cgbo = find_attr(m1, "cgbo");

    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_tox == SIZE_MAX || idx_toxm == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX ||
       idx_capmod == SIZE_MAX || idx_acm == SIZE_MAX || idx_cgso == SIZE_MAX || idx_cgdo == SIZE_MAX || idx_cgbo == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double const w{1e-6};
    double const l{1e-6};
    double const tox{1e-8};
    (void)m1->ptr->set_attribute(idx_w, {.d{w}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{l}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_tox, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_toxm, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vth0, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_phi, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_capmod, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_acm, {.d{acm}, .type{::phy_engine::model::variant_type::d}});

    // Make overlap contribution measurable; units are F/m.
    (void)m1->ptr->set_attribute(idx_cgso, {.d{1e-7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgdo, {.d{1e-7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgbo, {.d{1e-7}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const iac = bv.branches[0].current;
    return std::abs(iac.imag()) / omega;  // equivalent capacitance seen by the AC source
}

int main()
{
    double const c_acm0 = run_case(0.0);
    double const c_acm1 = run_case(1.0);

    if(!std::isfinite(c_acm0) || !std::isfinite(c_acm1)) { return 1; }
    if(!(c_acm0 > 0.0)) { return 2; }

    // acm toggles only the implementation path (fixed caps vs charge-matrix), so results should match.
    double const rel = std::abs(c_acm1 - c_acm0) / std::max(c_acm0, 1e-30);
    if(!(rel < 1e-3)) { return 3; }
    return 0;
}

