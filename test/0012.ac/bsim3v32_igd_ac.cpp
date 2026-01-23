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

static double run_case(bool enable_igd) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e3};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Gate drive is a DC + AC series source.
    auto [vbias_src, vbias_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vbias_src, 0, n_bias);
    add_to_node(nl, *vbias_src, 1, nl.ground_node);

    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_gate);
    add_to_node(nl, *vac_src, 1, n_bias);

    // D is grounded (so Vgd is driven by VAC), S/B grounded.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, nl.ground_node);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_aigd = find_attr(m1, "aigd");
    auto const idx_bigd = find_attr(m1, "bigd");
    auto const idx_cigd = find_attr(m1, "cigd");
    auto const idx_eigd = find_attr(m1, "eigd");
    if(idx_kp == SIZE_MAX || idx_aigd == SIZE_MAX || idx_bigd == SIZE_MAX || idx_cigd == SIZE_MAX || idx_eigd == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    if(enable_igd)
    {
        (void)m1->ptr->set_attribute(idx_aigd, {.d{1e9}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_bigd, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_cigd, {.d{0.1}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_eigd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    }
    else
    {
        (void)m1->ptr->set_attribute(idx_aigd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    }

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_off = run_case(false);
    double const i_on = run_case(true);
    if(!std::isfinite(i_off) || !std::isfinite(i_on)) { return 1; }

    if(!(i_off < 1e-12)) { return 2; }
    if(!(i_on > 1e-9)) { return 3; }
    return 0;
}

