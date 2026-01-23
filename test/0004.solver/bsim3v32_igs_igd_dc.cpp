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

static double run_case(double vgate, bool use_igs) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Drain/source/bulk at 0V.
    auto [vd_src, vd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vd_src, 0, n_drain);
    add_to_node(nl, *vd_src, 1, nl.ground_node);

    auto [vs_src, vs_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_source = create_node(nl);
    add_to_node(nl, *vs_src, 0, n_source);
    add_to_node(nl, *vs_src, 1, nl.ground_node);

    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    // Gate driven by a VDC source so we can read its branch current (leakage).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = vgate});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, n_source);
    add_to_node(nl, *m1, 3, n_bulk);

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_aigs = find_attr(m1, "aigs");
    auto const idx_bigs = find_attr(m1, "bigs");
    auto const idx_cigs = find_attr(m1, "cigs");
    auto const idx_eigs = find_attr(m1, "eigs");
    auto const idx_aigd = find_attr(m1, "aigd");
    auto const idx_bigd = find_attr(m1, "bigd");
    auto const idx_cigd = find_attr(m1, "cigd");
    auto const idx_eigd = find_attr(m1, "eigd");
    if(idx_kp == SIZE_MAX || idx_aigs == SIZE_MAX || idx_bigs == SIZE_MAX || idx_cigs == SIZE_MAX || idx_eigs == SIZE_MAX ||
       idx_aigd == SIZE_MAX || idx_bigd == SIZE_MAX || idx_cigd == SIZE_MAX || idx_eigd == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    if(use_igs)
    {
        (void)m1->ptr->set_attribute(idx_aigs, {.d{1e9}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_bigs, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_cigs, {.d{0.1}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_eigs, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_aigd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    }
    else
    {
        (void)m1->ptr->set_attribute(idx_aigd, {.d{1e9}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_bigd, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_cigd, {.d{0.1}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_eigd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
        (void)m1->ptr->set_attribute(idx_aigs, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    }

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vg_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_gs = run_case(2.0, true);
    double const i_gd = run_case(2.0, false);
    if(!std::isfinite(i_gs) || !std::isfinite(i_gd)) { return 1; }

    if(!(i_gs > 1e-9)) { return 2; }
    if(!(i_gd > 1e-9)) { return 3; }
    return 0;
}

