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

static double run_case(double m_mult) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Drain held at +3V to force reverse bias and breakdown when BVD is small.
    auto [vd_src, vd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vd_src, 0, n_drain);
    add_to_node(nl, *vd_src, 1, nl.ground_node);

    // Bulk at 0V through a VDC source so we can measure bulk current.
    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    // Gate and source at 0V.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node); // source
    add_to_node(nl, *m1, 3, n_bulk);         // bulk

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_is = find_attr(m1, "diode_Is");
    auto const idx_n = find_attr(m1, "diode_N");
    auto const idx_bvd = find_attr(m1, "bvd");
    auto const idx_ibvd = find_attr(m1, "ibvd");
    auto const idx_m = find_attr(m1, "m");
    if(idx_kp == SIZE_MAX || idx_is == SIZE_MAX || idx_n == SIZE_MAX || idx_bvd == SIZE_MAX || idx_ibvd == SIZE_MAX || idx_m == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Disable channel; focus on the D-B diode breakdown.
    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_m, {.d{m_mult}, .type{::phy_engine::model::variant_type::d}});

    // Set a small breakdown voltage so 3V reverse bias triggers breakdown.
    (void)m1->ptr->set_attribute(idx_bvd, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ibvd, {.d{1e-9}, .type{::phy_engine::model::variant_type::d}});

    // Keep forward Is tiny to make breakdown dominate.
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_n, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vb_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i1 = run_case(1.0);
    double const i10 = run_case(10.0);
    if(!std::isfinite(i1) || !std::isfinite(i10)) { return 1; }

    // Breakdown current should scale approximately with the instance multiplier.
    // Use a loose bound to keep the regression robust.
    if(!(i10 > i1 * 7.0)) { return 2; }
    if(!(i10 < i1 * 13.0)) { return 3; }
    return 0;
}

