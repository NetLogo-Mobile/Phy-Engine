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

static double run_case(double temp_c) noexcept
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
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, n_source);
    add_to_node(nl, *m1, 3, n_bulk);

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_temp = find_attr(m1, "Temp");
    auto const idx_tnom = find_attr(m1, "tnom");
    auto const idx_aigb = find_attr(m1, "aigb");
    auto const idx_bigb = find_attr(m1, "bigb");
    auto const idx_cigb = find_attr(m1, "cigb");
    auto const idx_eigb = find_attr(m1, "eigb");
    if(idx_kp == SIZE_MAX || idx_temp == SIZE_MAX || idx_tnom == SIZE_MAX || idx_aigb == SIZE_MAX || idx_bigb == SIZE_MAX || idx_cigb == SIZE_MAX ||
       idx_eigb == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Disable channel so current is dominated by leakage path.
    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    // Anchor to a known nominal point for temperature scaling.
    (void)m1->ptr->set_attribute(idx_tnom, {.d{27.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_temp, {.d{temp_c}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_aigb, {.d{1e9}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_bigb, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cigb, {.d{0.1}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_eigb, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vg_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_nom = run_case(27.0);
    double const i_hot = run_case(127.0);
    if(!std::isfinite(i_nom) || !std::isfinite(i_hot)) { return 1; }

    // With barrier scaling enabled, leakage must increase with temperature.
    if(!(i_nom > 1e-12)) { return 2; }
    if(!(i_hot > 1.2 * i_nom)) { return 3; }
    return 0;
}

