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

static double run_case(double rgate_mod) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Gate driven by VDC so we can measure its branch current.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, nl.ground_node);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_aigb = find_attr(m1, "aigb");
    auto const idx_bigb = find_attr(m1, "bigb");
    auto const idx_cigb = find_attr(m1, "cigb");
    auto const idx_eigb = find_attr(m1, "eigb");
    auto const idx_rg = find_attr(m1, "rg");
    auto const idx_rgatemod = find_attr(m1, "rgateMod");
    if(idx_kp == SIZE_MAX || idx_aigb == SIZE_MAX || idx_bigb == SIZE_MAX || idx_cigb == SIZE_MAX || idx_eigb == SIZE_MAX || idx_rg == SIZE_MAX ||
       idx_rgatemod == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_aigb, {.d{1e9}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_bigb, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cigb, {.d{0.1}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_eigb, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_rg, {.d{1e6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_rgatemod, {.d{rgate_mod}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vg_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_no_rg = run_case(0.0);
    double const i_with_rg = run_case(1.0);
    if(!std::isfinite(i_no_rg) || !std::isfinite(i_with_rg)) { return 1; }

    if(!(i_no_rg > 1e-6)) { return 2; }
    if(!(i_with_rg < i_no_rg * 1e-2)) { return 3; }
    return 0;
}

