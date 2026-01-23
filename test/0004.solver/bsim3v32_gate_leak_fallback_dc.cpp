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

static double run_case(bool enable_igs, bool enable_igd) noexcept
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
    auto const idx_aigs = find_attr(m1, "aigs");
    auto const idx_aigd = find_attr(m1, "aigd");
    auto const idx_bigb = find_attr(m1, "bigb");
    auto const idx_cigb = find_attr(m1, "cigb");
    auto const idx_eigb = find_attr(m1, "eigb");
    auto const idx_bigs = find_attr(m1, "bigs");
    auto const idx_bigd = find_attr(m1, "bigd");
    if(idx_kp == SIZE_MAX || idx_aigs == SIZE_MAX || idx_aigd == SIZE_MAX || idx_bigb == SIZE_MAX || idx_cigb == SIZE_MAX || idx_eigb == SIZE_MAX ||
       idx_bigs == SIZE_MAX || idx_bigd == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    // Provide only the IGB "shape" parameters. Keep IGS/IGD shape params unset -> must fall back to IGB when enabled.
    (void)m1->ptr->set_attribute(idx_bigb, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cigb, {.d{0.1}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_eigb, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    // Explicitly clear IGS/IGD b-parameters to force fallback path.
    (void)m1->ptr->set_attribute(idx_bigs, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_bigd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_aigs, {.d{enable_igs ? 1e9 : 0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_aigd, {.d{enable_igd ? 1e9 : 0.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vg_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_off = run_case(false, false);
    double const i_gs = run_case(true, false);
    double const i_gd = run_case(false, true);
    if(!std::isfinite(i_off) || !std::isfinite(i_gs) || !std::isfinite(i_gd)) { return 1; }

    if(!(i_off < 1e-12)) { return 2; }
    if(!(i_gs > 1e-9)) { return 3; }
    if(!(i_gd > 1e-9)) { return 4; }
    return 0;
}
