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

static double run_case(double m_mult, double rbsb_ohm) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, nl.ground_node);  // D
    add_to_node(nl, *m1, 1, nl.ground_node);  // G
    add_to_node(nl, *m1, 2, nl.ground_node);  // S
    add_to_node(nl, *m1, 3, nl.ground_node);  // B (must be connected for rbsb to allocate)

    auto const idx_m = find_attr(m1, "m");
    auto const idx_rbsb = find_attr(m1, "rbsb");
    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_is = find_attr(m1, "diode_Is");
    if(idx_m == SIZE_MAX || idx_rbsb == SIZE_MAX || idx_kp == SIZE_MAX || idx_is == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_m, {.d{m_mult}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_rbsb, {.d{rbsb_ohm}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});

    // Force internal node allocation now so we can attach a test source to it.
    auto const iv = m1->ptr->generate_internal_node_view();
    if(iv.size != 1 || iv.nodes == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }

    // Apply a reverse-bias voltage to keep the source-body diode off; current then flows through rbsb only.
    // V(nbs) = -1V wrt bulk(0V) => I ≈ 1 / rbsb_eff.
    auto [vtest, vtest_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -1.0});
    add_to_node(nl, *vtest, 0, *iv.nodes);
    add_to_node(nl, *vtest, 1, nl.ground_node);

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vtest->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    // rbsb scales like other series resistances: rbsb_eff = rbsb / (m*nf)
    // Choose values such that rbsb_eff matches between cases.
    double const i_a = run_case(1.0, 100.0);
    double const i_b = run_case(10.0, 1000.0);
    if(!std::isfinite(i_a) || !std::isfinite(i_b)) { return 1; }

    if(!(i_a > 1e-6 && i_b > 1e-6)) { return 2; }
    double const denom = std::max({i_a, i_b, 1e-24});
    if(!(std::abs(i_a - i_b) / denom < 1e-3)) { return 3; }
    return 0;
}

