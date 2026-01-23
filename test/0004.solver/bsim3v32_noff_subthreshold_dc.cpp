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

static double run_case(double noff) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // VDS=1V, VGS=0.5V (below Vth0=0.7V), subthreshold region.
    auto [vds_src, vds_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 1.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vds_src, 0, n_drain);
    add_to_node(nl, *vds_src, 1, nl.ground_node);

    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.5});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_w = find_attr(m1, "W");
    auto const idx_l = find_attr(m1, "L");
    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_gamma = find_attr(m1, "gamma");
    auto const idx_lambda = find_attr(m1, "lambda");
    auto const idx_pclm = find_attr(m1, "pclm");
    auto const idx_dsub = find_attr(m1, "dsub");
    auto const idx_ua = find_attr(m1, "ua");
    auto const idx_ub = find_attr(m1, "ub");
    auto const idx_uc = find_attr(m1, "uc");
    auto const idx_noff = find_attr(m1, "noff");

    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_kp == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX || idx_gamma == SIZE_MAX ||
       idx_lambda == SIZE_MAX || idx_pclm == SIZE_MAX || idx_dsub == SIZE_MAX || idx_ua == SIZE_MAX || idx_ub == SIZE_MAX || idx_uc == SIZE_MAX ||
       idx_noff == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_w, {.d{10e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_kp, {.d{1e-4}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vth0, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_phi, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_gamma, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    // Keep advanced effects off.
    (void)m1->ptr->set_attribute(idx_lambda, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pclm, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_dsub, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ua, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ub, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_uc, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_noff, {.d{noff}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    auto const bv = vds_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i0 = run_case(0.0);
    double const i2 = run_case(2.0);
    if(!std::isfinite(i0) || !std::isfinite(i2)) { return 1; }

    // Larger noff -> larger effective n -> larger subthreshold current at same Vgs (worse subthreshold slope).
    if(!(i0 > 1e-18)) { return 2; }
    if(!(i2 > i0)) { return 3; }
    return 0;
}
