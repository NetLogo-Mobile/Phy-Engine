#include <cmath>

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

static std::complex<double> run_case(double pbswg) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Bias node at 3V DC (AC ground).
    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_bias);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    // Drive drain with 1V AC around the 3V DC bias.
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_drain);
    add_to_node(nl, *vac_src, 1, n_bias);

    // Gate held at 0V, so channel is off; we want junction depletion C to dominate.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    // Isolate the effect of pbswg on gate-edge sidewall depletion capacitance:
    // - Use only gate-edge sidewall (cjswg * weff).
    // - Set cj/ad and cjsw/pd to 0.
    // - Make intrinsic channel capacitances negligible by using a very thick oxide.
    auto const idx_cj = find_attr(m1, "cj");
    auto const idx_ad = find_attr(m1, "ad");
    auto const idx_cjsw = find_attr(m1, "cjsw");
    auto const idx_pd = find_attr(m1, "pd");
    auto const idx_cjswg = find_attr(m1, "cjswg");
    auto const idx_pbswg = find_attr(m1, "pbswg");
    auto const idx_mjswg = find_attr(m1, "mjswg");
    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_w = find_attr(m1, "W");
    if(idx_cj == SIZE_MAX || idx_ad == SIZE_MAX || idx_cjsw == SIZE_MAX || idx_pd == SIZE_MAX || idx_cjswg == SIZE_MAX || idx_pbswg == SIZE_MAX ||
       idx_mjswg == SIZE_MAX || idx_tox == SIZE_MAX || idx_w == SIZE_MAX)
    {
        return {NAN, NAN};
    }

    (void)m1->ptr->set_attribute(idx_cj, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ad, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cjsw, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_w, {.d{2e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cjswg, {.d{2e-3}, .type{::phy_engine::model::variant_type::d}}); // F/m
    (void)m1->ptr->set_attribute(idx_mjswg, {.d{0.5}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pbswg, {.d{pbswg}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_tox, {.d{1e-3}, .type{::phy_engine::model::variant_type::d}}); // m

    if(!c.analyze()) { return {NAN, NAN}; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return {NAN, NAN}; }
    return bv.branches[0].current;
}

int main()
{
    // Larger pbswg => weaker reverse-bias reduction => larger depletion capacitance => larger |Im(Iac)|.
    auto const i_small = run_case(0.5);
    auto const i_big = run_case(2.0);

    if(!std::isfinite(i_small.real()) || !std::isfinite(i_small.imag())) { return 1; }
    if(!std::isfinite(i_big.real()) || !std::isfinite(i_big.imag())) { return 2; }

    if(!(std::abs(i_big.imag()) > std::abs(i_small.imag()) * 1.2)) { return 3; }
    return 0;
}

