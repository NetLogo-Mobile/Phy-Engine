#include <cmath>
#include <complex>
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

    constexpr std::size_t kMaxScan{768};
    for(std::size_t idx{}; idx < kMaxScan; ++idx)
    {
        auto const n = m->ptr->get_attribute_name(idx);
        if(!eq_name(n)) { continue; }
        return idx;
    }
    return SIZE_MAX;
}

struct mj_params
{
    double mjd{-1.0};
    double mjs{-1.0};
};

static std::complex<double> run_case(bool drive_drain, mj_params mp) noexcept
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

    // Gate held at 0V, so channel is off; we want junction depletion C to dominate.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    ::phy_engine::model::node_t* n_drain = __builtin_addressof(n_bias);
    ::phy_engine::model::node_t* n_source = __builtin_addressof(n_bias);

    ::phy_engine::model::model_base* vac = nullptr;
    if(drive_drain)
    {
        auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
        auto& n_d = create_node(nl);
        add_to_node(nl, *vac_src, 0, n_d);
        add_to_node(nl, *vac_src, 1, n_bias);
        vac = vac_src;
        n_drain = __builtin_addressof(n_d);
    }
    else
    {
        auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
        auto& n_s = create_node(nl);
        add_to_node(nl, *vac_src, 0, n_s);
        add_to_node(nl, *vac_src, 1, n_bias);
        vac = vac_src;
        n_source = __builtin_addressof(n_s);
    }

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, *n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, *n_source);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_cj = find_attr(m1, "cj");
    auto const idx_ad = find_attr(m1, "ad");
    auto const idx_as = find_attr(m1, "as");
    auto const idx_cjsw = find_attr(m1, "cjsw");
    auto const idx_pd = find_attr(m1, "pd");
    auto const idx_ps = find_attr(m1, "ps");
    auto const idx_pb = find_attr(m1, "pb");
    auto const idx_mj = find_attr(m1, "mj");
    auto const idx_mjd = find_attr(m1, "mjd");
    auto const idx_mjs = find_attr(m1, "mjs");
    if(idx_tox == SIZE_MAX || idx_cj == SIZE_MAX || idx_ad == SIZE_MAX || idx_as == SIZE_MAX || idx_cjsw == SIZE_MAX || idx_pd == SIZE_MAX ||
       idx_ps == SIZE_MAX || idx_pb == SIZE_MAX || idx_mj == SIZE_MAX || idx_mjd == SIZE_MAX || idx_mjs == SIZE_MAX)
    {
        return {NAN, NAN};
    }

    // Make intrinsic channel capacitances negligible so junction C dominates.
    (void)m1->ptr->set_attribute(idx_tox, {.d{1e-3}, .type{::phy_engine::model::variant_type::d}});  // m

    // Use only bottom depletion capacitance.
    (void)m1->ptr->set_attribute(idx_cj, {.d{2e-3}, .type{::phy_engine::model::variant_type::d}});   // F/m^2
    (void)m1->ptr->set_attribute(idx_ad, {.d{1e-10}, .type{::phy_engine::model::variant_type::d}});  // m^2
    (void)m1->ptr->set_attribute(idx_as, {.d{1e-10}, .type{::phy_engine::model::variant_type::d}});  // m^2
    (void)m1->ptr->set_attribute(idx_cjsw, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ps, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pb, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});

    // Baseline grading coefficients.
    (void)m1->ptr->set_attribute(idx_mj, {.d{0.5}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_mjd, {.d{mp.mjd}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_mjs, {.d{mp.mjs}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return {NAN, NAN}; }

    auto const bv = vac->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return {NAN, NAN}; }
    return bv.branches[0].current;
}

int main()
{
    // Baseline (no overrides): drain and source should match.
    auto const id0 = run_case(true, {});
    auto const is0 = run_case(false, {});
    if(!std::isfinite(id0.real()) || !std::isfinite(id0.imag())) { return 1; }
    if(!std::isfinite(is0.real()) || !std::isfinite(is0.imag())) { return 2; }
    if(!(std::abs(id0.imag() - is0.imag()) < std::abs(id0.imag()) * 0.05 + 1e-12)) { return 3; }

    // Set mjd only: drain junction capacitance should change, source should remain near baseline.
    auto const id_mjd = run_case(true, {.mjd{0.9}, .mjs{-1.0}});
    auto const is_mjd = run_case(false, {.mjd{0.9}, .mjs{-1.0}});
    if(!std::isfinite(id_mjd.real()) || !std::isfinite(id_mjd.imag())) { return 4; }
    if(!std::isfinite(is_mjd.real()) || !std::isfinite(is_mjd.imag())) { return 5; }
    if(!(std::abs(id_mjd.imag() - id0.imag()) > std::abs(id0.imag()) * 0.10 + 1e-12)) { return 6; }
    if(!(std::abs(is_mjd.imag() - is0.imag()) < std::abs(is0.imag()) * 0.05 + 1e-12)) { return 7; }

    // Set mjs only: source junction capacitance should change, drain should remain near baseline.
    auto const id_mjs = run_case(true, {.mjd{-1.0}, .mjs{0.9}});
    auto const is_mjs = run_case(false, {.mjd{-1.0}, .mjs{0.9}});
    if(!std::isfinite(id_mjs.real()) || !std::isfinite(id_mjs.imag())) { return 8; }
    if(!std::isfinite(is_mjs.real()) || !std::isfinite(is_mjs.imag())) { return 9; }
    if(!(std::abs(is_mjs.imag() - is0.imag()) > std::abs(is0.imag()) * 0.10 + 1e-12)) { return 10; }
    if(!(std::abs(id_mjs.imag() - id0.imag()) < std::abs(id0.imag()) * 0.05 + 1e-12)) { return 11; }

    return 0;
}

