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

    constexpr std::size_t kMaxScan{512};
    for(std::size_t idx{}; idx < kMaxScan; ++idx)
    {
        auto const n = m->ptr->get_attribute_name(idx);
        if(!eq_name(n)) { continue; }
        return idx;
    }
    return SIZE_MAX;
}

struct junc_params
{
    double cjd{};
    double cjs{};
    double pbd{};
    double pbs{};
};

static std::complex<double> run_case(bool drive_drain, junc_params jp) noexcept
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
    auto const idx_cjd = find_attr(m1, "cjd");
    auto const idx_cjs = find_attr(m1, "cjs");
    auto const idx_cjsw = find_attr(m1, "cjsw");
    auto const idx_pd = find_attr(m1, "pd");
    auto const idx_ps = find_attr(m1, "ps");
    auto const idx_ad = find_attr(m1, "ad");
    auto const idx_as = find_attr(m1, "as");
    auto const idx_pb = find_attr(m1, "pb");
    auto const idx_pbd = find_attr(m1, "pbd");
    auto const idx_pbs = find_attr(m1, "pbs");
    if(idx_tox == SIZE_MAX || idx_cj == SIZE_MAX || idx_cjd == SIZE_MAX || idx_cjs == SIZE_MAX || idx_cjsw == SIZE_MAX || idx_pd == SIZE_MAX ||
       idx_ps == SIZE_MAX || idx_ad == SIZE_MAX || idx_as == SIZE_MAX || idx_pb == SIZE_MAX || idx_pbd == SIZE_MAX || idx_pbs == SIZE_MAX)
    {
        return {NAN, NAN};
    }

    // Make intrinsic channel capacitances negligible so junction C dominates.
    (void)m1->ptr->set_attribute(idx_tox, {.d{1e-3}, .type{::phy_engine::model::variant_type::d}});  // m

    // Isolate bottom depletion capacitance only (no sidewall).
    (void)m1->ptr->set_attribute(idx_cj, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cjsw, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ps, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ad, {.d{1e-10}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_as, {.d{1e-10}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_cjd, {.d{jp.cjd}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cjs, {.d{jp.cjs}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_pb, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pbd, {.d{jp.pbd}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pbs, {.d{jp.pbs}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return {NAN, NAN}; }

    auto const bv = vac->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return {NAN, NAN}; }
    return bv.branches[0].current;
}

int main()
{
    // 1) Per-junction bottom Cj density: cjd affects only the drain junction, cjs only the source junction.
    junc_params p1{.cjd{4e-3}, .cjs{1e-3}, .pbd{0.0}, .pbs{0.0}};
    auto const id = run_case(true, p1);
    auto const is = run_case(false, p1);
    if(!std::isfinite(id.real()) || !std::isfinite(id.imag())) { return 1; }
    if(!std::isfinite(is.real()) || !std::isfinite(is.imag())) { return 2; }
    if(!(std::abs(id.imag()) > std::abs(is.imag()) * 2.0)) { return 3; }

    // 2) Per-junction bottom potential: pbd/pbs override pb for bottom junction only.
    junc_params p2{.cjd{2e-3}, .cjs{2e-3}, .pbd{4.0}, .pbs{1.0}};
    auto const id2 = run_case(true, p2);
    auto const is2 = run_case(false, p2);
    if(!std::isfinite(id2.real()) || !std::isfinite(id2.imag())) { return 4; }
    if(!std::isfinite(is2.real()) || !std::isfinite(is2.imag())) { return 5; }
    if(!(std::abs(id2.imag()) > std::abs(is2.imag()) * 1.25)) { return 6; }

    return 0;
}

