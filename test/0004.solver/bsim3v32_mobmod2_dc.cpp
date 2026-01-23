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

static double run_case(double mobmod) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // VDS = 50mV (linear region)
    auto [vds_src, vds_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.05});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vds_src, 0, n_drain);
    add_to_node(nl, *vds_src, 1, nl.ground_node);

    // VGS = 1.2V
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 1.2});
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
    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_toxm = find_attr(m1, "toxm");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_u0 = find_attr(m1, "u0");
    auto const idx_ua = find_attr(m1, "ua");
    auto const idx_ub = find_attr(m1, "ub");
    auto const idx_uc = find_attr(m1, "uc");
    auto const idx_mobmod = find_attr(m1, "mobMod");
    auto const idx_lambda = find_attr(m1, "lambda");
    auto const idx_pclm = find_attr(m1, "pclm");
    auto const idx_dsub = find_attr(m1, "dsub");
    auto const idx_eta0 = find_attr(m1, "eta0");
    auto const idx_etab = find_attr(m1, "etab");
    auto const idx_dvt0 = find_attr(m1, "dvt0");
    auto const idx_dvt1 = find_attr(m1, "dvt1");
    auto const idx_dvt2 = find_attr(m1, "dvt2");
    auto const idx_k1 = find_attr(m1, "k1");
    auto const idx_k2 = find_attr(m1, "k2");
    auto const idx_keta = find_attr(m1, "keta");

    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_tox == SIZE_MAX || idx_toxm == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX ||
       idx_u0 == SIZE_MAX || idx_ua == SIZE_MAX || idx_ub == SIZE_MAX || idx_uc == SIZE_MAX || idx_mobmod == SIZE_MAX || idx_lambda == SIZE_MAX ||
       idx_pclm == SIZE_MAX || idx_dsub == SIZE_MAX || idx_eta0 == SIZE_MAX || idx_etab == SIZE_MAX || idx_dvt0 == SIZE_MAX || idx_dvt1 == SIZE_MAX ||
       idx_dvt2 == SIZE_MAX || idx_k1 == SIZE_MAX || idx_k2 == SIZE_MAX || idx_keta == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double const tox{1e-8};
    (void)m1->ptr->set_attribute(idx_w, {.d{10e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_tox, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_toxm, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vth0, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_phi, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_u0, {.d{0.05}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ua, {.d{1e-8}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ub, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_uc, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_mobmod, {.d{mobmod}, .type{::phy_engine::model::variant_type::d}});

    // Disable other effects (keep Vgsteff-based core stable).
    (void)m1->ptr->set_attribute(idx_lambda, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pclm, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_dsub, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_eta0, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_etab, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_dvt0, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_dvt1, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_dvt2, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_k1, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_k2, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_keta, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vds_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return -bv.branches[0].current.real();
}

int main()
{
    double const id_mob2 = run_case(2.0);
    double const id_mob3 = run_case(3.0);

    if(!std::isfinite(id_mob2) || !std::isfinite(id_mob3)) { return 1; }

    double const a2 = std::abs(id_mob2);
    double const a3 = std::abs(id_mob3);
    if(!(a2 > 1e-12)) { return 2; }

    // With the same (ua,tox), mobMod=3 uses a slightly larger Eeff term (vgsteff+2Vt),
    // so it should yield lower current than mobMod=2.
    if(!(a2 > a3 * 1.02)) { return 3; }
    return 0;
}

