#include <cmath>
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

static double run_case(double voffcv) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Gate DC bias: VDC=0.8V from ng_bias to ground.
    auto [vdc_gate, vdc_gate_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.8});
    auto& ng_bias = create_node(nl);
    add_to_node(nl, *vdc_gate, 0, ng_bias);
    add_to_node(nl, *vdc_gate, 1, nl.ground_node);

    // Gate AC excitation: VAC=1V from ng to ng_bias (AC referenced to AC-grounded bias node).
    auto [vac_gate, vac_gate_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& ng = create_node(nl);
    add_to_node(nl, *vac_gate, 0, ng);
    add_to_node(nl, *vac_gate, 1, ng_bias);

    // Bulk current probe: 0V VDC between nb and ground.
    auto [vdc_bulk, vdc_bulk_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& nb = create_node(nl);
    add_to_node(nl, *vdc_bulk, 0, nb);
    add_to_node(nl, *vdc_bulk, 1, nl.ground_node);

    // NMOS: D,S grounded, B=nb, G=ng.
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, nl.ground_node);
    add_to_node(nl, *m1, 1, ng);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nb);

    auto const idx_w = find_attr(m1, "W");
    auto const idx_l = find_attr(m1, "L");
    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_toxm = find_attr(m1, "toxm");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_capmod = find_attr(m1, "capMod");
    auto const idx_voff = find_attr(m1, "voff");
    auto const idx_voffcv = find_attr(m1, "voffcv");

    auto const idx_cgso = find_attr(m1, "cgso");
    auto const idx_cgdo = find_attr(m1, "cgdo");
    auto const idx_cgbo = find_attr(m1, "cgbo");
    auto const idx_cgs = find_attr(m1, "Cgs");
    auto const idx_cgd = find_attr(m1, "Cgd");
    auto const idx_cgb = find_attr(m1, "Cgb");

    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_tox == SIZE_MAX || idx_toxm == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX ||
       idx_capmod == SIZE_MAX || idx_voff == SIZE_MAX || idx_voffcv == SIZE_MAX || idx_cgso == SIZE_MAX || idx_cgdo == SIZE_MAX || idx_cgbo == SIZE_MAX ||
       idx_cgs == SIZE_MAX || idx_cgd == SIZE_MAX || idx_cgb == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    double const w{1e-6};
    double const l{1e-6};
    double const tox{1e-8};
    (void)m1->ptr->set_attribute(idx_w, {.d{w}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{l}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_tox, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_toxm, {.d{tox}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vth0, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_phi, {.d{0.7}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_capmod, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});

    // Isolate the CV cutoff gate: keep DC voff=0, vary voffcv.
    (void)m1->ptr->set_attribute(idx_voff, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_voffcv, {.d{voffcv}, .type{::phy_engine::model::variant_type::d}});

    // Ensure we are measuring intrinsic charge-based coupling (no overlap/fixed caps).
    (void)m1->ptr->set_attribute(idx_cgso, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgdo, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgbo, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgs, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cgb, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vdc_bulk->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }

    return std::abs(bv.branches[0].current.imag());
}

int main()
{
    // With Vgdc=0.8V and Vth0=0.7V:
    // - voffcv=0.0 => vgst≈+0.1 => f_cut≈0 => weaker gate-bulk coupling
    // - voffcv=0.2 => vgst≈-0.1 => f_cut≈1 => stronger gate-bulk coupling
    double const ib0 = run_case(0.0);
    double const ib2 = run_case(0.2);

    if(!std::isfinite(ib0) || !std::isfinite(ib2)) { return 1; }
    if(!(ib2 > ib0 * 1.5)) { return 2; }
    return 0;
}

