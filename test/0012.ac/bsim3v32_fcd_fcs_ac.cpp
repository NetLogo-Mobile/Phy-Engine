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

static std::complex<double> run_case(double fcd) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Bias node at -0.95V DC (AC ground). For NMOS, Vbd = Vb - Vd = 0 - (-0.95) = +0.95V (forward bias).
    auto [v_bias_src, v_bias_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -0.95});
    auto& n_bias = create_node(nl);
    add_to_node(nl, *v_bias_src, 0, n_bias);
    add_to_node(nl, *v_bias_src, 1, nl.ground_node);

    // Drive drain with 1V AC around the -0.95V DC bias.
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_drain);
    add_to_node(nl, *vac_src, 1, n_bias);

    // Gate/source at 0V so channel is off; bulk at 0V.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    // Enable depletion capacitance on the drain junction only.
    auto const idx_cj = find_attr(m1, "cj");
    auto const idx_ad = find_attr(m1, "ad");
    auto const idx_pb = find_attr(m1, "pb");
    auto const idx_fcd = find_attr(m1, "fcd");
    auto const idx_diode_is = find_attr(m1, "diode_Is");
    if(idx_cj == SIZE_MAX || idx_ad == SIZE_MAX || idx_pb == SIZE_MAX || idx_fcd == SIZE_MAX || idx_diode_is == SIZE_MAX) { return {NAN, NAN}; }

    // Suppress diode conductance so the AC current is dominated by depletion capacitance.
    (void)m1->ptr->set_attribute(idx_diode_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_pb, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_cj, {.d{2e-3}, .type{::phy_engine::model::variant_type::d}});  // F/m^2
    (void)m1->ptr->set_attribute(idx_ad, {.d{1e-10}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_fcd, {.d{fcd}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return {NAN, NAN}; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return {NAN, NAN}; }
    return bv.branches[0].current;
}

int main()
{
    // Larger fcd -> later transition point and different linear continuation; at strong forward bias,
    // it should noticeably change the depletion capacitance.
    auto const i_lo = run_case(0.2);
    auto const i_hi = run_case(0.8);

    if(!std::isfinite(i_lo.real()) || !std::isfinite(i_lo.imag())) { return 1; }
    if(!std::isfinite(i_hi.real()) || !std::isfinite(i_hi.imag())) { return 2; }

    // AC current magnitude is dominated by imaginary (capacitive) part.
    if(!(std::abs(i_hi.imag()) > std::abs(i_lo.imag()) * 1.5)) { return 3; }
    return 0;
}

