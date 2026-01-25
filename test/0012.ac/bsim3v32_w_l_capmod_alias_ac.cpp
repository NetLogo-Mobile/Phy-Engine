#include <cmath>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VAC.h>
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

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::AC);
    double const omega{1e6};
    c.get_analyze_setting().ac.omega = omega;

    auto& nl = c.get_netlist();

    // Gate driven by 1V AC source, referenced to ground.
    auto [vac_src, vac_pos] = add_model(nl, ::phy_engine::model::VAC{.m_Vp{1.0}, .m_omega{omega}});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vac_src, 0, n_gate);
    add_to_node(nl, *vac_src, 1, nl.ground_node);

    // NMOS tied off in cutoff: D,S,B grounded; G at 0V DC (VAC is 0V in OP).
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, nl.ground_node);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, nl.ground_node);

    auto const idx_w = find_attr(m1, "w");
    auto const idx_l = find_attr(m1, "l");
    auto const idx_tox = find_attr(m1, "tox");
    auto const idx_toxm = find_attr(m1, "toxm");
    auto const idx_vth0 = find_attr(m1, "Vth0");
    auto const idx_phi = find_attr(m1, "phi");
    auto const idx_capmod = find_attr(m1, "capmod");
    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_tox == SIZE_MAX || idx_toxm == SIZE_MAX || idx_vth0 == SIZE_MAX || idx_phi == SIZE_MAX ||
       idx_capmod == SIZE_MAX)
    {
        return 1;
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

    if(!c.analyze()) { return 2; }

    auto const bv = vac_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return 3; }

    auto const iac = bv.branches[0].current;
    double const ceq = std::abs(iac.imag()) / omega;

    double const cox = ::phy_engine::model::details::k_eps_ox / tox;
    double const cgg = cox * w * l;

    if(!std::isfinite(ceq) || !(ceq > cgg * 0.05)) { return 4; }
    if(!(ceq < cgg * 5.0)) { return 5; }
    return 0;
}

