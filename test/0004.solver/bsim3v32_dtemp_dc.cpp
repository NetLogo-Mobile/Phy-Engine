#include <cmath>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
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

static double run_case(double env_temp_c, bool set_inst_temp, double inst_temp_c, double dtemp_c) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    c.get_environment().temperature = env_temp_c;

    auto& nl = c.get_netlist();

    auto [vdd_src, vdd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 3.0});
    auto& n_vdd = create_node(nl);
    add_to_node(nl, *vdd_src, 0, n_vdd);
    add_to_node(nl, *vdd_src, 1, nl.ground_node);

    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_vdd);
    add_to_node(nl, *rload, 1, n_drain);

    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);  // source
    add_to_node(nl, *m1, 3, nl.ground_node);  // bulk

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_is = find_attr(m1, "diode_Is");
    auto const idx_tnom = find_attr(m1, "tnom");
    auto const idx_xti = find_attr(m1, "xti");
    auto const idx_eg = find_attr(m1, "eg");
    auto const idx_temp = find_attr(m1, "Temp");
    auto const idx_dtemp = find_attr(m1, "dtemp");
    if(idx_kp == SIZE_MAX || idx_is == SIZE_MAX || idx_tnom == SIZE_MAX || idx_xti == SIZE_MAX || idx_eg == SIZE_MAX || idx_temp == SIZE_MAX ||
       idx_dtemp == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-12}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_tnom, {.d{27.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_xti, {.d{3.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_eg, {.d{1.11}, .type{::phy_engine::model::variant_type::d}});

    if(set_inst_temp)
    {
        (void)m1->ptr->set_attribute(idx_temp, {.d{inst_temp_c}, .type{::phy_engine::model::variant_type::d}});
    }
    (void)m1->ptr->set_attribute(idx_dtemp, {.d{dtemp_c}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    // DTEMP should affect device temperature even when TEMP is not overridden.
    double const v_env27_d0 = run_case(27.0, false, 0.0, 0.0);
    double const v_env27_d100 = run_case(27.0, false, 0.0, 100.0);
    if(!std::isfinite(v_env27_d0) || !std::isfinite(v_env27_d100)) { return 1; }
    if(!(v_env27_d100 < v_env27_d0 - 1e-4)) { return 2; }

    // TEMP override must compose with DTEMP: TEMP=27 + DTEMP=100 should match ENV=127 (DTEMP=0).
    double const v_env127_d0 = run_case(127.0, false, 0.0, 0.0);
    double const v_temp27_d100 = run_case(127.0, true, 27.0, 100.0);
    if(!std::isfinite(v_env127_d0) || !std::isfinite(v_temp27_d100)) { return 3; }
    if(!(std::abs(v_env127_d0 - v_temp27_d100) < 1e-4)) { return 4; }

    return 0;
}

