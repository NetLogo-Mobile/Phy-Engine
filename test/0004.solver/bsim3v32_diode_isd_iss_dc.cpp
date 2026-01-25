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

    constexpr std::size_t kMaxScan{768};
    for(std::size_t idx{}; idx < kMaxScan; ++idx)
    {
        auto const n = m->ptr->get_attribute_name(idx);
        if(!eq_name(n)) { continue; }
        return idx;
    }
    return SIZE_MAX;
}

static double run_case(bool drive_drain, double isd, double iss) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Bulk at +0.7V to forward-bias the diffusion diode under test.
    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.7});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    // Gate at 0V so channel is off.
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Node under test returned to ground through a resistor.
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_x = create_node(nl);
    add_to_node(nl, *rload, 0, n_x);
    add_to_node(nl, *rload, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    if(drive_drain)
    {
        // Test drain diode: drain at node-under-test, source tied to bulk (so B-S is off).
        add_to_node(nl, *m1, 0, n_x);
        add_to_node(nl, *m1, 1, n_gate);
        add_to_node(nl, *m1, 2, n_bulk);
        add_to_node(nl, *m1, 3, n_bulk);
    }
    else
    {
        // Test source diode: source at node-under-test, drain tied to bulk (so B-D is off).
        add_to_node(nl, *m1, 0, n_bulk);
        add_to_node(nl, *m1, 1, n_gate);
        add_to_node(nl, *m1, 2, n_x);
        add_to_node(nl, *m1, 3, n_bulk);
    }

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_is = find_attr(m1, "diode_Is");
    auto const idx_n = find_attr(m1, "diode_N");
    auto const idx_isd = find_attr(m1, "diode_Isd");
    auto const idx_iss = find_attr(m1, "diode_Iss");
    auto const idx_js = find_attr(m1, "js");
    auto const idx_jsw = find_attr(m1, "jsw");
    auto const idx_jswg = find_attr(m1, "jswg");
    if(idx_kp == SIZE_MAX || idx_is == SIZE_MAX || idx_n == SIZE_MAX || idx_isd == SIZE_MAX || idx_iss == SIZE_MAX || idx_js == SIZE_MAX ||
       idx_jsw == SIZE_MAX || idx_jswg == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Disable channel current and geometry-current scaling; rely on diode_Isd/diode_Iss only.
    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_js, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jsw, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jswg, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_n, {.d{2.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_isd, {.d{isd}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_iss, {.d{iss}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    // Bulk supply current magnitude is the forward diode current.
    auto const bv = vb_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    // Larger diode_Isd than diode_Iss should make the drain diode conduct more than the source diode under identical bias.
    double const id = run_case(true, 1e-14, 1e-17);
    double const is = run_case(false, 1e-14, 1e-17);
    if(!std::isfinite(id) || !std::isfinite(is)) { return 1; }
    if(!(id > 1e-12) || !(is > 1e-12)) { return 2; }
    if(!(id > is * 100.0)) { return 3; }
    return 0;
}

