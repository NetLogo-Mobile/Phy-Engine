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

static double run_case(double nf) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Bulk bias = 0.7V (forward-bias the body diode into a measurable but not fully saturated region).
    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.7});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    // VG = 0V (keep channel off; exercise only the body diode scaling).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Load resistor from drain to ground.
    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *rload, 0, n_drain);
    add_to_node(nl, *rload, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);
    add_to_node(nl, *m1, 3, n_bulk);

    auto const idx_nf = find_attr(m1, "nf");
    auto const idx_is = find_attr(m1, "diode_Is");
    if(idx_nf == SIZE_MAX || idx_is == SIZE_MAX) { return std::numeric_limits<double>::quiet_NaN(); }
    // Set a larger saturation current so forward diode current is measurable.
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-10}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_nf, {.d{nf}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }
    return n_drain.node_information.an.voltage.real();
}

int main()
{
    double const vd_nf1 = run_case(1.0);
    double const vd_nf10 = run_case(10.0);

    if(!std::isfinite(vd_nf1) || !std::isfinite(vd_nf10)) { return 1; }

    // Larger nf => larger diode current => higher drain voltage (closer to bulk).
    if(!(vd_nf10 > vd_nf1 + 1e-3)) { return 2; }
    return 0;
}
