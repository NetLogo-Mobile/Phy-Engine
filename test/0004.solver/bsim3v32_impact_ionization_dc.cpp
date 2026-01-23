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

static double run_case(double vds, bool enable_ii) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Drain at +VDS
    auto [vds_src, vds_pos] = add_model(nl, ::phy_engine::model::VDC{.V = vds});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vds_src, 0, n_drain);
    add_to_node(nl, *vds_src, 1, nl.ground_node);

    // Gate at +2V (strong inversion)
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 2.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Bulk held at -1V so S/B and D/B diodes remain reverse-biased.
    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = -1.0});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, nl.ground_node);  // source
    add_to_node(nl, *m1, 3, n_bulk);          // bulk

    auto const idx_w = find_attr(m1, "W");
    auto const idx_l = find_attr(m1, "L");
    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_diode_is = find_attr(m1, "diode_Is");
    auto const idx_alpha0 = find_attr(m1, "alpha0");
    auto const idx_beta0 = find_attr(m1, "beta0");
    auto const idx_vdsatii = find_attr(m1, "vdsatii");
    if(idx_w == SIZE_MAX || idx_l == SIZE_MAX || idx_kp == SIZE_MAX || idx_diode_is == SIZE_MAX || idx_alpha0 == SIZE_MAX || idx_beta0 == SIZE_MAX ||
       idx_vdsatii == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_w, {.d{10e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_l, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_kp, {.d{1e-4}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_diode_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_alpha0, {.d{enable_ii ? 0.05 : 0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_beta0, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_vdsatii, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    // Measure bulk supply current magnitude via the VDC branch.
    auto const bv = vb_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const i_no = run_case(5.0, false);
    double const i_on = run_case(5.0, true);
    if(!std::isfinite(i_no) || !std::isfinite(i_on)) { return 1; }

    // With impact ionization enabled, bulk current must increase significantly.
    if(!(i_on > i_no * 100.0 + 1e-15)) { return 2; }

    // Also check it increases with higher Vds (field-driven).
    double const i_on_1v = run_case(1.0, true);
    if(!std::isfinite(i_on_1v)) { return 3; }
    if(!(i_on > i_on_1v * 5.0)) { return 4; }

    return 0;
}

