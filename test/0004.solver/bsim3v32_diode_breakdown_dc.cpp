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

static double run_case(bool use_source_diode, bool enable_breakdown) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    // Reverse-bias one junction at +1.1V (bulk at 0V).
    auto [vd_src, vd_pos] = add_model(nl, ::phy_engine::model::VDC{.V = use_source_diode ? 0.0 : 1.1});
    auto& n_drain = create_node(nl);
    add_to_node(nl, *vd_src, 0, n_drain);
    add_to_node(nl, *vd_src, 1, nl.ground_node);

    auto [vs_src, vs_pos] = add_model(nl, ::phy_engine::model::VDC{.V = use_source_diode ? 1.1 : 0.0});
    auto& n_source = create_node(nl);
    add_to_node(nl, *vs_src, 0, n_source);
    add_to_node(nl, *vs_src, 1, nl.ground_node);

    // Gate and source at 0V (disable channel; isolate diode path).
    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    // Bulk external pin at 0V through a VDC source so we can measure bulk current.
    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    add_to_node(nl, *m1, 0, n_drain);
    add_to_node(nl, *m1, 1, n_gate);
    add_to_node(nl, *m1, 2, n_source);  // source
    add_to_node(nl, *m1, 3, n_bulk);          // bulk

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_is = find_attr(m1, "diode_Is");
    auto const idx_n = find_attr(m1, "diode_N");
    auto const idx_bvd = find_attr(m1, "bvd");
    auto const idx_ibvd = find_attr(m1, "ibvd");
    auto const idx_bvs = find_attr(m1, "bvs");
    auto const idx_ibvs = find_attr(m1, "ibvs");
    if(idx_kp == SIZE_MAX || idx_is == SIZE_MAX || idx_n == SIZE_MAX || idx_bvd == SIZE_MAX || idx_ibvd == SIZE_MAX || idx_bvs == SIZE_MAX ||
       idx_ibvs == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-9}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_n, {.d{1.0}, .type{::phy_engine::model::variant_type::d}});

    // Choose ibv=Is so that PN_junction Bv_eff ~= bv; then bias Vj=Bv+0.1 gives a modest breakdown multiplier.
    (void)m1->ptr->set_attribute(idx_bvd, {.d{(!use_source_diode && enable_breakdown) ? 1.0 : 0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ibvd, {.d{1e-9}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_bvs, {.d{(use_source_diode && enable_breakdown) ? 1.0 : 0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ibvs, {.d{1e-9}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vb_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const id_off = run_case(false, false);
    double const id_on = run_case(false, true);
    double const is_off = run_case(true, false);
    double const is_on = run_case(true, true);
    if(!std::isfinite(id_off) || !std::isfinite(id_on) || !std::isfinite(is_off) || !std::isfinite(is_on)) { return 1; }

    if(!(id_off > 1e-10) || !(is_off > 1e-10)) { return 2; }
    if(!(id_on > id_off * 20.0)) { return 3; }
    if(!(is_on > is_off * 20.0)) { return 4; }
    if(!(id_on > 1e-8) || !(is_on > 1e-8)) { return 5; }
    return 0;
}
