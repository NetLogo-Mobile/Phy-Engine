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

static double run_case(bool drive_drain, double jsrwgd, double jsrwgs) noexcept
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl = c.get_netlist();

    auto [vb_src, vb_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.7});
    auto& n_bulk = create_node(nl);
    add_to_node(nl, *vb_src, 0, n_bulk);
    add_to_node(nl, *vb_src, 1, nl.ground_node);

    auto [vg_src, vg_pos] = add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
    auto& n_gate = create_node(nl);
    add_to_node(nl, *vg_src, 0, n_gate);
    add_to_node(nl, *vg_src, 1, nl.ground_node);

    auto [rload, rload_pos] = add_model(nl, ::phy_engine::model::resistance{.r = 1'000.0});
    auto& n_x = create_node(nl);
    add_to_node(nl, *rload, 0, n_x);
    add_to_node(nl, *rload, 1, nl.ground_node);

    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});
    if(drive_drain)
    {
        add_to_node(nl, *m1, 0, n_x);
        add_to_node(nl, *m1, 1, n_gate);
        add_to_node(nl, *m1, 2, n_bulk);
        add_to_node(nl, *m1, 3, n_bulk);
    }
    else
    {
        add_to_node(nl, *m1, 0, n_bulk);
        add_to_node(nl, *m1, 1, n_gate);
        add_to_node(nl, *m1, 2, n_x);
        add_to_node(nl, *m1, 3, n_bulk);
    }

    auto const idx_kp = find_attr(m1, "Kp");
    auto const idx_is = find_attr(m1, "diode_Is");
    auto const idx_isr = find_attr(m1, "diode_Isr");
    auto const idx_n = find_attr(m1, "diode_N");
    auto const idx_nr = find_attr(m1, "diode_Nr");
    auto const idx_w = find_attr(m1, "W");
    auto const idx_js = find_attr(m1, "js");
    auto const idx_jsw = find_attr(m1, "jsw");
    auto const idx_jswg = find_attr(m1, "jswg");
    auto const idx_jsr = find_attr(m1, "jsr");
    auto const idx_jsrwgd = find_attr(m1, "jsrwgd");
    auto const idx_jsrwgs = find_attr(m1, "jsrwgs");
    auto const idx_ad = find_attr(m1, "ad");
    auto const idx_as = find_attr(m1, "as");
    auto const idx_pd = find_attr(m1, "pd");
    auto const idx_ps = find_attr(m1, "ps");
    if(idx_kp == SIZE_MAX || idx_is == SIZE_MAX || idx_isr == SIZE_MAX || idx_n == SIZE_MAX || idx_nr == SIZE_MAX || idx_w == SIZE_MAX || idx_js == SIZE_MAX ||
       idx_jsw == SIZE_MAX || idx_jswg == SIZE_MAX || idx_jsr == SIZE_MAX || idx_jsrwgd == SIZE_MAX || idx_jsrwgs == SIZE_MAX || idx_ad == SIZE_MAX ||
       idx_as == SIZE_MAX || idx_pd == SIZE_MAX || idx_ps == SIZE_MAX)
    {
        return std::numeric_limits<double>::quiet_NaN();
    }

    (void)m1->ptr->set_attribute(idx_kp, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_is, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_isr, {.d{1e-30}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_n, {.d{2.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_nr, {.d{2.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_w, {.d{1e-6}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_js, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jsw, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jswg, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});

    (void)m1->ptr->set_attribute(idx_jsr, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ad, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_as, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_pd, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_ps, {.d{0.0}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jsrwgd, {.d{jsrwgd}, .type{::phy_engine::model::variant_type::d}});
    (void)m1->ptr->set_attribute(idx_jsrwgs, {.d{jsrwgs}, .type{::phy_engine::model::variant_type::d}});

    if(!c.analyze()) { return std::numeric_limits<double>::quiet_NaN(); }

    auto const bv = vb_src->ptr->generate_branch_view();
    if(bv.size != 1 || bv.branches == nullptr) { return std::numeric_limits<double>::quiet_NaN(); }
    return std::abs(bv.branches[0].current.real());
}

int main()
{
    double const id = run_case(true, 1e-6, 1e-7);
    double const is = run_case(false, 1e-6, 1e-7);
    if(!std::isfinite(id) || !std::isfinite(is)) { return 1; }
    if(!(id > 1e-12) || !(is > 1e-12)) { return 2; }
    if(!(id > is * 5.0)) { return 3; }
    return 0;
}

