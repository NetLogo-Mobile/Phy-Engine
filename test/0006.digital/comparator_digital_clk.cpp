#include <cstdint>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/controller/comparator.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    bool set_vdc(::phy_engine::netlist::netlist& nl, ::phy_engine::netlist::model_pos pos, double v)
    {
        auto* m = get_model(nl, pos);
        if(m == nullptr || m->ptr == nullptr) { return false; }
        ::phy_engine::model::variant vi{};
        vi.d = v;
        vi.type = ::phy_engine::model::variant_type::d;
        return m->ptr->set_attribute(0, vi);
    }
}  // namespace

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};

    auto [va, va_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 1.0})};
    auto [vb, vb_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 0.5})};

    ::phy_engine::model::comparator cmp{};
    cmp.Ll = 0.0;
    cmp.Hl = 5.0;
    auto [u, u_pos]{add_model(nl, ::std::move(cmp))};

    auto [out, out_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};

    auto& node_a{create_node(nl)};
    auto& node_b{create_node(nl)};
    auto& node_o{create_node(nl)};
    auto& gnd{nl.ground_node};

    add_to_node(nl, *va, 0, node_a);
    add_to_node(nl, *va, 1, gnd);
    add_to_node(nl, *vb, 0, node_b);
    add_to_node(nl, *vb, 1, gnd);

    add_to_node(nl, *u, 0, node_a);
    add_to_node(nl, *u, 1, node_b);
    add_to_node(nl, *u, 2, node_o);

    add_to_node(nl, *out, 0, node_o);

    auto read_out = [&]() -> ::phy_engine::model::digital_node_statement_t
    {
        auto const v = out->ptr->get_attribute(0);
        if(v.type != ::phy_engine::model::variant_type::digital) { return ::phy_engine::model::digital_node_statement_t::X; }
        return v.digital;
    };

    if(!c.analyze())
    {
        ::fast_io::io::perr("comparator_digital_clk: analyze failed\n");
        return 1;
    }
    c.digital_clk();
    if(read_out() != ::phy_engine::model::digital_node_statement_t::true_state)
    {
        ::fast_io::io::perr("comparator_digital_clk: expected true when A>=B\n");
        return 1;
    }

    if(!set_vdc(nl, vb_pos, 1.5))
    {
        ::fast_io::io::perr("comparator_digital_clk: failed to set Vb\n");
        return 1;
    }
    if(!c.analyze())
    {
        ::fast_io::io::perr("comparator_digital_clk: analyze failed\n");
        return 1;
    }
    c.digital_clk();
    if(read_out() != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        ::fast_io::io::perr("comparator_digital_clk: expected false when A<B\n");
        return 1;
    }

    return 0;
}

