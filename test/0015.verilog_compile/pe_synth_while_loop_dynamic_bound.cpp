#include <array>
#include <cstddef>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
    inline ::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
    {
        ::phy_engine::model::variant vi{};
        vi.digital = v;
        vi.type = ::phy_engine::model::variant_type::digital;
        return vi;
    }
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module top(input [1:0] n, input [3:0] a, output reg [3:0] y);
  reg [2:0] i;
  always @* begin
    y = 4'b0000;
    i = 0;
    while(i < n) begin
      y[i] = a[i];
      i = i + 1;
    end
  end
endmodule
)";

    ::phy_engine::circult cct{};
    cct.set_analyze_type(::phy_engine::analyze_type::TR);
    cct.get_analyze_setting().tr.t_step = 1e-9;
    cct.get_analyze_setting().tr.t_stop = 1e-9;
    auto& nl = cct.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"top");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    ::std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    auto port_idx = [&](::fast_io::u8string_view name) noexcept -> ::std::size_t
    {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            if(p.name == name) { return i; }
        }
        return SIZE_MAX;
    };

    auto const n1_idx = port_idx(u8"n[1]");
    auto const n0_idx = port_idx(u8"n[0]");
    if(n1_idx == SIZE_MAX || n0_idx == SIZE_MAX) { return 4; }

    ::std::array<::std::size_t, 4> a_idx{
        port_idx(u8"a[3]"),
        port_idx(u8"a[2]"),
        port_idx(u8"a[1]"),
        port_idx(u8"a[0]"),
    };
    ::std::array<::std::size_t, 4> y_idx{
        port_idx(u8"y[3]"),
        port_idx(u8"y[2]"),
        port_idx(u8"y[1]"),
        port_idx(u8"y[0]"),
    };
    for(auto v: a_idx)
    {
        if(v == SIZE_MAX) { return 5; }
    }
    for(auto v: y_idx)
    {
        if(v == SIZE_MAX) { return 6; }
    }

    auto [in_n1, p0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{});
    auto [in_n0, p1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{});
    (void)p0;
    (void)p1;
    if(in_n1 == nullptr || in_n0 == nullptr) { return 7; }

    ::std::array<::phy_engine::model::model_base*, 4> in_a{};
    for(auto& p: in_a)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{});
        (void)pos;
        if(m == nullptr) { return 8; }
        p = m;
    }

    ::std::array<::phy_engine::model::model_base*, 4> out_y{};
    for(auto& p: out_y)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr) { return 9; }
        p = m;
    }

    if(!::phy_engine::netlist::add_to_node(nl, *in_n1, 0, *ports[n1_idx])) { return 10; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_n0, 0, *ports[n0_idx])) { return 11; }
    for(::std::size_t i{}; i < 4; ++i)
    {
        if(!::phy_engine::netlist::add_to_node(nl, *in_a[i], 0, *ports[a_idx[i]])) { return 12; }
        if(!::phy_engine::netlist::add_to_node(nl, *out_y[i], 0, *ports[y_idx[i]])) { return 13; }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = true,
        .allow_multi_driver = true,
        .loop_unroll_limit = 16,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 14; }

    if(!cct.analyze()) { return 15; }

    auto settle = [&]() noexcept
    {
        for(::std::size_t i{}; i < 32; ++i) { cct.digital_clk(); }
    };

    auto set_in = [&](::phy_engine::model::model_base* in, ::phy_engine::model::digital_node_statement_t v) noexcept {
        (void)in->ptr->set_attribute(0, dv(v));
    };

    // n = 3, a = 1101 => y[2:0]=a[2:0]=101, y[3]=0 => 0101
    set_in(in_n1, ::phy_engine::model::digital_node_statement_t::true_state);
    set_in(in_n0, ::phy_engine::model::digital_node_statement_t::true_state);

    set_in(in_a[0], ::phy_engine::model::digital_node_statement_t::true_state);   // a[3]
    set_in(in_a[1], ::phy_engine::model::digital_node_statement_t::true_state);   // a[2]
    set_in(in_a[2], ::phy_engine::model::digital_node_statement_t::false_state);  // a[1]
    set_in(in_a[3], ::phy_engine::model::digital_node_statement_t::true_state);   // a[0]

    settle();

    if(ports[y_idx[0]]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 16; }
    if(ports[y_idx[1]]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 17; }
    if(ports[y_idx[2]]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::false_state) { return 18; }
    if(ports[y_idx[3]]->node_information.dn.state != ::phy_engine::model::digital_node_statement_t::true_state) { return 19; }

    return 0;
}
