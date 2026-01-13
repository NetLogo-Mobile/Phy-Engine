#include <cstddef>
#include <cstdint>

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
module top(input [2:0] a, input [2:0] b, output [2:0] y);
  assign y = a + b;
endmodule
)";

    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

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

    // Create INPUT/OUTPUT models for each bit port.
    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }
        }
        else
        {
            return 8;
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 9; }
    if(!c.analyze()) { return 10; }

    // Ensure we actually emitted at least one adder macro.
    std::size_t full_adders{};
    std::size_t half_adders{};
    std::size_t xors{};
    std::size_t ands{};
    std::size_t ors{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            auto const n = m->ptr->get_model_name();
            if(n == u8"FULL_ADDER") { ++full_adders; }
            if(n == u8"HALF_ADDER") { ++half_adders; }
            if(n == u8"XOR") { ++xors; }
            if(n == u8"AND") { ++ands; }
            if(n == u8"OR") { ++ors; }
        }
    }
    if(full_adders == 0 || half_adders == 0)
    {
        std::fprintf(stderr,
                     "adder macros missing: full=%zu half=%zu (remaining gates: XOR=%zu AND=%zu OR=%zu)\n",
                     full_adders,
                     half_adders,
                     xors,
                     ands,
                     ors);
        return 11;
    }

    // Drive a=3 (011), b=4 (100) => y=7 (111)
    for(auto const& blk : nl.models)
    {
        for(auto* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() != u8"INPUT") { continue; }
            // external IO INPUTs have name like "a[0]", "a[1]", "b[0]"...
            auto const nm = m->name;
            if(nm == u8"a[0]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
            if(nm == u8"a[1]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
            if(nm == u8"a[2]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::false_state)); }
            if(nm == u8"b[0]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::false_state)); }
            if(nm == u8"b[1]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::false_state)); }
            if(nm == u8"b[2]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
        }
    }
    c.digital_clk();

    // Read y[0..2]
    auto bit_of = [&](::fast_io::u8string_view port_name) noexcept -> bool {
        for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(pi);
            if(p.name == port_name)
            {
                auto const s = ports[pi]->node_information.dn.state;
                return s == ::phy_engine::model::digital_node_statement_t::true_state;
            }
        }
        return false;
    };

    std::uint8_t y{};
    if(bit_of(u8"y[0]")) { y |= 1u; }
    if(bit_of(u8"y[1]")) { y |= 2u; }
    if(bit_of(u8"y[2]")) { y |= 4u; }
    if(y != 7u) { return 12; }

    return 0;
}
