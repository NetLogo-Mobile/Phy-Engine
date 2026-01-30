#include <cstddef>
#include <cstdint>
#include <cstdio>

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

char dn_char(::phy_engine::model::digital_node_statement_t st) noexcept
{
    using s = ::phy_engine::model::digital_node_statement_t;
    switch(st)
    {
        case s::false_state: return '0';
        case s::true_state: return '1';
        case s::indeterminate_state: return 'X';
        case s::high_impedence_state: return 'Z';
        default: return '?';
    }
}
}  // namespace

int main()
{
    // Cross-pass interaction: `optimize_adders` runs before O3 techmap/resub/sweep and must remain correct/stable.
    constexpr ::fast_io::u8string_view src = u8R"(
module top(input [3:0] a, input [3:0] b, output [4:0] y);
  // In this subset, '+' result width is max operand width; widen to keep carry-out.
  assign y = {1'b0, a} + {1'b0, b};
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
        .assume_binary_inputs = true,
        .opt_level = 4,
        .optimize_adders = true,
        .techmap_enable = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        std::fprintf(stderr, "pe_synth failed: %.*s\n", static_cast<int>(err.message.size()), reinterpret_cast<char const*>(err.message.data()));
        return 9;
    }
    if(!c.analyze()) { return 10; }

    std::size_t full_adders{};
    std::size_t half_adders{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            auto const n = m->ptr->get_model_name();
            if(n == u8"FULL_ADDER") { ++full_adders; }
            if(n == u8"HALF_ADDER") { ++half_adders; }
        }
    }
    if(full_adders == 0 || half_adders == 0)
    {
        std::fprintf(stderr, "adder macros missing after O3 pipeline: full=%zu half=%zu\n", full_adders, half_adders);
        return 11;
    }

    auto set_in = [&](::fast_io::u8string_view nm, bool v) noexcept
    {
        for(auto const& blk : nl.models)
        {
            for(auto* m = blk.begin; m != blk.curr; ++m)
            {
                if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                if(m->ptr->get_model_name() != u8"INPUT") { continue; }
                if(m->name != nm) { continue; }
                (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                                   : ::phy_engine::model::digital_node_statement_t::false_state));
                return;
            }
        }
    };

    auto settle = [&]() noexcept
    {
        for(std::size_t i{}; i < 8u; ++i) { c.digital_clk(); }
    };

    auto state_of = [&](::fast_io::u8string_view port_name) noexcept -> ::phy_engine::model::digital_node_statement_t
    {
        for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(pi);
            if(p.name != port_name) { continue; }
            return ports[pi]->node_information.dn.state;
        }
        return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
    };

    for(std::uint32_t a{}; a < 16u; ++a)
    {
        for(std::uint32_t b{}; b < 16u; ++b)
        {
            for(std::uint32_t i{}; i < 4u; ++i)
            {
                char8_t na[5]{u8'a', u8'[', static_cast<char8_t>(u8'0' + i), u8']', u8'\0'};
                char8_t nb[5]{u8'b', u8'[', static_cast<char8_t>(u8'0' + i), u8']', u8'\0'};
                set_in(::fast_io::u8string_view{na, 4}, ((a >> i) & 1u) != 0u);
                set_in(::fast_io::u8string_view{nb, 4}, ((b >> i) & 1u) != 0u);
            }
            settle();

            std::uint32_t y{};
            char y_bits[6]{};
            for(std::uint32_t i{}; i < 5u; ++i)
            {
                char8_t ny[5]{u8'y', u8'[', static_cast<char8_t>(u8'0' + i), u8']', u8'\0'};
                auto const st = state_of(::fast_io::u8string_view{ny, 4});
                y_bits[i] = dn_char(st);
                if(st == ::phy_engine::model::digital_node_statement_t::true_state) { y |= (1u << i); }
            }

            auto const exp = a + b;
            if(y != exp)
            {
                std::fprintf(stderr, "mismatch: a=%u b=%u y=%u expected=%u (y[4:0]=%c%c%c%c%c)\n",
                             a,
                             b,
                             y,
                             exp,
                             y_bits[4],
                             y_bits[3],
                             y_bits[2],
                             y_bits[1],
                             y_bits[0]);
                return 12;
            }
        }
    }

    return 0;
}
