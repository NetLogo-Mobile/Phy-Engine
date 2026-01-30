#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

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

struct run_result
{
    std::size_t gate_count{};
};

std::size_t count_logic_gates(::phy_engine::netlist::netlist const& nl) noexcept
{
    std::size_t gates{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            auto const n = m->ptr->get_model_name();
            if(n == u8"AND" || n == u8"OR" || n == u8"XOR" || n == u8"XNOR" || n == u8"NOT" || n == u8"NAND" || n == u8"NOR" || n == u8"IMP" ||
               n == u8"NIMP" || n == u8"YES")
            {
                ++gates;
            }
        }
    }
    return gates;
}

std::optional<run_result> run_once(::fast_io::u8string_view src, std::uint8_t opt_level) noexcept
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return std::nullopt; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"top");
    if(mod == nullptr) { return std::nullopt; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return std::nullopt; }

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
            if(m == nullptr || m->ptr == nullptr) { return std::nullopt; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return std::nullopt; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return std::nullopt; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return std::nullopt; }
        }
        else
        {
            return std::nullopt;
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = false,  // Ensure the flattening pass is safe under 4-valued semantics.
        .opt_level = opt_level,
    };

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        std::fprintf(stderr,
                     "pe_synth failed (O%u): %.*s\n",
                     static_cast<unsigned>(opt_level),
                     static_cast<int>(err.message.size()),
                     reinterpret_cast<char const*>(err.message.data()));
        return std::nullopt;
    }

    if(!c.analyze()) { return std::nullopt; }

    auto port_index = [&](::fast_io::u8string_view name) noexcept -> std::optional<std::size_t>
    {
        for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            if(p.name == name) { return i; }
        }
        return std::nullopt;
    };

    auto idx_a = port_index(u8"a");
    auto idx_b = port_index(u8"b");
    auto idx_y = port_index(u8"y");
    if(!idx_a || !idx_b || !idx_y) { return std::nullopt; }

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
        for(std::size_t i{}; i < 4u; ++i) { c.digital_clk(); }
    };

    auto read_out = [&](std::size_t idx) noexcept -> std::optional<bool>
    {
        auto const s = ports[idx]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
        if(s == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
        return std::nullopt;
    };

    for(std::uint32_t mask{}; mask < 4u; ++mask)
    {
        bool const a = (mask & 0x01u) != 0;
        bool const b = (mask & 0x02u) != 0;
        set_in(u8"a", a);
        set_in(u8"b", b);
        settle();

        auto const y = read_out(*idx_y);
        if(!y) { return std::nullopt; }
        if(*y != (a && b)) { return std::nullopt; }
    }

    run_result rr{};
    rr.gate_count = count_logic_gates(nl);
    return rr;
}
}  // namespace

int main()
{
    // Purpose: (a & b) & a should flatten to (a & b), reducing one AND gate (even though the duplicate "a" is not adjacent).
    constexpr ::fast_io::u8string_view src = u8R"(
module top(
  input a,
  input b,
  output y
);
  assign y = (a & b) & a;
endmodule
)";

    auto const r1 = run_once(src, 1);
    auto const r2 = run_once(src, 2);
    if(!r1 || !r2) { return 1; }

    if(r2->gate_count >= r1->gate_count)
    {
        std::fprintf(stderr, "expected O2 gate_count < O1 (O1=%zu O2=%zu)\n", r1->gate_count, r2->gate_count);
        return 2;
    }

    return 0;
}

