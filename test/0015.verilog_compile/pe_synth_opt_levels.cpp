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

    // Create INPUT/OUTPUT models for each bit port.
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

    run_result rr{};
    rr.gate_count = count_logic_gates(nl);

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
    auto idx_c = port_index(u8"c");
    auto idx_d = port_index(u8"d");
    auto idx_e = port_index(u8"e");
    auto idx_f = port_index(u8"f");
    auto idx_g = port_index(u8"g");
    auto idx_y1 = port_index(u8"y1");
    auto idx_y2 = port_index(u8"y2");
    auto idx_y3 = port_index(u8"y3");
    if(!idx_a || !idx_b || !idx_c || !idx_d || !idx_e || !idx_f || !idx_g || !idx_y1 || !idx_y2 || !idx_y3) { return std::nullopt; }

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

    // Exhaustive 2-valued check for all 5 inputs.
    for(std::uint32_t mask{}; mask < 128u; ++mask)
    {
        bool const a = (mask & 0x01u) != 0;
        bool const b = (mask & 0x02u) != 0;
        bool const c_in = (mask & 0x04u) != 0;
        bool const d = (mask & 0x08u) != 0;
        bool const e = (mask & 0x10u) != 0;
        bool const f = (mask & 0x20u) != 0;
        bool const g = (mask & 0x40u) != 0;

        set_in(u8"a", a);
        set_in(u8"b", b);
        set_in(u8"c", c_in);
        set_in(u8"d", d);
        set_in(u8"e", e);
        set_in(u8"f", f);
        set_in(u8"g", g);
        settle();

        auto const y1 = read_out(*idx_y1);
        auto const y2 = read_out(*idx_y2);
        auto const y3 = read_out(*idx_y3);
        if(!y1 || !y2 || !y3)
        {
            std::fprintf(stderr, "non-binary output at O%u (mask=%u)\n", static_cast<unsigned>(opt_level), mask);
            return std::nullopt;
        }

        bool const exp_y1 = (a && b) || (a && c_in);
        bool const exp_y2 = !(d && e);
        bool const exp_y3 = f;
        if(*y1 != exp_y1 || *y2 != exp_y2 || *y3 != exp_y3)
        {
            std::fprintf(stderr,
                         "mismatch at O%u (mask=%u): got y1=%d y2=%d y3=%d, expected y1=%d y2=%d y3=%d\n",
                         static_cast<unsigned>(opt_level),
                         mask,
                         *y1 ? 1 : 0,
                         *y2 ? 1 : 0,
                         *y3 ? 1 : 0,
                         exp_y1 ? 1 : 0,
                         exp_y2 ? 1 : 0,
                         exp_y3 ? 1 : 0);
            return std::nullopt;
        }
    }

    return rr;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module top(input a, input b, input c, input d, input e, input f, input g, output y1, output y2, output y3);
  // factoring target: (a&b) | (a&c) => a & (b|c)
  assign y1 = (a & b) | (a & c);
  // inverter-fusion target: ~(d&e) => NAND(d,e)
  assign y2 = ~(d & e);
  // QM target: (f&g)|(f&~g) => f
  assign y3 = (f & g) | (f & ~g);
endmodule
)";

    auto const o0 = run_once(src, 0);
    auto const o1 = run_once(src, 1);
    auto const o2 = run_once(src, 2);
    auto const o3 = run_once(src, 3);
    if(!o0 || !o1 || !o2 || !o3) { return 1; }

    if(!(o0->gate_count > o1->gate_count))
    {
        std::fprintf(stderr, "expected O1 to reduce gates: O0=%zu O1=%zu\n", o0->gate_count, o1->gate_count);
        return 2;
    }
    if(!(o1->gate_count > o2->gate_count))
    {
        std::fprintf(stderr, "expected O2 to reduce gates: O1=%zu O2=%zu\n", o1->gate_count, o2->gate_count);
        return 3;
    }
    if(!(o2->gate_count > o3->gate_count))
    {
        std::fprintf(stderr, "expected O3 to reduce gates: O2=%zu O3=%zu\n", o2->gate_count, o3->gate_count);
        return 4;
    }

    return 0;
}
