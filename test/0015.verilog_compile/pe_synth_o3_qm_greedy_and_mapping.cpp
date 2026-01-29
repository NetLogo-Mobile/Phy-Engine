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

std::size_t count_model(::phy_engine::netlist::netlist const& nl, ::fast_io::u8string_view name) noexcept
{
    std::size_t cnt{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() == name) { ++cnt; }
        }
    }
    return cnt;
}

struct run_result
{
    bool ok{};
    std::size_t gate_count{};
    std::size_t nots{};
    std::size_t imps{};
    std::size_t nimps{};
    std::size_t xnors{};
};

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
        .assume_binary_inputs = true,
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

    constexpr ::fast_io::u8string_view inputs[10]{u8"a", u8"b", u8"c", u8"d", u8"e", u8"f", u8"g", u8"h", u8"p", u8"q"};
    for(auto const nm : inputs)
    {
        if(!port_index(nm)) { return std::nullopt; }
    }
    auto idx_y_big = port_index(u8"y_big");
    auto idx_y_imp = port_index(u8"y_imp");
    auto idx_y_nimp = port_index(u8"y_nimp");
    auto idx_y_xnor = port_index(u8"y_xnor");
    if(!idx_y_big || !idx_y_imp || !idx_y_nimp || !idx_y_xnor) { return std::nullopt; }

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

    // Exhaustive 2-valued check for all 10 inputs (1024 vectors).
    for(std::uint32_t mask{}; mask < 1024u; ++mask)
    {
        bool const a = (mask & 0x001u) != 0;
        bool const b = (mask & 0x002u) != 0;
        bool const c_in = (mask & 0x004u) != 0;
        bool const d = (mask & 0x008u) != 0;
        bool const e = (mask & 0x010u) != 0;
        bool const f = (mask & 0x020u) != 0;
        bool const g = (mask & 0x040u) != 0;
        bool const h = (mask & 0x080u) != 0;
        bool const p = (mask & 0x100u) != 0;
        bool const q = (mask & 0x200u) != 0;

        set_in(u8"a", a);
        set_in(u8"b", b);
        set_in(u8"c", c_in);
        set_in(u8"d", d);
        set_in(u8"e", e);
        set_in(u8"f", f);
        set_in(u8"g", g);
        set_in(u8"h", h);
        set_in(u8"p", p);
        set_in(u8"q", q);
        settle();

        auto const y_big = read_out(*idx_y_big);
        auto const y_imp = read_out(*idx_y_imp);
        auto const y_nimp = read_out(*idx_y_nimp);
        auto const y_xnor = read_out(*idx_y_xnor);
        if(!y_big || !y_imp || !y_nimp || !y_xnor)
        {
            std::fprintf(stderr, "non-binary output at O%u (mask=%u)\n", static_cast<unsigned>(opt_level), mask);
            return std::nullopt;
        }

        bool const exp_big = (a && b && c_in && d && e && f && g && h) || (a && b && c_in && d && e && f && g && !h);
        bool const exp_imp = (!p) || q;
        bool const exp_nimp = a && (!b);
        bool const exp_xnor = !(p ^ q);
        if(*y_big != exp_big || *y_imp != exp_imp || *y_nimp != exp_nimp || *y_xnor != exp_xnor)
        {
            std::fprintf(stderr, "mismatch at O%u (mask=%u)\n", static_cast<unsigned>(opt_level), mask);
            return std::nullopt;
        }
    }

    run_result rr{};
    rr.ok = true;
    rr.gate_count = count_logic_gates(nl);
    rr.nots = count_model(nl, u8"NOT");
    rr.imps = count_model(nl, u8"IMP");
    rr.nimps = count_model(nl, u8"NIMP");
    rr.xnors = count_model(nl, u8"XNOR");
    return rr;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module top(input a, input b, input c, input d, input e, input f, input g, input h, input p, input q,
           output y_big, output y_imp, output y_nimp, output y_xnor);
  // 8-var cone: (ABCDEFGH) | (ABCDEFG~H) => ABCDEFG (QM greedy path)
  assign y_big = (a&b&c&d&e&f&g&h) | (a&b&c&d&e&f&g&~h);

  // input-inverter mapping targets (expect NOT gates to be eliminated at O2+)
  assign y_imp  = (~p) | q;     // => IMP(p,q)
  assign y_nimp = a & (~b);     // => NIMP(a,b)
  assign y_xnor = p ^ (~q);     // => XNOR(p,q)
endmodule
)";

    auto const o1 = run_once(src, 1);
    auto const o2 = run_once(src, 2);
    auto const o3 = run_once(src, 3);
    if(!o1 || !o2 || !o3) { return 1; }

    if(!(o1->gate_count > o2->gate_count))
    {
        std::fprintf(stderr, "expected O2 to reduce gates: O1=%zu O2=%zu\n", o1->gate_count, o2->gate_count);
        return 2;
    }
    if(!(o2->gate_count > o3->gate_count))
    {
        std::fprintf(stderr, "expected O3 to reduce gates (QM/greedy): O2=%zu O3=%zu\n", o2->gate_count, o3->gate_count);
        return 3;
    }

    if(o2->nots != 0)
    {
        std::fprintf(stderr, "expected O2 to eliminate NOT gates via mapping, got NOT=%zu\n", o2->nots);
        return 4;
    }
    if(o2->imps == 0 || o2->nimps == 0 || o2->xnors == 0)
    {
        std::fprintf(stderr, "expected O2 to use IMP/NIMP/XNOR: IMP=%zu NIMP=%zu XNOR=%zu\n", o2->imps, o2->nimps, o2->xnors);
        return 5;
    }

    return 0;
}
