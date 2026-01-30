#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <vector>

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

struct run_result
{
    std::size_t gate_count{};
    ::phy_engine::verilog::digital::pe_synth_report report{};
};

std::optional<run_result> run_once(std::uint8_t opt_level) noexcept
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    // A small combinational design where Omax should behave like O3 by default (no regressions).
    static constexpr ::fast_io::u8string_view src = u8R"(
module top(
  input  [2:0] a,
  input  [2:0] b,
  output       y
);
  // 3-term OR of ANDs; also contains a redundant term to exercise cleanup.
  wire t0;
  wire t1;
  wire t2;
  wire red;
  assign t0  = a[0] & b[0];
  assign t1  = a[1] & b[1];
  assign t2  = a[2] & b[2];
  assign red = (a[0] & b[0]) & 1'b1;
  assign y   = t0 | t1 | t2 | red;
endmodule
)";

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
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    // Attach INPUT/OUTPUT models to top-level ports.
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

    run_result rr{};
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = opt_level,
        .omax_max_iter = 2,       // keep the test fast
        .omax_timeout_ms = 0,     // deterministic for CI
        .omax_randomize = false,  // Omax should not be worse than O3 by default
        .omax_verify = (opt_level >= 4),
        .omax_verify_exact_max_inputs = 12,  // 6 inputs => exhaustive check (fast)
        .report_enable = true,
        .report = __builtin_addressof(rr.report),
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

    rr.gate_count = count_logic_gates(nl);

    if(!c.analyze()) { return std::nullopt; }

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

    auto read_out = [&](::fast_io::u8string_view nm) noexcept -> std::optional<bool>
    {
        for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(pi);
            if(p.name != nm) { continue; }
            auto const s = ports[pi]->node_information.dn.state;
            if(s == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
            if(s == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
            return std::nullopt;
        }
        return std::nullopt;
    };

    // Exhaustive check for 6 inputs.
    for(std::uint32_t m{}; m < 64u; ++m)
    {
        bool const a0 = (m & 1u) != 0u;
        bool const a1 = (m & 2u) != 0u;
        bool const a2 = (m & 4u) != 0u;
        bool const b0 = (m & 8u) != 0u;
        bool const b1 = (m & 16u) != 0u;
        bool const b2 = (m & 32u) != 0u;

        // Port names are bit-blasted during elaboration.
        set_in(u8"a[0]", a0);
        set_in(u8"a[1]", a1);
        set_in(u8"a[2]", a2);
        set_in(u8"b[0]", b0);
        set_in(u8"b[1]", b1);
        set_in(u8"b[2]", b2);
        settle();

        auto const y = read_out(u8"y");
        if(!y) { return std::nullopt; }
        bool const expected = (a0 && b0) || (a1 && b1) || (a2 && b2);
        if(*y != expected)
        {
            std::fprintf(stderr,
                         "mismatch (O%u): m=%u y=%u expected=%u\n",
                         static_cast<unsigned>(opt_level),
                         m,
                         static_cast<unsigned>(*y),
                         static_cast<unsigned>(expected));
            return std::nullopt;
        }
    }

    return rr;
}
}  // namespace

int main()
{
    auto const o3 = run_once(3);
    if(!o3) { return 1; }

    auto const omax = run_once(4);
    if(!omax) { return 2; }

    if(omax->gate_count > o3->gate_count)
    {
        std::fprintf(stderr, "Omax regressed: O3=%zu Omax=%zu\n", o3->gate_count, omax->gate_count);
        return 3;
    }

    if(omax->report.omax_best_gate_count.empty())
    {
        std::fprintf(stderr, "expected Omax report trace to be non-empty\n");
        return 4;
    }

    if(omax->report.omax_summary.empty())
    {
        std::fprintf(stderr, "expected Omax summary to be non-empty\n");
        return 5;
    }

    if(omax->report.omax_best_cost.empty())
    {
        std::fprintf(stderr, "expected Omax objective cost trace to be non-empty\n");
        return 6;
    }
    if(omax->report.omax_best_cost.size() != omax->report.omax_best_gate_count.size())
    {
        std::fprintf(stderr, "expected Omax cost trace to align with gate trace\n");
        return 7;
    }

    return 0;
}
