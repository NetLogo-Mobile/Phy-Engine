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

struct run_result
{
    std::size_t gate_count{};
    ::phy_engine::verilog::digital::pe_synth_report report{};
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
        .techmap_enable = false,   // isolate decomposition behavior
        .decomp_min_vars = 11,     // exact leaf count in this test
        .decomp_max_vars = 16,
        .decomp_max_gates = 4096,  // allow large SOP cones
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

    // Exhaustive check: y == d[sel].
    for(std::uint32_t m{}; m < (1u << 11); ++m)
    {
        bool const s0 = (m & (1u << 0)) != 0u;
        bool const s1 = (m & (1u << 1)) != 0u;
        bool const s2 = (m & (1u << 2)) != 0u;
        std::uint32_t const sel = (static_cast<std::uint32_t>(s2) << 2u) | (static_cast<std::uint32_t>(s1) << 1u) | static_cast<std::uint32_t>(s0);

        set_in(u8"s0", s0);
        set_in(u8"s1", s1);
        set_in(u8"s2", s2);
        for(std::uint32_t i{}; i < 8u; ++i)
        {
            bool const dvv = (m & (1u << (3u + i))) != 0u;
            // names: d0..d7
            char8_t name_buf[3]{u8'd', static_cast<char8_t>(u8'0' + i), u8'\0'};
            set_in(::fast_io::u8string_view{name_buf, 2}, dvv);
        }
        settle();

        auto const y = read_out(u8"y");
        if(!y) { return std::nullopt; }

        bool expected{};
        {
            // d_i are packed after s0..s2 in the loop above.
            expected = (m & (1u << (3u + sel))) != 0u;
        }

        if(*y != expected)
        {
            std::fprintf(stderr,
                         "mismatch (O%u): sel=%u y=%u expected=%u\n",
                         static_cast<unsigned>(opt_level),
                         sel,
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
    // An 8:1 mux written as a big SOP, plus per-term duplicated negations to avoid CSE creating shared ~s* leaves.
    constexpr ::fast_io::u8string_view src = u8R"(
module top(
    input wire s0,
    input wire s1,
    input wire s2,
    input wire d0,
    input wire d1,
    input wire d2,
    input wire d3,
    input wire d4,
    input wire d5,
    input wire d6,
    input wire d7,
    output wire y
);
    wire ns0_0, ns0_1, ns0_2, ns0_3;
    wire ns1_0, ns1_1, ns1_2, ns1_3;
    wire ns2_0, ns2_1, ns2_2, ns2_3;
    assign ns0_0 = ~s0;
    assign ns0_1 = ~s0;
    assign ns0_2 = ~s0;
    assign ns0_3 = ~s0;
    assign ns1_0 = ~s1;
    assign ns1_1 = ~s1;
    assign ns1_2 = ~s1;
    assign ns1_3 = ~s1;
    assign ns2_0 = ~s2;
    assign ns2_1 = ~s2;
    assign ns2_2 = ~s2;
    assign ns2_3 = ~s2;

    assign y = (ns2_0 & ns1_0 & ns0_0 & d0) |
               (ns2_1 & ns1_1 & s0    & d1) |
               (ns2_2 & s1    & ns0_1 & d2) |
               (ns2_3 & s1    & s0    & d3) |
               (s2    & ns1_2 & ns0_2 & d4) |
               (s2    & ns1_3 & s0    & d5) |
               (s2    & s1    & ns0_3 & d6) |
               (s2    & s1    & s0    & d7);
endmodule
)";

    auto r2 = run_once(src, 2);
    if(!r2) { return 1; }
    auto r3 = run_once(src, 3);
    if(!r3) { return 1; }

    if(r3->gate_count > r2->gate_count)
    {
        std::fprintf(stderr, "expected O3 to not increase gates via decomposition (O2=%zu, O3=%zu)\n", r2->gate_count, r3->gate_count);
        return 2;
    }

    bool saw_decomp{};
    for(auto const& ps : r3->report.passes)
    {
        if(ps.pass != u8"bdd_decompose") { continue; }
        saw_decomp = true;
    }
    if(!saw_decomp)
    {
        std::fprintf(stderr, "expected bdd_decompose pass to run\n");
        return 3;
    }

    return 0;
}
