#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
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

struct build_result
{
    ::phy_engine::circult c{};
    ::std::vector<::phy_engine::model::node_t*> ports{};
    ::phy_engine::verilog::digital::instance_state top{};
};

bool build_o0_netlist(build_result& br) noexcept
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    // The function is explicitly listed as 4 minterms (b=0,d=0, a/c any), so QM should collapse it to ~b & ~d.
    static constexpr ::fast_io::u8string_view src = u8R"(
module top(
  input  a,
  input  b,
  input  c,
  input  d,
  output y
);
  // Duplicate each complemented literal through distinct wires so the synthesized NOT outputs have fanout=1.
  // (The QM pass requires internal nets to be exclusive to the cone for safe deletion.)
  wire a0, a1;
  wire c0, c1;
  wire b0, b1, b2, b3;
  wire d0, d1, d2, d3;
  assign a0 = a;
  assign a1 = a;
  assign c0 = c;
  assign c1 = c;
  assign b0 = b;
  assign b1 = b;
  assign b2 = b;
  assign b3 = b;
  assign d0 = d;
  assign d1 = d;
  assign d2 = d;
  assign d3 = d;

  wire na0, na1;
  wire nc0, nc1;
  wire nb0, nb1, nb2, nb3;
  wire nd0, nd1, nd2, nd3;
  assign na0 = ~a0;
  assign na1 = ~a1;
  assign nc0 = ~c0;
  assign nc1 = ~c1;
  assign nb0 = ~b0;
  assign nb1 = ~b1;
  assign nb2 = ~b2;
  assign nb3 = ~b3;
  assign nd0 = ~d0;
  assign nd1 = ~d1;
  assign nd2 = ~d2;
  assign nd3 = ~d3;

  wire t0, t1, t2, t3;
  assign t0 = na0 & nb0 & nc0 & nd0; // a=0,c=0
  assign t1 = na1 & nb1 & c1  & nd1; // a=0,c=1
  assign t2 = a   & nb2 & nc1 & nd2; // a=1,c=0
  assign t3 = a   & nb3 & c   & nd3; // a=1,c=1
  assign y  = t0 | t1 | t2 | t3;
endmodule
)";

    br.c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = br.c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = br.c.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return false; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"top");
    if(mod == nullptr) { return false; }
    br.top = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(br.top.mod == nullptr) { return false; }

    br.ports.reserve(br.top.mod->ports.size());
    for(std::size_t i{}; i < br.top.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        br.ports.push_back(__builtin_addressof(n));
    }

    // Attach INPUT/OUTPUT models to top-level ports.
    for(std::size_t pi{}; pi < br.top.mod->ports.size(); ++pi)
    {
        auto const& p = br.top.mod->ports.index_unchecked(pi);
        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return false; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *br.ports[pi])) { return false; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return false; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *br.ports[pi])) { return false; }
        }
        else
        {
            return false;
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .assume_binary_inputs = true,
        .opt_level = 0,
    };

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, br.top, br.ports, &err, opt))
    {
        std::fprintf(stderr, "pe_synth O0 failed: %.*s\n", static_cast<int>(err.message.size()), reinterpret_cast<char const*>(err.message.data()));
        return false;
    }

    return true;
}
}  // namespace

int main() noexcept
{
    using namespace phy_engine::verilog::digital;

    // Enabled: QM should reduce the explicit sum-of-minterms into ~b & ~d.
    {
        build_result br{};
        if(!build_o0_netlist(br)) { return 1; }
        auto& nl = br.c.get_netlist();
        ::phy_engine::verilog::digital::details::optimize_eliminate_yes_buffers(nl, br.ports);
        auto const before = count_logic_gates(nl);

        pe_synth_options opt{};
        opt.assume_binary_inputs = true;
        opt.qm_max_vars = 10;
        opt.qm_max_gates = 128;
        bool const changed =
            ::phy_engine::verilog::digital::details::optimize_qm_two_level_minimize_in_pe_netlist(nl, br.ports, opt, nullptr);
        auto const after = count_logic_gates(nl);

        if(!changed || after >= before)
        {
            std::fprintf(stderr, "qm enabled: changed=%d before=%zu after=%zu\n", changed ? 1 : 0, before, after);
            return 2;
        }
    }

    // Disabled via per-pass budget: should become a no-op.
    {
        build_result br{};
        if(!build_o0_netlist(br)) { return 3; }
        auto& nl = br.c.get_netlist();
        ::phy_engine::verilog::digital::details::optimize_eliminate_yes_buffers(nl, br.ports);
        auto const before = count_logic_gates(nl);

        pe_synth_options opt{};
        opt.assume_binary_inputs = true;
        opt.qm_max_vars = 0;  // disables QM/Espresso
        bool const changed =
            ::phy_engine::verilog::digital::details::optimize_qm_two_level_minimize_in_pe_netlist(nl, br.ports, opt, nullptr);
        auto const after = count_logic_gates(nl);

        if(changed || after != before)
        {
            std::fprintf(stderr, "qm disabled: changed=%d before=%zu after=%zu\n", changed ? 1 : 0, before, after);
            return 4;
        }
    }

    return 0;
}
