#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
struct run_result
{
    std::size_t or_gates{};
};

std::size_t count_or_gates(::phy_engine::netlist::netlist const& nl) noexcept
{
    std::size_t gates{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() == u8"OR") { ++gates; }
        }
    }
    return gates;
}

std::optional<run_result> run_once(bool infer_dc) noexcept
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    constexpr ::fast_io::u8string_view src = u8R"(
module top(input clk, input rst_n, output y);
  reg [2:0] state;
  always_ff @(posedge clk or negedge rst_n) begin
    if(!rst_n) state <= 3'b001;
    else begin
      case(state)
        3'b001: state <= 3'b010;
        3'b010: state <= 3'b100;
        3'b100: state <= 3'b001;
        default: state <= 3'b001;
      endcase
    end
  end
  assign y = state[0] | state[1] | state[2];
endmodule
)";

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
        .opt_level = 3,
    };
    opt.infer_dc_from_fsm = infer_dc;

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        std::fprintf(stderr,
                     "pe_synth failed (infer_dc=%u): %.*s\n",
                     static_cast<unsigned>(infer_dc),
                     static_cast<int>(err.message.size()),
                     reinterpret_cast<char const*>(err.message.data()));
        return std::nullopt;
    }

    run_result rr{};
    rr.or_gates = count_or_gates(nl);
    return rr;
}
}  // namespace

int main()
{
    auto base = run_once(false);
    if(!base)
    {
        std::fprintf(stderr, "failed to build baseline netlist\n");
        return 2;
    }

    auto with_dc = run_once(true);
    if(!with_dc)
    {
        std::fprintf(stderr, "failed to build DC netlist\n");
        return 3;
    }

    if(!(with_dc->or_gates < base->or_gates))
    {
        std::fprintf(stderr,
                     "expected OR gate reduction with FSM DC (base=%zu dc=%zu)\n",
                     base->or_gates,
                     with_dc->or_gates);
        return 4;
    }

    return 0;
}
