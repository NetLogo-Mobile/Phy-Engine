#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
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

constexpr std::uint16_t expect_shift_sticky(std::uint16_t in15) noexcept
{
    in15 &= 0x7FFFu;
    std::uint16_t out = static_cast<std::uint16_t>(in15 >> 1);
    std::uint16_t const b0 = in15 & 1u;
    std::uint16_t const b1 = (in15 >> 1) & 1u;
    if((b0 | b1) != 0) { out |= 1u; }
    return out;
}
}  // namespace

int main()
{
    // Regression: PE synthesis/sim must respect procedural sequencing for:
    //   x = (x >> 1); x[0] = x[0] | sticky;
    decltype(auto) src = u8R"(
module top(input [14:0] in, output [14:0] out);
  reg [14:0] x;
  reg sticky;
  always @* begin
    x = in;
    sticky = x[0];
    x = (x >> 1);
    x[0] = x[0] | sticky;
  end
  assign out = x;
endmodule
)";

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

    auto port_idx = [&](::std::string_view name) noexcept -> ::std::size_t {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            ::std::string_view pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
            if(pn == name) { return i; }
        }
        return SIZE_MAX;
    };

    ::std::array<::phy_engine::model::model_base*, 15> in_bits{};
    for(auto& p : in_bits)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr) { return 4; }
        p = m;
    }
    ::std::array<::phy_engine::model::model_base*, 15> out_bits{};
    for(auto& p : out_bits)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr) { return 5; }
        p = m;
    }

    ::std::array<::std::size_t, 15> pi_in{};
    ::std::array<::std::size_t, 15> pi_out{};
    for(std::size_t i{}; i < 15; ++i)
    {
        auto const name_in = "in[" + ::std::to_string(i) + "]";
        auto const name_out = "out[" + ::std::to_string(i) + "]";
        pi_in[i] = port_idx(name_in);
        pi_out[i] = port_idx(name_out);
        if(pi_in[i] == SIZE_MAX || pi_out[i] == SIZE_MAX) { return 6; }
        if(!::phy_engine::netlist::add_to_node(nl, *in_bits[i], 0, *ports[pi_in[i]])) { return 7; }
        if(!::phy_engine::netlist::add_to_node(nl, *out_bits[i], 0, *ports[pi_out[i]])) { return 8; }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 9; }
    if(!c.analyze()) { return 10; }

    auto set_in = [&](std::size_t i, bool bit) noexcept {
        (void)in_bits[i]->ptr->set_attribute(0, dv(bit ? ::phy_engine::model::digital_node_statement_t::true_state
                                                       : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto settle = [&]() noexcept
    {
        c.digital_clk();
        c.digital_clk();
    };

    auto run_case = [&](std::uint16_t in15) -> int {
        for(std::size_t i{}; i < 15; ++i) { set_in(i, ((in15 >> i) & 1u) != 0u); }
        settle();

        auto const exp = expect_shift_sticky(in15);
        for(std::size_t i{}; i < 15; ++i)
        {
            auto const s = ports[pi_out[i]]->node_information.dn.state;
            if(s != ::phy_engine::model::digital_node_statement_t::false_state &&
               s != ::phy_engine::model::digital_node_statement_t::true_state)
            {
                return 12;
            }
            bool const got = (s == ::phy_engine::model::digital_node_statement_t::true_state);
            bool const want = ((exp >> i) & 1u) != 0u;
            if(got != want) { return 13; }
        }
        return 0;
    };

    if(int rc = run_case(0x0001u)) { return rc; }
    if(int rc = run_case(0x0002u)) { return rc; }
    if(int rc = run_case(0x1234u)) { return rc; }
    if(int rc = run_case(0x7FFFu)) { return rc; }

    return 0;
}
