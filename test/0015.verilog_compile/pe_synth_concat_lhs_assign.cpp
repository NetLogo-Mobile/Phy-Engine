#include <array>
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
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module adder8 (
    input  wire [7:0] a,
    input  wire [7:0] b,
    input  wire       cin,
    output wire [7:0] sum,
    output wire       cout
);
    // Note: in this subset, '+' result width is max operand width (carry-out is discarded unless widened).
    assign {cout, sum} = {1'b0, a} + {1'b0, b} + cin;
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
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"adder8");
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

    auto port_idx = [&](::fast_io::u8string_view name) noexcept -> ::std::size_t
    {
        for(::std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(i);
            if(p.name == name) { return i; }
        }
        return SIZE_MAX;
    };

    using u8sv = ::fast_io::u8string_view;
    constexpr u8sv a_names[8]{u8"a[7]", u8"a[6]", u8"a[5]", u8"a[4]", u8"a[3]", u8"a[2]", u8"a[1]", u8"a[0]"};
    constexpr u8sv b_names[8]{u8"b[7]", u8"b[6]", u8"b[5]", u8"b[4]", u8"b[3]", u8"b[2]", u8"b[1]", u8"b[0]"};
    constexpr u8sv sum_names[8]{u8"sum[7]", u8"sum[6]", u8"sum[5]", u8"sum[4]", u8"sum[3]", u8"sum[2]", u8"sum[1]", u8"sum[0]"};

    std::array<std::size_t, 8> a_idx{};
    std::array<std::size_t, 8> b_idx{};
    std::array<std::size_t, 8> sum_idx{};
    for(std::size_t i{}; i < 8; ++i)
    {
        a_idx[i] = port_idx(a_names[i]);
        b_idx[i] = port_idx(b_names[i]);
        sum_idx[i] = port_idx(sum_names[i]);
    }

    auto const cin_idx = port_idx(u8"cin");
    auto const cout_idx = port_idx(u8"cout");
    if(cin_idx == SIZE_MAX || cout_idx == SIZE_MAX) { return 4; }
    for(auto v : a_idx) { if(v == SIZE_MAX) { return 5; } }
    for(auto v : b_idx) { if(v == SIZE_MAX) { return 6; } }
    for(auto v : sum_idx) { if(v == SIZE_MAX) { return 7; } }

    std::array<::phy_engine::model::model_base*, 8> in_a{};
    std::array<::phy_engine::model::model_base*, 8> in_b{};
    ::phy_engine::model::model_base* in_cin{};
    std::array<::phy_engine::model::model_base*, 8> out_sum{};
    ::phy_engine::model::model_base* out_cout{};

    for(auto& p : in_a)
    {
        auto [m, pos] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr) { return 8; }
        p = m;
    }
    for(auto& p : in_b)
    {
        auto [m, pos] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr) { return 9; }
        p = m;
    }
    {
        auto [m, pos] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr) { return 10; }
        in_cin = m;
    }
    for(auto& p : out_sum)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr) { return 11; }
        p = m;
    }
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr) { return 12; }
        out_cout = m;
    }

    for(std::size_t i{}; i < 8; ++i)
    {
        // Mark as external IO so pe_synth doesn't treat them as constant INPUTs.
        in_a[i]->name = ::fast_io::u8string{a_names[i]};
        in_b[i]->name = ::fast_io::u8string{b_names[i]};
        if(!::phy_engine::netlist::add_to_node(nl, *in_a[i], 0, *ports[a_idx[i]])) { return 13; }
        if(!::phy_engine::netlist::add_to_node(nl, *in_b[i], 0, *ports[b_idx[i]])) { return 14; }
        if(!::phy_engine::netlist::add_to_node(nl, *out_sum[i], 0, *ports[sum_idx[i]])) { return 15; }
    }
    in_cin->name = ::fast_io::u8string{u8"cin"};
    out_cout->name = ::fast_io::u8string{u8"cout"};
    for(std::size_t i{}; i < 8; ++i) { out_sum[i]->name = ::fast_io::u8string{sum_names[i]}; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_cin, 0, *ports[cin_idx])) { return 16; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_cout, 0, *ports[cout_idx])) { return 17; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 18; }
    if(!c.analyze()) { return 19; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) noexcept {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto set_inputs = [&](std::uint8_t a, std::uint8_t b, bool cin) noexcept
    {
        for(std::size_t bit{}; bit < 8; ++bit)
        {
            set_in(in_a[7 - bit], ((a >> bit) & 1u) != 0);
            set_in(in_b[7 - bit], ((b >> bit) & 1u) != 0);
        }
        set_in(in_cin, cin);
    };

    auto read_bit = [&](std::size_t pi) noexcept -> std::optional<bool> {
        auto const s = ports[pi]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
        if(s == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
        return std::nullopt;
    };

    auto read_u8 = [&]() noexcept -> std::optional<std::uint8_t> {
        std::uint8_t v{};
        for(std::size_t bit{}; bit < 8; ++bit)
        {
            auto b = read_bit(sum_idx[7 - bit]);
            if(!b) { return std::nullopt; }
            if(*b) { v |= static_cast<std::uint8_t>(1u << bit); }
        }
        return v;
    };

    auto settle = [&]() noexcept {
        for(::std::size_t i{}; i < 4; ++i) { c.digital_clk(); }
    };

    for(std::uint8_t a : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{7}, std::uint8_t{0x55}, std::uint8_t{0xFF}})
    {
        for(std::uint8_t b : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{3}, std::uint8_t{9}, std::uint8_t{0x0F}, std::uint8_t{0xFF}})
        {
            for(bool cin : {false, true})
            {
                set_inputs(a, b, cin);
                settle();

                auto const sum = read_u8();
                auto const cout = read_bit(cout_idx);
                if(!sum || !cout) { return 20; }

                std::uint16_t const ref =
                    static_cast<std::uint16_t>(a) + static_cast<std::uint16_t>(b) + static_cast<std::uint16_t>(cin ? 1u : 0u);
                if(*sum != static_cast<std::uint8_t>(ref & 0xFFu))
                {
                    std::fprintf(stderr,
                                 "sum mismatch: a=%u b=%u cin=%u sum=%u expected=%u\n",
                                 static_cast<unsigned>(a),
                                 static_cast<unsigned>(b),
                                 static_cast<unsigned>(cin),
                                 static_cast<unsigned>(*sum),
                                 static_cast<unsigned>(ref & 0xFFu));
                    return 21;
                }
                bool const exp_cout = (((ref >> 8) & 1u) != 0u);
                if(*cout != exp_cout)
                {
                    std::fprintf(stderr,
                                 "cout mismatch: a=%u b=%u cin=%u cout=%u expected=%u (ref=%u)\n",
                                 static_cast<unsigned>(a),
                                 static_cast<unsigned>(b),
                                 static_cast<unsigned>(cin),
                                 static_cast<unsigned>(*cout),
                                 static_cast<unsigned>(exp_cout),
                                 static_cast<unsigned>(ref));
                    return 22;
                }
            }
        }
    }

    return 0;
}
