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

struct run_result
{
    bool ok{};
    std::uint16_t out{};
    std::size_t mul2s{};
    std::size_t unknown_out_bits{};
    std::size_t x_out_bits{};
    std::size_t z_out_bits{};
    std::uint16_t x_mask{};
};

run_result run_once(::fast_io::u8string_view src, bool enable_mul2_opt) noexcept
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
    if(!cr.errors.empty() || cr.modules.empty()) { return {}; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"mul8x8");
    if(mod == nullptr) { return {}; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return {}; }

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
            if(m == nullptr || m->ptr == nullptr) { return {}; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return {}; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return {}; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return {}; }
        }
        else
        {
            return {};
        }
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_mul2 = enable_mul2_opt,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        std::fprintf(stderr,
                     "pe_synth failed (optimize_mul2=%d): %.*s\n",
                     enable_mul2_opt ? 1 : 0,
                     static_cast<int>(err.message.size()),
                     reinterpret_cast<char const*>(err.message.data()));
        return {};
    }
    if(!c.analyze()) { return {}; }

    auto settle = [&]() noexcept
    {
        // Digital combinational networks are expected to settle in one tick, but run a few cycles to be safe.
        for(::std::size_t i{}; i < 8u; ++i) { c.digital_clk(); }
    };

    // Drive a=3 (0000_0011), b=7 (0000_0111) => p=21.
    for(auto const& blk : nl.models)
    {
        for(auto* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() != u8"INPUT") { continue; }
            auto const nm = m->name;
            if(nm == u8"a[0]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
            if(nm == u8"a[1]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
            if(nm == u8"b[0]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
            if(nm == u8"b[1]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
            if(nm == u8"b[2]") { (void)m->ptr->set_attribute(0, dv(::phy_engine::model::digital_node_statement_t::true_state)); }
        }
    }
    settle();

    auto bit_of = [&](::fast_io::u8string_view port_name) noexcept -> ::phy_engine::model::digital_node_statement_t
    {
        for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(pi);
            if(p.name == port_name) { return ports[pi]->node_information.dn.state; }
        }
        return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
    };
    auto has_port = [&](::fast_io::u8string_view port_name) noexcept -> bool
    {
        for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
        {
            auto const& p = top_inst.mod->ports.index_unchecked(pi);
            if(p.name == port_name) { return true; }
        }
        return false;
    };

    run_result rr{.ok = true};

    // Input sanity: ensure the driven inputs actually reached the port nodes.
    constexpr ::fast_io::u8string_view a_ports[8]{u8"a[0]", u8"a[1]", u8"a[2]", u8"a[3]", u8"a[4]", u8"a[5]", u8"a[6]", u8"a[7]"};
    constexpr ::fast_io::u8string_view b_ports[8]{u8"b[0]", u8"b[1]", u8"b[2]", u8"b[3]", u8"b[4]", u8"b[5]", u8"b[6]", u8"b[7]"};

    for(std::size_t bit{}; bit < 8u; ++bit)
    {
        if(!has_port(a_ports[bit]) || !has_port(b_ports[bit]))
        {
            std::fprintf(stderr, "missing expected port bit: a[%zu]=%d b[%zu]=%d\n", bit, has_port(a_ports[bit]) ? 1 : 0, bit, has_port(b_ports[bit]) ? 1 : 0);
            return {};
        }
        auto const as = bit_of(a_ports[bit]);
        auto const bs = bit_of(b_ports[bit]);
        if(as != ::phy_engine::model::digital_node_statement_t::false_state && as != ::phy_engine::model::digital_node_statement_t::true_state)
        {
            std::fprintf(stderr, "input a[%zu] is not 0/1 (optimize_mul2=%d): %d\n", bit, enable_mul2_opt ? 1 : 0, static_cast<int>(as));
            return {};
        }
        if(bs != ::phy_engine::model::digital_node_statement_t::false_state && bs != ::phy_engine::model::digital_node_statement_t::true_state)
        {
            std::fprintf(stderr, "input b[%zu] is not 0/1 (optimize_mul2=%d): %d\n", bit, enable_mul2_opt ? 1 : 0, static_cast<int>(bs));
            return {};
        }
    }

    if(bit_of(u8"a[0]") != ::phy_engine::model::digital_node_statement_t::true_state ||
       bit_of(u8"a[1]") != ::phy_engine::model::digital_node_statement_t::true_state ||
       bit_of(u8"b[0]") != ::phy_engine::model::digital_node_statement_t::true_state ||
       bit_of(u8"b[1]") != ::phy_engine::model::digital_node_statement_t::true_state ||
       bit_of(u8"b[2]") != ::phy_engine::model::digital_node_statement_t::true_state)
    {
        std::fprintf(stderr, "expected driven input bits not high (optimize_mul2=%d)\n", enable_mul2_opt ? 1 : 0);
        return {};
    }

    // Const-driver sanity (best-effort): ensure constant INPUT nodes drive their configured literal value.
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() != u8"INPUT") { continue; }
            if(!m->name.empty()) { continue; }  // only constants synthesized by pe_synth
            auto pv = m->ptr->generate_pin_view();
            if(pv.size != 1 || pv.pins[0].nodes == nullptr) { continue; }
            auto const s_node = pv.pins[0].nodes->node_information.dn.state;
            auto const vi = m->ptr->get_attribute(0);
            if(vi.type != ::phy_engine::model::variant_type::digital)
            {
                std::fprintf(stderr, "const INPUT node has non-digital attribute\n");
                return {};
            }
            auto const s_attr = vi.digital;
            if(s_node != s_attr)
            {
                std::fprintf(stderr,
                             "const INPUT mismatch: attr=%d node=%d\n",
                             static_cast<int>(s_attr),
                             static_cast<int>(s_node));
                return {};
            }
        }
    }

    auto parse_index = [](::fast_io::u8string_view name) noexcept -> ::std::optional<std::size_t>
    {
        if(name.size() < 4) { return ::std::nullopt; }  // "p[0]"
        if(name[0] != u8'p') { return ::std::nullopt; }
        if(name[1] != u8'[') { return ::std::nullopt; }
        std::size_t i{2};
        std::size_t v{};
        bool any{};
        for(; i < name.size(); ++i)
        {
            auto const c = name[i];
            if(c == u8']') { break; }
            if(c < u8'0' || c > u8'9') { return ::std::nullopt; }
            any = true;
            v = v * 10u + static_cast<std::size_t>(c - u8'0');
        }
        if(!any) { return ::std::nullopt; }
        if(i >= name.size() || name[i] != u8']') { return ::std::nullopt; }
        return v;
    };

    std::size_t seen_p_bits{};
    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        if(p.dir != port_dir::output) { continue; }
        if(p.name.empty() || p.name[0] != u8'p') { continue; }
        auto const idx_opt = parse_index(::fast_io::u8string_view{p.name.data(), p.name.size()});
        if(!idx_opt) { continue; }
        auto const bit = *idx_opt;
        if(bit >= 16u) { continue; }

        ++seen_p_bits;
        auto const s = ports[pi]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::true_state)
        {
            rr.out |= static_cast<std::uint16_t>(1u << bit);
        }
        else if(s != ::phy_engine::model::digital_node_statement_t::false_state)
        {
            ++rr.unknown_out_bits;
            if(s == ::phy_engine::model::digital_node_statement_t::indeterminate_state) { ++rr.x_out_bits; }
            if(s == ::phy_engine::model::digital_node_statement_t::high_impedence_state) { ++rr.z_out_bits; }
            if(s == ::phy_engine::model::digital_node_statement_t::indeterminate_state) { rr.x_mask |= static_cast<std::uint16_t>(1u << bit); }
        }
    }
    if(seen_p_bits != 16u)
    {
        std::fprintf(stderr, "unexpected number of output bits: %zu\n", seen_p_bits);
        return {};
    }

    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() == u8"MUL2") { ++rr.mul2s; }
        }
    }

    return rr;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module mul8x8 (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    output wire [15:0] p
);
    assign p = a * b;
endmodule
)";

    // Baseline: gate-level multiplier must produce the correct result.
    auto const plain = run_once(src, false);
    if(!plain.ok)
    {
        std::fprintf(stderr, "mul8x8 baseline pe_synth failed\n");
        return 10;
    }
    if(plain.out != 21u || plain.unknown_out_bits != 0u)
    {
        std::fprintf(stderr,
                     "mul8x8 baseline mismatch: got=%u expected=21 (unknown_out_bits=%zu X=%zu Z=%zu x_mask=0x%04x)\n",
                     static_cast<unsigned>(plain.out),
                     plain.unknown_out_bits,
                     plain.x_out_bits,
                     plain.z_out_bits,
                     static_cast<unsigned>(plain.x_mask));
        return 11;
    }

    // Optimized: should contain MUL2 tiles and produce correct result.
    auto const opt = run_once(src, true);
    if(!opt.ok)
    {
        std::fprintf(stderr, "mul8x8 MUL2-opt failed\n");
        return 1;
    }
    if(opt.mul2s < 16u)
    {
        std::fprintf(stderr, "MUL2 tiles missing: MUL2=%zu\n", opt.mul2s);
        return 2;
    }
    if(opt.out != 21u)
    {
        std::fprintf(stderr,
                     "mul8x8 MUL2-opt mismatch: got=%u expected=21 (unknown_out_bits=%zu X=%zu Z=%zu x_mask=0x%04x)\n",
                     static_cast<unsigned>(opt.out),
                     opt.unknown_out_bits,
                     opt.x_out_bits,
                     opt.z_out_bits,
                     static_cast<unsigned>(opt.x_mask));
        return 3;
    }
    if(opt.unknown_out_bits != 0u)
    {
        std::fprintf(stderr,
                     "mul8x8 has unknown output bits: %zu (X=%zu Z=%zu x_mask=0x%04x)\n",
                     opt.unknown_out_bits,
                     opt.x_out_bits,
                     opt.z_out_bits,
                     static_cast<unsigned>(opt.x_mask));
        return 4;
    }

    return 0;
}
