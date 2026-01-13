#pragma once

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <random>
#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

namespace fp8_test
{
inline ::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}

inline std::string read_file_text(std::filesystem::path const& path)
{
    std::ifstream ifs(path, std::ios::binary);
    if(!ifs.is_open()) { throw std::runtime_error("failed to open: " + path.string()); }
    std::string s;
    ifs.seekg(0, std::ios::end);
    auto const n = static_cast<std::size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    s.resize(n);
    if(n != 0) { ifs.read(s.data(), static_cast<std::streamsize>(n)); }
    return s;
}

inline std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
{
    if(!s.starts_with(base)) { return std::nullopt; }
    if(s.size() < base.size() + 3) { return std::nullopt; }  // "a[0]"
    if(s[base.size()] != '[') { return std::nullopt; }
    if(s.back() != ']') { return std::nullopt; }
    auto inner = s.substr(base.size() + 1, s.size() - (base.size() + 2));
    if(inner.empty()) { return std::nullopt; }
    std::size_t v{};
    for(char ch : inner)
    {
        if(ch < '0' || ch > '9') { return std::nullopt; }
        v = v * 10 + static_cast<std::size_t>(ch - '0');
    }
    return v;
}

enum class op_kind : std::uint8_t { add, sub, mul, div };

inline int run_pe_sim_and_export(op_kind op, std::filesystem::path const& verilog_rel, ::fast_io::u8string_view top_name, char const* sav_name)
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;
    using namespace phy_engine::phy_lab_wrapper;

    constexpr std::size_t kW = 8;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / verilog_rel;
    std::fprintf(stderr, "[fp8] load %s\n", src_path.string().c_str());
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    std::fprintf(stderr, "[fp8] compile\n");
    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, top_name);
    if(top_mod == nullptr) { return 2; }
    std::fprintf(stderr, "[fp8] elaborate\n");
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    std::array<std::size_t, kW> sig_a{};
    std::array<std::size_t, kW> sig_b{};
    std::array<std::size_t, kW> sig_y{};
    sig_a.fill(static_cast<std::size_t>(-1));
    sig_b.fill(static_cast<std::size_t>(-1));
    sig_y.fill(static_cast<std::size_t>(-1));

    for(auto const& p : top_inst.mod->ports)
    {
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(p.dir == port_dir::input)
        {
            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kW) { sig_a[*idx] = p.signal; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kW) { sig_b[*idx] = p.signal; }
        }
        else if(p.dir == port_dir::output)
        {
            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kW) { sig_y[*idx] = p.signal; }
        }
    }
    for(auto s : sig_a) { if(s == static_cast<std::size_t>(-1)) { return 4; } }
    for(auto s : sig_b) { if(s == static_cast<std::size_t>(-1)) { return 5; } }
    for(auto s : sig_y) { if(s == static_cast<std::size_t>(-1)) { return 6; } }

    auto set_sig_bit = [&](std::size_t sig, bool v) {
        top_inst.state.values.index_unchecked(sig) = v ? logic_t::true_state : logic_t::false_state;
    };
    auto read_sig_bit = [&](std::size_t sig) -> std::optional<bool> {
        auto const v = top_inst.state.values.index_unchecked(sig);
        if(v == logic_t::false_state) { return false; }
        if(v == logic_t::true_state) { return true; }
        return std::nullopt;
    };
    auto read_sig_u8 = [&]() -> std::optional<std::uint8_t> {
        std::uint8_t out{};
        for(std::size_t i{}; i < kW; ++i)
        {
            auto b = read_sig_bit(sig_y[i]);
            if(!b) { return std::nullopt; }
            if(*b) { out |= static_cast<std::uint8_t>(1u << i); }
        }
        return out;
    };

    auto is_nan = [](std::uint8_t x) noexcept -> bool { return ((x >> 3) & 0xFu) == 0xFu; };
    auto is_zero = [](std::uint8_t x) noexcept -> bool { return ((x >> 3) & 0xFu) == 0u && (x & 0x7u) == 0u; };

    struct sample
    {
        std::uint8_t a{};
        std::uint8_t b{};
        std::uint8_t y{};
    };

    // 1) Verilog simulator self-test (compile+elaborate+simulate()) + collect outputs as golden.
    std::fprintf(stderr, "[fp8] verilog-sim selftest\n");
    std::vector<sample> cases{};
    cases.reserve(512);
    std::uint64_t tick{};
    std::uint32_t lcg{0x12345678u};
    for(int t = 0; t < 512; ++t)
    {
        lcg = lcg * 1664525u + 1013904223u;
        std::uint8_t a = static_cast<std::uint8_t>(lcg >> 24);
        lcg = lcg * 1664525u + 1013904223u;
        std::uint8_t b = static_cast<std::uint8_t>(lcg >> 24);

        for(std::size_t i{}; i < kW; ++i) { set_sig_bit(sig_a[i], ((a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kW; ++i) { set_sig_bit(sig_b[i], ((b >> i) & 1u) != 0); }
        ::phy_engine::verilog::digital::simulate(top_inst, tick++);

        auto yv = read_sig_u8();
        if(!yv) { return 7; }
        cases.push_back(sample{a, b, *yv});

        // Minimal self-consistency checks (not a full NV compliance suite).
        if(is_nan(a) || is_nan(b))
        {
            if(*yv != 0x7F) { return 8; }
        }
        if(op == op_kind::mul)
        {
            if((is_zero(a) || is_zero(b)) && !(is_nan(a) || is_nan(b)) && *yv != 0x00) { return 8; }
        }
        if(op == op_kind::div)
        {
            if(is_zero(a) && *yv != 0x00) { return 8; }
            if(is_zero(b) && !is_nan(a) && *yv != 0x7F) { return 8; }
        }
    }

    // 2) Synthesize to PE netlist, then PE simulate + crosscheck with reference.
    std::fprintf(stderr, "[fp8] build pe netlist + synthesize\n");
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::array<::phy_engine::model::model_base*, kW> in_a{};
    std::array<::phy_engine::model::model_base*, kW> in_b{};
    std::array<std::size_t, kW> out_y{};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    out_y.fill(static_cast<std::size_t>(-1));

    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == port_dir::input)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(
                nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 9; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 10; }

            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kW) { in_a[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kW) { in_b[*idx] = m; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 11; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 12; }

            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kW) { out_y[*idx] = pi; }
        }
        else
        {
            return 13;
        }
    }

    for(auto* m : in_a) { if(m == nullptr) { return 14; } }
    for(auto* m : in_b) { if(m == nullptr) { return 15; } }
    for(auto pi : out_y) { if(pi == static_cast<std::size_t>(-1)) { return 16; } }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 17; }

    auto count_models_by_name = [&]() {
        struct key_hash
        {
            std::size_t operator()(::fast_io::u8string const& s) const noexcept
            {
                return std::hash<std::string_view>{}(std::string_view(reinterpret_cast<char const*>(s.data()), s.size()));
            }
        };
        struct key_eq
        {
            bool operator()(::fast_io::u8string const& a, ::fast_io::u8string const& b) const noexcept { return a == b; }
        };

        std::unordered_map<::fast_io::u8string, std::size_t, key_hash, key_eq> counts{};
        counts.reserve(64);
        for(auto& blk : nl.models)
        {
            for(auto* m = blk.begin; m != blk.curr; ++m)
            {
                if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                auto const name = m->ptr->get_model_name();
                counts[::fast_io::u8string{name}]++;
            }
        }
        return counts;
    };

    auto const counts = count_models_by_name();
    auto const get = [&](char const* nm) -> std::size_t {
        auto const len = std::strlen(nm);
        auto const key = ::fast_io::u8string{::fast_io::u8string_view{reinterpret_cast<char8_t const*>(nm), len}};
        auto it = counts.find(key);
        return (it == counts.end()) ? 0 : it->second;
    };

    std::fprintf(stderr,
                 "[fp8] pe models=%zu (FULL_ADDER=%zu HALF_ADDER=%zu XOR=%zu AND=%zu OR=%zu NOT=%zu YES=%zu)\n",
                 ::phy_engine::netlist::get_num_of_model(nl),
                 get("FULL_ADDER"),
                 get("HALF_ADDER"),
                 get("XOR"),
                 get("AND"),
                 get("OR"),
                 get("NOT"),
                 get("YES"));
    if(!c.analyze()) { return 18; }

    std::fprintf(stderr, "[fp8] pe-sim crosscheck\n");
    auto set_in = [&](::phy_engine::model::model_base* m, bool v) noexcept {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto set_inputs = [&](std::uint8_t a, std::uint8_t b) noexcept {
        for(std::size_t i{}; i < kW; ++i) { set_in(in_a[i], ((a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kW; ++i) { set_in(in_b[i], ((b >> i) & 1u) != 0); }
    };
    auto read_bit = [&](std::size_t pi) noexcept -> std::optional<bool> {
        auto const s = ports[pi]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
        if(s == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
        return std::nullopt;
    };
    auto read_u8 = [&]() noexcept -> std::optional<std::uint8_t> {
        std::uint8_t v{};
        for(std::size_t i{}; i < kW; ++i)
        {
            auto b = read_bit(out_y[i]);
            if(!b) { return std::nullopt; }
            if(*b) { v |= static_cast<std::uint8_t>(1u << i); }
        }
        return v;
    };

    for(auto const& tc : cases)
    {
        set_inputs(tc.a, tc.b);
        c.digital_clk();
        auto y = read_u8();
        if(!y) { return 19; }
        if(*y != tc.y) { return 20; }
    }

    // 3) Export PE->PL (.sav): row1=a, row2=b, row3=y.
    {
        std::fprintf(stderr, "[fp8] pe_to_pl convert (wires=%d)\n", 1);
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        auto const row_y_a = 1.0;
        auto const row_y_b = 0.5;
        auto const row_y_out = 0.0;

        // Layout requirement: low bit on the right (LSB at +x, MSB at -x).
        auto const x_for_bit_lsb_right = [](std::size_t idx, std::size_t n, double xmin, double xmax) noexcept -> double {
            if(n <= 1) { return (xmin + xmax) * 0.5; }
            auto const ridx = (n - 1) - idx;
            double const t = static_cast<double>(ridx) / static_cast<double>(n - 1);
            return xmin + (xmax - xmin) * t;
        };

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx)
            -> std::optional<phy_engine::phy_lab_wrapper::position> {
            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "a"); idx && *idx < kW)
                {
                    return position{x_for_bit_lsb_right(*idx, kW, -1.0, 1.0), row_y_a, 0.0};
                }
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "b"); idx && *idx < kW)
                {
                    return position{x_for_bit_lsb_right(*idx, kW, -1.0, 1.0), row_y_b, 0.0};
                }
            }
            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "y"); idx && *idx < kW)
                {
                    return position{x_for_bit_lsb_right(*idx, kW, -1.0, 1.0), row_y_out, 0.0};
                }
            }
            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
        std::fprintf(stderr, "[fp8] save %s\n", sav_name);
        auto const out_path = std::filesystem::path(__FILE__).parent_path() / sav_name;
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 21; }
        if(std::filesystem::file_size(out_path) < 128) { return 22; }
    }

    std::fprintf(stderr, "[fp8] done\n");
    return 0;
}
}  // namespace fp8_test
