#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <vector>

namespace
{
::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}

std::string read_file_text(std::filesystem::path const& path)
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

std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
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
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    static constexpr std::size_t kA = 8;
    static constexpr std::size_t kB = 8;
    static constexpr std::size_t kP = 16;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "mul8x8_serial.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    std::fprintf(stderr, "[mul8x8_serial] compile\n");
    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"mul8");
    if(top_mod == nullptr) { return 2; }

    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    // ---- Verilog self-sim (verilog::digital::simulate) ----
    std::array<std::size_t, kA> sig_a{};
    std::array<std::size_t, kB> sig_b{};
    std::array<std::size_t, kP> sig_p{};
    sig_a.fill(static_cast<std::size_t>(-1));
    sig_b.fill(static_cast<std::size_t>(-1));
    sig_p.fill(static_cast<std::size_t>(-1));

    for(auto const& p : top_inst.mod->ports)
    {
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(p.dir == port_dir::input)
        {
            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kA) { sig_a[*idx] = p.signal; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kB) { sig_b[*idx] = p.signal; }
        }
        else if(p.dir == port_dir::output)
        {
            if(auto idx = parse_bit_index(port_name, "p"); idx && *idx < kP) { sig_p[*idx] = p.signal; }
        }
    }
    for(auto s : sig_a) { if(s == static_cast<std::size_t>(-1)) { return 4; } }
    for(auto s : sig_b) { if(s == static_cast<std::size_t>(-1)) { return 5; } }
    for(auto s : sig_p) { if(s == static_cast<std::size_t>(-1)) { return 6; } }

    auto set_sig_bit = [&](std::size_t sig, bool v) {
        top_inst.state.values.index_unchecked(sig) = v ? logic_t::true_state : logic_t::false_state;
    };
    auto read_sig_bit = [&](std::size_t sig) -> std::optional<bool> {
        auto const v = top_inst.state.values.index_unchecked(sig);
        if(v == logic_t::false_state) { return false; }
        if(v == logic_t::true_state) { return true; }
        return std::nullopt;
    };
    auto read_sig_u16 = [&]() -> std::optional<std::uint16_t> {
        std::uint16_t out{};
        for(std::size_t i{}; i < kP; ++i)
        {
            auto b = read_sig_bit(sig_p[i]);
            if(!b) { return std::nullopt; }
            if(*b) { out |= static_cast<std::uint16_t>(1u << i); }
        }
        return out;
    };

    auto verilog_check_one = [&](std::uint8_t a, std::uint8_t b, std::uint64_t& tick) -> bool {
        for(std::size_t i{}; i < kA; ++i) { set_sig_bit(sig_a[i], ((a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kB; ++i) { set_sig_bit(sig_b[i], ((b >> i) & 1u) != 0); }
        ::phy_engine::verilog::digital::simulate(top_inst, tick++);

        auto pv = read_sig_u16();
        if(!pv)
        {
            std::fprintf(stderr, "[mul8x8_serial] verilog-sim produced X/Z at a=%u b=%u\n", static_cast<unsigned>(a), static_cast<unsigned>(b));
            return false;
        }
        auto const ref = static_cast<std::uint16_t>(static_cast<std::uint16_t>(a) * static_cast<std::uint16_t>(b));
        if(*pv != ref)
        {
            std::fprintf(
                stderr,
                "[mul8x8_serial] verilog-sim mismatch a=%u b=%u got=%u ref=%u\n",
                static_cast<unsigned>(a),
                static_cast<unsigned>(b),
                static_cast<unsigned>(*pv),
                static_cast<unsigned>(ref));
            return false;
        }
        return true;
    };

    std::fprintf(stderr, "[mul8x8_serial] verilog-sim selftest (key+random)\n");
    std::uint64_t tick{};

    // ---- PE synthesis + simulation crosscheck ----
    std::fprintf(stderr, "[mul8x8_serial] pe synth\n");
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

    std::array<::phy_engine::model::model_base*, kA> in_a{};
    std::array<::phy_engine::model::model_base*, kB> in_b{};
    std::array<std::size_t, kP> out_p{};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    out_p.fill(static_cast<std::size_t>(-1));

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

            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kA) { in_a[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kB) { in_b[*idx] = m; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 11; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 12; }

            if(auto idx = parse_bit_index(port_name, "p"); idx && *idx < kP) { out_p[*idx] = pi; }
        }
        else
        {
            return 13;
        }
    }

    for(auto* m : in_a) { if(m == nullptr) { return 14; } }
    for(auto* m : in_b) { if(m == nullptr) { return 15; } }
    for(auto pi : out_p) { if(pi == static_cast<std::size_t>(-1)) { return 16; } }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 17; }
    if(!c.analyze()) { return 18; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) noexcept {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto set_inputs = [&](std::uint8_t a, std::uint8_t b) noexcept {
        for(std::size_t i{}; i < kA; ++i) { set_in(in_a[i], ((a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kB; ++i) { set_in(in_b[i], ((b >> i) & 1u) != 0); }
    };
    auto read_bit = [&](std::size_t pi) noexcept -> std::optional<bool> {
        auto const s = ports[pi]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
        if(s == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
        return std::nullopt;
    };
    auto read_u16 = [&]() noexcept -> std::optional<std::uint16_t> {
        std::uint16_t v{};
        for(std::size_t i{}; i < kP; ++i)
        {
            auto b = read_bit(out_p[i]);
            if(!b) { return std::nullopt; }
            if(*b) { v |= static_cast<std::uint16_t>(1u << i); }
        }
        return v;
    };

    std::fprintf(stderr, "[mul8x8_serial] pe-sim crosscheck (exhaustive)\n");
    auto check_one = [&](std::uint8_t a, std::uint8_t b) -> bool {
        set_inputs(a, b);
        c.digital_clk();
        auto pv = read_u16();
        if(!pv)
        {
            std::fprintf(stderr, "[mul8x8_serial] pe-sim produced X/Z at a=%u b=%u\n", static_cast<unsigned>(a), static_cast<unsigned>(b));
            return false;
        }
        auto const ref = static_cast<std::uint16_t>(static_cast<std::uint16_t>(a) * static_cast<std::uint16_t>(b));
        if(*pv != ref)
        {
            std::fprintf(
                stderr,
                "[mul8x8_serial] pe-sim mismatch a=%u b=%u got=%u ref=%u\n",
                static_cast<unsigned>(a),
                static_cast<unsigned>(b),
                static_cast<unsigned>(*pv),
                static_cast<unsigned>(ref));
            return false;
        }
        return true;
    };

    // Key vectors (tend to trigger width/shift/carry bugs).
    static constexpr std::array<std::uint8_t, 10> kVals{
        std::uint8_t{0x00},
        std::uint8_t{0x01},
        std::uint8_t{0x02},
        std::uint8_t{0x7F},
        std::uint8_t{0x80},
        std::uint8_t{0x81},
        std::uint8_t{0xFE},
        std::uint8_t{0xFF},
        std::uint8_t{0x55},
    };
    for(auto a : kVals)
    {
        for(auto b : kVals)
        {
            if(!verilog_check_one(a, b, tick)) { return 8; }
            if(!check_one(a, b)) { return 20; }
        }
    }

    // Deterministic random sampling for fast failure.
    std::mt19937 rng{0xC0FFEEu};
    std::uniform_int_distribution<int> dist(0, 255);
    constexpr int kRandomIters = 256;
    for(int i = 0; i < kRandomIters; ++i)
    {
        auto const a = static_cast<std::uint8_t>(dist(rng));
        auto const b = static_cast<std::uint8_t>(dist(rng));
        if(!verilog_check_one(a, b, tick)) { return 9; }
        if(!check_one(a, b)) { return 21; }
    }

    return 0;
}
