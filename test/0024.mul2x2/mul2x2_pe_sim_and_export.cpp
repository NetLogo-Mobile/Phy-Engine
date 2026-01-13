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
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
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

std::unordered_map<std::string, std::size_t> count_models_by_name(::phy_engine::netlist::netlist const& nl)
{
    std::unordered_map<std::string, std::size_t> counts{};
    counts.reserve(64);
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            auto const name = m->ptr->get_model_name();
            counts[std::string(reinterpret_cast<char const*>(name.data()), name.size())]++;
        }
    }
    return counts;
}
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;
    using namespace phy_engine::phy_lab_wrapper;

    static constexpr std::size_t kA = 2;
    static constexpr std::size_t kB = 2;
    static constexpr std::size_t kY = 4;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "mul2x2.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    std::fprintf(stderr, "[mul2x2] compile\n");
    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"mul2x2_top");
    if(top_mod == nullptr) { return 2; }

    // Elaborate for Verilog sim + PE synthesis.
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    // ---- Verilog self-sim (verilog::digital::simulate) ----
    std::array<std::size_t, kA> sig_a{};
    std::array<std::size_t, kB> sig_b{};
    std::array<std::size_t, kY> sig_y{};
    sig_a.fill(static_cast<std::size_t>(-1));
    sig_b.fill(static_cast<std::size_t>(-1));
    sig_y.fill(static_cast<std::size_t>(-1));

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
            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kY) { sig_y[*idx] = p.signal; }
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
    auto read_sig_u4 = [&]() -> std::optional<std::uint8_t> {
        std::uint8_t out{};
        for(std::size_t i{}; i < kY; ++i)
        {
            auto b = read_sig_bit(sig_y[i]);
            if(!b) { return std::nullopt; }
            if(*b) { out |= static_cast<std::uint8_t>(1u << i); }
        }
        return out;
    };

    std::fprintf(stderr, "[mul2x2] verilog-sim selftest\n");
    std::uint64_t tick{};
    for(std::uint8_t a = 0; a < 4; ++a)
    {
        for(std::uint8_t b = 0; b < 4; ++b)
        {
            for(std::size_t i{}; i < kA; ++i) { set_sig_bit(sig_a[i], ((a >> i) & 1u) != 0); }
            for(std::size_t i{}; i < kB; ++i) { set_sig_bit(sig_b[i], ((b >> i) & 1u) != 0); }
            ::phy_engine::verilog::digital::simulate(top_inst, tick++);

            auto yv = read_sig_u4();
            if(!yv) { return 7; }
            auto const ref = static_cast<std::uint8_t>(a * b);
            if(*yv != ref) { return 8; }
        }
    }

    // ---- PE synthesis + simulation crosscheck ----
    std::fprintf(stderr, "[mul2x2] pe synth\n");

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
    std::array<std::size_t, kY> out_y{};
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

            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kY) { out_y[*idx] = pi; }
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
        .optimize_adders = true,  // request: enable FULL/HALF adder recognition
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 17; }

    auto counts = count_models_by_name(nl);
    std::fprintf(stderr,
                 "[mul2x2] pe models=%zu (FULL_ADDER=%zu HALF_ADDER=%zu)\n",
                 ::phy_engine::netlist::get_num_of_model(nl),
                 counts["FULL_ADDER"],
                 counts["HALF_ADDER"]);

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
    auto read_u4 = [&]() noexcept -> std::optional<std::uint8_t> {
        std::uint8_t v{};
        for(std::size_t i{}; i < kY; ++i)
        {
            auto b = read_bit(out_y[i]);
            if(!b) { return std::nullopt; }
            if(*b) { v |= static_cast<std::uint8_t>(1u << i); }
        }
        return v;
    };

    std::fprintf(stderr, "[mul2x2] pe-sim crosscheck\n");
    for(std::uint8_t a = 0; a < 4; ++a)
    {
        for(std::uint8_t b = 0; b < 4; ++b)
        {
            set_inputs(a, b);
            c.digital_clk();
            auto yv = read_u4();
            if(!yv) { return 19; }
            auto const ref = static_cast<std::uint8_t>(a * b);
            if(*yv != ref) { return 20; }
        }
    }

    // ---- Export PE->PL (.sav) ----
    // Row1: a (LSB on right), Row2: b (LSB on right), Row3: y[3:0] (LSB on right).
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        auto const row_y_a = 1.0;
        auto const row_y_b = 0.5;
        auto const row_y_out = 0.0;

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
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "a"); idx && *idx < kA)
                {
                    return position{x_for_bit_lsb_right(*idx, kA, -0.5, 0.5), row_y_a, 0.0};
                }
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "b"); idx && *idx < kB)
                {
                    return position{x_for_bit_lsb_right(*idx, kB, -0.5, 0.5), row_y_b, 0.0};
                }
            }
            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "y"); idx && *idx < kY)
                {
                    return position{x_for_bit_lsb_right(*idx, kY, -1.0, 1.0), row_y_out, 0.0};
                }
            }
            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
        auto const out_path = std::filesystem::path(__FILE__).parent_path() / "mul2x2_pe_to_pl.sav";
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 21; }
        if(std::filesystem::file_size(out_path) < 128) { return 22; }
    }

    return 0;
}

