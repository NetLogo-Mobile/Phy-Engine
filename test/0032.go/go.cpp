#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
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
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    static constexpr std::size_t kW = 19;
    static constexpr std::size_t kH = 19;

    auto const base_dir = std::filesystem::path(__FILE__).parent_path();
    std::filesystem::path src_path = base_dir / "go.v";
    std::string src_s{};
    try
    {
        src_s = read_file_text(src_path);
    }
    catch(...)
    {
        src_path = base_dir / "go.v";
        src_s = read_file_text(src_path);
    }
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl = c.get_netlist();

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        return 1;
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"go19x19_english");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::unordered_map<std::string, ::phy_engine::model::model_base*> input_by_name{};
    input_by_name.reserve(top_inst.mod->ports.size());

    std::vector<std::string> port_names{};
    port_names.resize(top_inst.mod->ports.size());

    std::array<::phy_engine::model::model_base*, 19> in_x{};
    std::array<::phy_engine::model::model_base*, 19> in_y{};
    for(auto& p : in_x) { p = nullptr; }
    for(auto& p : in_y) { p = nullptr; }

    ::phy_engine::model::model_base* in_clk{};
    ::phy_engine::model::model_base* in_rst_n{};
    ::phy_engine::model::model_base* in_place{};

    std::optional<std::size_t> out_black{};
    std::optional<std::size_t> out_white{};

    std::array<std::array<std::optional<std::size_t>, kW>, kH> row_port{};
    for(auto& r : row_port) { for(auto& x : r) { x = std::nullopt; } }

    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        port_names[pi] = port_name;

        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }
            input_by_name.emplace(port_name, m);

            if(port_name == "clk") { in_clk = m; }
            else if(port_name == "rst_n") { in_rst_n = m; }
            else if(port_name == "place") { in_place = m; }
            else if(auto bit = parse_bit_index(port_name, "x"); bit && *bit < in_x.size()) { in_x[*bit] = m; }
            else if(auto bit = parse_bit_index(port_name, "y"); bit && *bit < in_y.size()) { in_y[*bit] = m; }
            continue;
        }

        if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }

            if(port_name == "black") { out_black = pi; }
            else if(port_name == "white") { out_white = pi; }

            for(std::size_t yy{}; yy < kH; ++yy)
            {
                std::string base = "row" + std::to_string(yy);
                if(auto bit = parse_bit_index(port_name, base); bit && *bit < kW)
                {
                    row_port[yy][*bit] = pi;
                }
            }
            continue;
        }

        return 8;
    }

    if(in_clk == nullptr || in_rst_n == nullptr || in_place == nullptr) { return 9; }
    for(auto* p : in_x)
    {
        if(p == nullptr) { return 10; }
    }
    for(auto* p : in_y)
    {
        if(p == nullptr) { return 11; }
    }

    for(std::size_t yy{}; yy < kH; ++yy)
    {
        for(std::size_t xx{}; xx < kW; ++xx)
        {
            if(!row_port[yy][xx]) { return 12; }
        }
    }

    if(!out_black || !out_white) { return 12; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = 4,
        .optimize_wires = true,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    if(!c.analyze()) { return 13; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto set_bus = [&](auto const& bus, std::uint32_t v) {
        for(std::size_t i{}; i < bus.size(); ++i)
        {
            set_in(bus[i], ((v >> i) & 1u) != 0u);
        }
    };

    bool rst_n_level{true};

    auto tick = [&] {
        set_in(in_clk, false);
        c.digital_clk();
        set_in(in_clk, true);
        c.digital_clk();
        set_in(in_clk, false);
        c.digital_clk();
    };

    auto read_port_bit = [&](std::size_t pi) -> std::optional<bool> {
        auto const st = ports[pi]->node_information.dn.state;
        if(st == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
        if(st == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
        ::fast_io::io::perr(::fast_io::err(), "non-binary port state: port=", port_names[pi], " state=", st, "\n");
        return std::nullopt;
    };

    auto read_cell = [&](std::size_t xx, std::size_t yy) -> std::optional<bool> {
        auto const pi = *row_port[yy][xx];
        return read_port_bit(pi);
    };

    auto read_out = [&](std::size_t pi) -> std::optional<bool> { return read_port_bit(pi); };

    auto settle = [&](std::size_t n_ticks) {
        set_in(in_place, false);
        set_bus(in_x, 0);
        set_bus(in_y, 0);
        for(std::size_t i{}; i < n_ticks; ++i) { tick(); }
    };

    auto reset = [&] {
        set_in(in_place, false);
        set_bus(in_x, 0);
        set_bus(in_y, 0);

        rst_n_level = false;
        set_in(in_rst_n, false);
        tick();
        tick();
        tick();
        rst_n_level = true;
        set_in(in_rst_n, true);
        tick();
        settle(256);
    };

    auto play = [&](std::uint8_t xx, std::uint8_t yy) {
        if(xx >= 19 || yy >= 19) { std::abort(); }
        set_bus(in_x, (1u << xx));
        set_bus(in_y, (1u << yy));
        set_in(in_place, true);
        tick();
        set_in(in_place, false);
        set_bus(in_x, 0);
        set_bus(in_y, 0);
        settle(256);
    };

    // 1) Reset: board must be empty.
    reset();
    for(std::size_t yy{}; yy < kH; ++yy)
    {
        for(std::size_t xx{}; xx < kW; ++xx)
        {
            auto const b = read_cell(xx, yy);
            if(!b) { return 14; }
            if(*b) { return 14; }
        }
    }

    auto indicator_is_solid = [&] {
        auto const b0 = read_out(*out_black);
        auto const w0 = read_out(*out_white);
        if(!b0 || !w0) { return false; }
        tick();
        auto const b1 = read_out(*out_black);
        auto const w1 = read_out(*out_white);
        if(!b1 || !w1) { return false; }
        return (*b0 == *b1) && (*w0 == *w1);
    };

    // 2) Turn indicator should blink for side-to-move (initially black).
    {
        auto const b0 = read_out(*out_black);
        auto const w0 = read_out(*out_white);
        if(!b0 || !w0) { return 15; }
        tick();
        auto const b1 = read_out(*out_black);
        auto const w1 = read_out(*out_white);
        if(!b1 || !w1) { return 15; }
        // black toggles, white stays low.
        if(*w0 || *w1) { return 15; }
        if(*b0 == *b1) { return 15; }
    }

    // 3) Capture a single stone => winner indicator becomes solid (white wins in this sequence).
    // Place at: (1,1),(1,2) then surround (1,1) with (0,1),(2,1),(1,0), with dummy moves to alternate turns.
    play(1, 1);       // B
    play(1, 2);       // W
    play(10, 10);     // B dummy
    play(0, 1);       // W
    play(10, 11);     // B dummy
    play(2, 1);       // W
    play(10, 12);     // B dummy
    play(1, 0);       // W -> capture (1,1)
    {
        auto const b11 = read_cell(1, 1);
        if(!b11 || *b11) { return 16; }  // captured

        if(!indicator_is_solid()) { return 17; }
        auto const b = read_out(*out_black);
        auto const w = read_out(*out_white);
        if(!b || !w) { return 17; }
        if(*b) { return 17; }
        if(!*w) { return 17; }  // white winner
    }

    // 4) After reset, indicator returns to blinking (no winner latched).
    reset();
    {
        auto const b0 = read_out(*out_black);
        auto const w0 = read_out(*out_white);
        if(!b0 || !w0) { return 18; }
        tick();
        auto const b1 = read_out(*out_black);
        auto const w1 = read_out(*out_white);
        if(!b1 || !w1) { return 18; }
        if(*w0 || *w1) { return 18; }
        if(*b0 == *b1) { return 18; }
    }

    // 5) Suicide (no capture): corner (0,0) with neighbors occupied.
    reset();
    play(10, 10);  // B dummy
    play(0, 1);    // W
    play(10, 11);  // B dummy
    play(1, 0);    // W
    play(0, 0);    // B attempt illegal suicide; must be rejected
    {
        auto const b00 = read_cell(0, 0);
        if(!b00 || *b00) { return 19; }
        auto const b01 = read_cell(0, 1);
        if(!b01 || !*b01) { return 20; }
        auto const b10 = read_cell(1, 0);
        if(!b10 || !*b10) { return 21; }

        // Turn must not advance on illegal suicide: next move should still be black.
        play(0, 2);
        auto const b02 = read_cell(0, 2);
        if(!b02 || !*b02) { return 22; }
        // Still no winner latched.
        if(indicator_is_solid()) { return 23; }
    }

    // 5) Gate count check (after passing the sim checks).
    auto const gates = ::phy_engine::verilog::digital::details::count_logic_gates(nl);
    // Note: This design uses onehot (19-bit) x/y inputs to keep the logic size manageable.
    // Gate count is still higher than small 6x6 game demos; keep a looser cap here.
    if(gates > 10000u)
    {
        std::size_t n_and{};
        std::size_t n_or{};
        std::size_t n_xor{};
        std::size_t n_xnor{};
        std::size_t n_not{};
        std::size_t n_nand{};
        std::size_t n_nor{};
        std::size_t n_imp{};
        std::size_t n_nimp{};
        std::size_t n_yes{};
        for(auto const& blk : nl.models)
        {
            for(auto const* m = blk.begin; m != blk.curr; ++m)
            {
                if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
                auto const n = m->ptr->get_model_name();
                if(n == u8"AND") { ++n_and; }
                else if(n == u8"OR") { ++n_or; }
                else if(n == u8"XOR") { ++n_xor; }
                else if(n == u8"XNOR") { ++n_xnor; }
                else if(n == u8"NOT") { ++n_not; }
                else if(n == u8"NAND") { ++n_nand; }
                else if(n == u8"NOR") { ++n_nor; }
                else if(n == u8"IMP") { ++n_imp; }
                else if(n == u8"NIMP") { ++n_nimp; }
                else if(n == u8"YES") { ++n_yes; }
            }
        }
        ::fast_io::io::perr(::fast_io::err(),
                            "gate_count=",
                            gates,
                            " AND=",
                            n_and,
                            " OR=",
                            n_or,
                            " XOR=",
                            n_xor,
                            " XNOR=",
                            n_xnor,
                            " NOT=",
                            n_not,
                            " NAND=",
                            n_nand,
                            " NOR=",
                            n_nor,
                            " IMP=",
                            n_imp,
                            " NIMP=",
                            n_nimp,
                            " YES=",
                            n_yes,
                            "\n");
        return 24;
    }

    return 0;
}
