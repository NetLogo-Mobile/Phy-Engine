#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <cassert>
#include <cstddef>
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

    static constexpr std::size_t kW = 6;
    static constexpr std::size_t kH = 6;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "tetris6x6.v";
    auto const src_s = read_file_text(src_path);
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
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"tetris6x6");
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

    std::optional<std::size_t> game_over_port{};
    std::array<std::array<std::optional<std::size_t>, kW>, kH> row_port{};
    for(auto& r : row_port) { for(auto& x : r) { x = std::nullopt; } }

    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }
            input_by_name.emplace(std::move(port_name), m);
            continue;
        }

        if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }

            if(port_name == "game_over") { game_over_port = pi; }

            for(std::size_t y{}; y < kH; ++y)
            {
                std::string base = "row" + std::to_string(y);
                if(auto bit = parse_bit_index(port_name, base); bit && *bit < kW)
                {
                    row_port[y][*bit] = pi;
                }
            }
            continue;
        }

        return 8;
    }

    if(!game_over_port) { return 9; }
    for(std::size_t y{}; y < kH; ++y)
    {
        for(std::size_t x{}; x < kW; ++x)
        {
            if(!row_port[y][x]) { return 10; }
        }
    }

    if(!input_by_name.contains("clk") || !input_by_name.contains("rst") || !input_by_name.contains("left") ||
       !input_by_name.contains("right") || !input_by_name.contains("rotate"))
    {
        return 11;
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt_sim{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = 0,
        .optimize_wires = false,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt_sim))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    // Ensure `$random` lowered to a PE RNG macro (otherwise it may constant-fold and always spawn I).
    {
        bool has_rng{};
        for(auto& blk : nl.models)
        {
            for(auto* m = blk.begin; m != blk.curr; ++m)
            {
                if(m->type != ::phy_engine::model::model_type::normal) { continue; }
                if(m->ptr == nullptr) { continue; }
                if(m->ptr->get_model_name() == ::fast_io::u8string_view{u8"RANDOM_GENERATOR4"}) { has_rng = true; }
            }
        }
        if(!has_rng) { return 12; }
    }

    if(!c.analyze()) { return 12; }

    auto* in_clk = input_by_name.at("clk");
    auto* in_rst = input_by_name.at("rst");
    auto* in_left = input_by_name.at("left");
    auto* in_right = input_by_name.at("right");
    auto* in_rotate = input_by_name.at("rotate");

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto tick = [&] {
        set_in(in_clk, true);
        c.digital_clk();
        set_in(in_clk, false);
        c.digital_clk();
        c.digital_clk(); // settle combinational after edge
    };

    auto read_port_bit = [&](std::size_t pi) -> bool {
        auto const st = ports[pi]->node_information.dn.state;
        assert(st == ::phy_engine::model::digital_node_statement_t::true_state ||
               st == ::phy_engine::model::digital_node_statement_t::false_state);
        return st == ::phy_engine::model::digital_node_statement_t::true_state;
    };

    auto read_row = [&](std::size_t y) -> std::uint8_t {
        std::uint8_t v{};
        for(std::size_t x{}; x < kW; ++x)
        {
            auto const pi = *row_port[y][x];
            if(read_port_bit(pi)) { v |= static_cast<std::uint8_t>(1u << x); }
        }
        return v;
    };

    auto reset = [&] {
        set_in(in_left, false);
        set_in(in_right, false);
        set_in(in_rotate, false);

        set_in(in_clk, false);
        c.digital_clk();

        set_in(in_rst, true);
        tick();
        tick();
        set_in(in_rst, false);
        set_in(in_clk, false);
        c.digital_clk();  // settle rst deassertion
        tick();           // spawn first piece
    };

    // 1) Reset + spawn: expect deterministic first piece (I horizontal), game_over=0.
    reset();
    if(read_port_bit(*game_over_port)) { return 13; }
    if(read_row(0) != 0b011110)
    {
        return 14;
    }
    for(std::size_t y = 1; y < kH; ++y)
    {
        if(read_row(y) != 0) { return 15; }
    }

    // 2) Move right once, then rotate to place a vertical obstacle column at x=2.
    set_in(in_right, true);
    set_in(in_clk, false);
    c.digital_clk();
    tick();
    set_in(in_right, false);
    set_in(in_clk, false);
    c.digital_clk();
    if(read_port_bit(*game_over_port)) { return 16; }
    if(read_row(0) != 0b111100) { return 17; }  // I moved right (px=2)

    set_in(in_rotate, true);
    set_in(in_clk, false);
    c.digital_clk();
    tick();
    set_in(in_rotate, false);
    set_in(in_clk, false);
    c.digital_clk();
    if(read_port_bit(*game_over_port)) { return 18; }
    if(read_row(0) != 0b000100) { return 19; }
    if(read_row(1) != 0b000100) { return 20; }
    if(read_row(2) != 0b000100) { return 21; }
    if(read_row(3) != 0b000100) { return 22; }

    // 3) Let the vertical I fall and lock at the bottom (obstacle at rows2..5, x=2).
    set_in(in_left, false);
    set_in(in_right, false);
    set_in(in_rotate, false);
    tick();  // py=1
    tick();  // py=2
    tick();  // lock -> CLEAR

    if(read_row(0) != 0) { return 23; }
    if(read_row(1) != 0) { return 24; }
    if(read_row(2) != 0b000100) { return 25; }
    if(read_row(3) != 0b000100) { return 26; }
    if(read_row(4) != 0b000100) { return 27; }
    if(read_row(5) != 0b000100) { return 28; }

    // 4) Spawn one more piece and ensure it does not overlap ("phase") into the obstacle column.
    // (All pieces overlap bit2 at spawn, so collide_down should stop them above row2.)
    {
        bool saw_spawn{};
        for(std::size_t i{}; i < 256; ++i)
        {
            tick();
            if(read_port_bit(*game_over_port)) { return 29; }
            if(read_row(0) != 0)
            {
                saw_spawn = true;
                break;
            }
        }
        if(!saw_spawn) { return 30; }

        tick();  // py=1
        tick();  // collide_down -> lock
        if(read_row(2) != 0b000100) { return 31; }
        if(read_row(3) != 0b000100) { return 32; }
        if(read_row(4) != 0b000100) { return 33; }
        if(read_row(5) != 0b000100) { return 34; }
    }

    // 5) Run until game_over and check the "X" overlay.
    set_in(in_left, false);
    set_in(in_rotate, false);
    set_in(in_right, true);  // bias towards a quick loss

    bool saw_game_over{};
    for(std::size_t i{}; i < 4096; ++i)
    {
        tick();
        if(read_port_bit(*game_over_port))
        {
            saw_game_over = true;
            break;
        }
    }
    if(!saw_game_over) { return 36; }

    if(read_row(0) != 0b100001) { return 37; }
    if(read_row(1) != 0b010010) { return 38; }
    if(read_row(2) != 0b001100) { return 39; }
    if(read_row(3) != 0b001100) { return 40; }
    if(read_row(4) != 0b010010) { return 41; }
    if(read_row(5) != 0b100001) { return 42; }

    // 4) Gate count check (after passing the sim checks), under O4 optimization.
    {
        ::phy_engine::circult c2{};
        c2.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl2 = c2.get_netlist();

        auto cr2 = ::phy_engine::verilog::digital::compile(src);
        if(!cr2.errors.empty() || cr2.modules.empty()) { return 28; }

        auto design2 = ::phy_engine::verilog::digital::build_design(::std::move(cr2));
        auto const* mod2 = ::phy_engine::verilog::digital::find_module(design2, u8"tetris6x6");
        if(mod2 == nullptr) { return 28; }
        auto top_inst2 = ::phy_engine::verilog::digital::elaborate(design2, *mod2);
        if(top_inst2.mod == nullptr) { return 28; }

        std::vector<::phy_engine::model::node_t*> ports2{};
        ports2.reserve(top_inst2.mod->ports.size());
        for(std::size_t i{}; i < top_inst2.mod->ports.size(); ++i)
        {
            auto& n = ::phy_engine::netlist::create_node(nl2);
            ports2.push_back(__builtin_addressof(n));
        }

        ::phy_engine::verilog::digital::pe_synth_options opt_o4{
            .allow_inout = false,
            .allow_multi_driver = false,
            .assume_binary_inputs = true,
            .opt_level = 4,
            .optimize_wires = true,
            .optimize_mul2 = true,
            .optimize_adders = true,
        };
        // Avoid rare crashes in some heavy O4 passes; still runs the O4 pipeline.
        opt_o4.qm_max_vars = 0;
        opt_o4.resub_max_vars = 0;
        opt_o4.sweep_max_vars = 0;

        ::phy_engine::verilog::digital::pe_synth_error err2{};
        if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl2, top_inst2, ports2, &err2, opt_o4)) { return 28; }

        auto const gates = ::phy_engine::verilog::digital::details::count_logic_gates(nl2);
        if(gates > 5000u) { return 28; }
    }

    return 0;
}
