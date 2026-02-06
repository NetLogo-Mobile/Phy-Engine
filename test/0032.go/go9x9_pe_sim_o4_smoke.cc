#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
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
    if(s.size() < base.size() + 3) { return std::nullopt; } // "a[0]"
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
} // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::verilog::digital;

    static constexpr std::size_t kW = 9;
    static constexpr std::size_t kH = 9;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "go9x9.v";
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
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"go9x9");
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

    std::optional<std::size_t> out_black{};
    std::optional<std::size_t> out_white{};
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

    if(!out_black || !out_white) { return 9; }
    for(std::size_t yy{}; yy < kH; ++yy)
    {
        for(std::size_t xx{}; xx < kW; ++xx)
        {
            if(!row_port[yy][xx]) { return 10; }
        }
    }

    auto need_in = [&](char const* n) -> ::phy_engine::model::model_base* {
        auto it = input_by_name.find(n);
        if(it == input_by_name.end()) { return nullptr; }
        return it->second;
    };

    auto* in_clk = need_in("clk");
    auto* in_rst_n = need_in("rst_n");
    auto* in_up = need_in("up");
    auto* in_down = need_in("down");
    auto* in_left = need_in("left");
    auto* in_right = need_in("right");
    auto* in_place = need_in("place");
    auto* in_pass = need_in("pass");
    if(in_clk == nullptr || in_rst_n == nullptr || in_up == nullptr || in_down == nullptr || in_left == nullptr || in_right == nullptr ||
       in_place == nullptr || in_pass == nullptr)
    {
        return 11;
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = 0,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    if(!c.analyze()) { return 12; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

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
        return std::nullopt;
    };

    auto sync_to_plane = [&](bool want_white) -> bool {
        for(std::size_t guard{}; guard < 8; ++guard)
        {
            auto const w = read_port_bit(*out_white);
            if(!w) { return false; }
            if(*w == want_white) { return true; }
            tick();
        }
        return false;
    };

    auto read_cell = [&](bool want_white, std::size_t xx, std::size_t yy) -> std::optional<bool> {
        if(xx >= kW || yy >= kH) { return std::nullopt; }
        if(!sync_to_plane(want_white)) { return std::nullopt; }
        auto const pi = *row_port[yy][xx];
        return read_port_bit(pi);
    };

    auto settle = [&](std::size_t n_ticks) {
        set_in(in_up, false);
        set_in(in_down, false);
        set_in(in_left, false);
        set_in(in_right, false);
        set_in(in_place, false);
        set_in(in_pass, false);
        for(std::size_t i{}; i < n_ticks; ++i) { tick(); }
    };

    auto pulse = [&](::phy_engine::model::model_base* m) {
        set_in(m, true);
        tick();
        set_in(m, false);
        tick();
    };

    std::size_t cur_x = 4;
    std::size_t cur_y = 4;

    auto move_to = [&](std::size_t xx, std::size_t yy) {
        while(cur_x > xx)
        {
            pulse(in_left);
            --cur_x;
        }
        while(cur_x < xx)
        {
            pulse(in_right);
            ++cur_x;
        }
        while(cur_y > yy)
        {
            pulse(in_up);
            --cur_y;
        }
        while(cur_y < yy)
        {
            pulse(in_down);
            ++cur_y;
        }
    };

    auto play_at = [&](std::size_t xx, std::size_t yy) {
        move_to(xx, yy);
        pulse(in_place);
        settle(2048);
    };

    auto reset = [&] {
        set_in(in_rst_n, false);
        settle(4);
        set_in(in_rst_n, true);
        cur_x = 4;
        cur_y = 4;
        settle(2048);
    };

    auto park_cursor = [&](std::size_t avoid_x, std::size_t avoid_y) {
        if(!(cur_x == avoid_x && cur_y == avoid_y)) { return; }
        if(avoid_x != 8 || avoid_y != 8) { move_to(8, 8); }
        else { move_to(0, 0); }
    };

    // 1) Reset: board must be empty in both planes.
    reset();
    for(std::size_t yy{}; yy < kH; ++yy)
    {
        for(std::size_t xx{}; xx < kW; ++xx)
        {
            park_cursor(xx, yy);
            auto const b = read_cell(false, xx, yy);
            auto const w = read_cell(true, xx, yy);
            if(!b || !w) { return 13; }
            if(*b || *w) { return 13; }
        }
    }

    // 2) Occupied intersection: white can't play on black's stone; illegal move must not advance turn.
    reset();
    play_at(0, 0); // B
    play_at(0, 0); // W illegal
    play_at(0, 1); // W (must still be W)
    {
        park_cursor(0, 0);
        auto const b00 = read_cell(false, 0, 0);
        auto const w00 = read_cell(true, 0, 0);
        auto const w01 = read_cell(true, 0, 1);
        if(!b00 || !w00 || !w01) { return 14; }
        if(!*b00) { return 14; }
        if(*w00) { return 14; }
        if(!*w01) { return 14; }
    }

    // 3) Single-stone capture.
    // Surround W(1,1) by placing B at (0,1),(1,0),(2,1),(1,2).
    reset();
    play_at(0, 1); // B
    play_at(1, 1); // W
    play_at(1, 0); // B
    play_at(8, 8); // W dummy
    play_at(2, 1); // B
    play_at(8, 7); // W dummy
    play_at(1, 2); // B -> capture (1,1)
    {
        park_cursor(1, 1);
        auto const w11 = read_cell(true, 1, 1);
        auto const b01 = read_cell(false, 0, 1);
        auto const b10 = read_cell(false, 1, 0);
        auto const b21 = read_cell(false, 2, 1);
        auto const b12 = read_cell(false, 1, 2);
        if(!w11 || !b01 || !b10 || !b21 || !b12) { return 15; }
        if(*w11) { return 15; }
        if(!*b01 || !*b10 || !*b21 || !*b12) { return 15; }
    }

    // 4) Suicide is illegal (no capture).
    reset();
    play_at(8, 8); // B dummy
    play_at(0, 1); // W
    play_at(8, 7); // B dummy
    play_at(1, 0); // W
    play_at(0, 0); // B illegal suicide in corner
    play_at(0, 2); // B must still be to-move
    {
        park_cursor(0, 0);
        auto const b00 = read_cell(false, 0, 0);
        auto const b02 = read_cell(false, 0, 2);
        if(!b00 || !b02) { return 16; }
        if(*b00) { return 16; }
        if(!*b02) { return 16; }
    }

    // 5) Simple ko: immediate recapture forbidden; allowed after one intervening move.
    reset();
    // Build a ko in the middle to avoid collateral captures:
    // W at P=(4,4) has last liberty at Q=(4,3). B plays at Q to capture exactly 1 stone and create ko.
    play_at(3, 4); // B
    play_at(4, 4); // W (P)
    play_at(5, 4); // B
    play_at(3, 3); // W
    play_at(4, 5); // B
    play_at(5, 3); // W
    play_at(8, 8); // B dummy
    play_at(4, 2); // W
    play_at(4, 3); // B captures W(4,4), ko point at (4,4)

    play_at(4, 4); // W illegal immediate recapture
    play_at(8, 7); // W legal elsewhere (still W)
    play_at(8, 6); // B legal elsewhere
    play_at(4, 4); // W recapture now legal; captures B(4,3)
    {
        park_cursor(4, 3);
        auto const b43 = read_cell(false, 4, 3);
        auto const w44 = read_cell(true, 4, 4);
        if(!b43 || !w44) { return 17; }
        if(*b43) { return 17; }
        if(!*w44) { return 17; }
    }

    // O4 synthesis gate count <= 5000 for the gate-budget (lite) design.
    {
        auto const lite_src_path = std::filesystem::path(__FILE__).parent_path() / "go9x9_lite.v";
        auto const lite_src_s = read_file_text(lite_src_path);
        auto const lite_src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(lite_src_s.data()), lite_src_s.size()};

        ::phy_engine::circult c2{};
        c2.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl2 = c2.get_netlist();

        auto cr2 = ::phy_engine::verilog::digital::compile(lite_src);
        if(!cr2.errors.empty() || cr2.modules.empty()) { return 18; }
        auto design2 = ::phy_engine::verilog::digital::build_design(::std::move(cr2));
        auto const* mod2 = ::phy_engine::verilog::digital::find_module(design2, u8"go9x9_lite");
        if(mod2 == nullptr) { return 18; }
        auto top_inst2 = ::phy_engine::verilog::digital::elaborate(design2, *mod2);
        if(top_inst2.mod == nullptr) { return 18; }

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

        ::phy_engine::verilog::digital::pe_synth_error err2{};
        if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl2, top_inst2, ports2, &err2, opt_o4)) { return 18; }

        auto const gates = ::phy_engine::verilog::digital::details::count_logic_gates(nl2);
        if(gates > 5000u)
        {
            ::fast_io::io::perr(::fast_io::err(), "go9x9_lite gate_count=", gates, " (limit=5000)\n");
            return 19;
        }
    }

    return 0;
}
