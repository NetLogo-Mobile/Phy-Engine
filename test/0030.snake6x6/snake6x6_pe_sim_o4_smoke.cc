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

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "snake6x6.v";
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
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"snake6x6");
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

    std::optional<std::size_t> game_over_port{};
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

    if(!input_by_name.contains("clk") || !input_by_name.contains("rst_n") || !input_by_name.contains("left") ||
       !input_by_name.contains("right") || !input_by_name.contains("rotate"))
    {
        return 11;
    }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .opt_level = 4,
        .optimize_wires = false,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };
    // Work around rare O4 crashes in some optimization passes by disabling them for this test.
    opt.qm_max_vars = 0;
    opt.resub_max_vars = 0;
    opt.sweep_max_vars = 0;
    // Keep O4 but avoid complex transforms that can create hard-to-debug Xs in simulation.
    opt.techmap_enable = false;
    opt.decompose_large_functions = false;
    opt.infer_dc_from_xz = false;
    opt.infer_dc_from_fsm = false;
    opt.infer_dc_from_odc = false;
    opt.dc_fsm_max_bits = 0;
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    if(!c.analyze()) { return 12; }

    auto* in_clk = input_by_name.at("clk");
    auto* in_rst = input_by_name.at("rst_n");
    auto* in_left = input_by_name.at("left");
    auto* in_right = input_by_name.at("right");
    auto* in_rotate = input_by_name.at("rotate");

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

        ::fast_io::io::perr(::fast_io::err(),
                            "non-binary port state: port=",
                            port_names[pi],
                            " state=",
                            st,
                            " analog=",
                            ports[pi]->num_of_analog_node,
                            " pins=",
                            ports[pi]->pins.size(),
                            "\n");
        for(auto* p : ports[pi]->pins)
        {
            if(p == nullptr || p->model == nullptr || p->model->ptr == nullptr) { continue; }
            auto const mn = p->model->ptr->get_model_name();
            std::string const mns(reinterpret_cast<char const*>(mn.data()), mn.size());
            std::string const inst(reinterpret_cast<char const*>(p->model->name.data()), p->model->name.size());
            std::string const pin(reinterpret_cast<char const*>(p->name.data()), p->name.size());
            ::fast_io::io::perr(::fast_io::err(), "  pin model=", mns, " inst=", inst, " pin=", pin, "\n");

            auto pv = p->model->ptr->generate_pin_view();
            ::phy_engine::model::node_t* first_in_node{};
            for(std::size_t i{}; i < pv.size; ++i)
            {
                auto const pn = pv.pins[i].name;
                std::string const pns(reinterpret_cast<char const*>(pn.data()), pn.size());
                auto* n = pv.pins[i].nodes;
                if(n == nullptr)
                {
                    ::fast_io::io::perr(::fast_io::err(), "    conn ", i, " ", pns, " = <null>\n");
                    continue;
                }
                if(i == 0) { first_in_node = n; }
                ::fast_io::io::perr(::fast_io::err(), "    conn ", i, " ", pns, " = ", n->node_information.dn.state, "\n");
            }

            if(first_in_node != nullptr)
            {
                ::fast_io::io::perr(::fast_io::err(), "    upstream pins:\n");
                for(auto* up : first_in_node->pins)
                {
                    if(up == nullptr || up->model == nullptr || up->model->ptr == nullptr) { continue; }
                    auto const up_mn = up->model->ptr->get_model_name();
                    std::string const up_mns(reinterpret_cast<char const*>(up_mn.data()), up_mn.size());
                    std::string const up_inst(reinterpret_cast<char const*>(up->model->name.data()), up->model->name.size());
                    std::string const up_pin(reinterpret_cast<char const*>(up->name.data()), up->name.size());
                    ::fast_io::io::perr(::fast_io::err(), "      ", up_mns, " inst=", up_inst, " pin=", up_pin, "\n");

                    // One extra level: print this upstream model's own connections.
                    auto up_pv = up->model->ptr->generate_pin_view();
                    for(std::size_t ui{}; ui < up_pv.size; ++ui)
                    {
                        auto const up_pn = up_pv.pins[ui].name;
                        std::string const up_pns(reinterpret_cast<char const*>(up_pn.data()), up_pn.size());
                        auto* un = up_pv.pins[ui].nodes;
                        if(un == nullptr)
                        {
                            ::fast_io::io::perr(::fast_io::err(), "        conn ", ui, " ", up_pns, " = <null>\n");
                            continue;
                        }
                        auto const unst = un->node_information.dn.state;
                        ::fast_io::io::perr(::fast_io::err(), "        conn ", ui, " ", up_pns, " = ", unst, "\n");
                        if(unst == ::phy_engine::model::digital_node_statement_t::indeterminate_state)
                        {
                            ::fast_io::io::perr(::fast_io::err(), "          node pins:\n");
                            for(auto* np : un->pins)
                            {
                                if(np == nullptr || np->model == nullptr || np->model->ptr == nullptr) { continue; }
                                auto const np_mn = np->model->ptr->get_model_name();
                                std::string const np_mns(reinterpret_cast<char const*>(np_mn.data()), np_mn.size());
                                std::string const np_inst(reinterpret_cast<char const*>(np->model->name.data()), np->model->name.size());
                                std::string const np_pin(reinterpret_cast<char const*>(np->name.data()), np->name.size());
                                ::fast_io::io::perr(::fast_io::err(), "            ", np_mns, " inst=", np_inst, " pin=", np_pin, "\n");
                            }

                            // Dump connections for common logic gate drivers on this node.
                            for(auto* np : un->pins)
                            {
                                if(np == nullptr || np->model == nullptr || np->model->ptr == nullptr) { continue; }
                                auto const np_mn = np->model->ptr->get_model_name();
                                if(np_mn != u8"OR" && np_mn != u8"AND" && np_mn != u8"NOT" && np_mn != u8"NAND" && np_mn != u8"NOR" &&
                                   np_mn != u8"XOR" && np_mn != u8"XNOR" && np_mn != u8"IMP" && np_mn != u8"NIMP" && np_mn != u8"YES")
                                {
                                    continue;
                                }

                                std::string const np_mns(reinterpret_cast<char const*>(np_mn.data()), np_mn.size());
                                std::string const np_inst(reinterpret_cast<char const*>(np->model->name.data()), np->model->name.size());
                                ::fast_io::io::perr(::fast_io::err(), "          driver ", np_mns, " inst=", np_inst, "\n");
                                auto d_pv = np->model->ptr->generate_pin_view();
                                for(std::size_t di{}; di < d_pv.size; ++di)
                                {
                                    auto const dn = d_pv.pins[di].name;
                                    std::string const dns(reinterpret_cast<char const*>(dn.data()), dn.size());
                                    auto* dn_node = d_pv.pins[di].nodes;
                                    if(dn_node == nullptr)
                                    {
                                        ::fast_io::io::perr(::fast_io::err(), "            ", di, " ", dns, " = <null>\n");
                                    }
                                    else
                                    {
                                        auto const dns_state = dn_node->node_information.dn.state;
                                        ::fast_io::io::perr(::fast_io::err(), "            ", di, " ", dns, " = ", dns_state, "\n");
                                        if(dns_state == ::phy_engine::model::digital_node_statement_t::indeterminate_state)
                                        {
                                            ::fast_io::io::perr(::fast_io::err(), "              node pins:\n");
                                            for(auto* p2 : dn_node->pins)
                                            {
                                                if(p2 == nullptr || p2->model == nullptr || p2->model->ptr == nullptr) { continue; }
                                                auto const p2_mn = p2->model->ptr->get_model_name();
                                                std::string const p2_mns(reinterpret_cast<char const*>(p2_mn.data()), p2_mn.size());
                                                std::string const p2_inst(reinterpret_cast<char const*>(p2->model->name.data()), p2->model->name.size());
                                                std::string const p2_pin(reinterpret_cast<char const*>(p2->name.data()), p2->name.size());
                                                ::fast_io::io::perr(::fast_io::err(), "                ", p2_mns, " inst=", p2_inst, " pin=", p2_pin, "\n");
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        return std::nullopt;
    };

    auto read_row = [&](std::size_t y) -> std::uint8_t {
        std::uint8_t v{};
        for(std::size_t x{}; x < kW; ++x)
        {
            auto const pi = *row_port[y][x];
            auto const b = read_port_bit(pi);
            if(!b) { return 0xFF; }
            if(*b) { v |= static_cast<std::uint8_t>(1u << x); }
        }
        return v;
    };

    auto reset = [&] {
        set_in(in_left, false);
        set_in(in_right, false);
        set_in(in_rotate, false);

        // reset asserted (active-low)
        set_in(in_rst, false);
        tick();
        tick();
        // release reset
        set_in(in_rst, true);
        tick();  // spawn first piece
    };

    // 1) Reset + spawn: expect deterministic first piece (I horizontal), game_over=0.
    reset();
    {
        auto const go = read_port_bit(*game_over_port);
        if(!go) { return 13; }
        if(*go) { return 13; }
    }
    if(read_row(0) != 0b011110) { return 14; }

    // 2) Rotate (flip) should rotate the first I piece in-place (no fall due to priority).
    set_in(in_rotate, true);
    tick();
    set_in(in_rotate, false);

    {
        auto const go = read_port_bit(*game_over_port);
        if(!go) { return 16; }
        if(*go) { return 16; }
    }
    if(read_row(0) != 0b000010) { return 17; }
    if(read_row(1) != 0b000010) { return 18; }
    if(read_row(2) != 0b000010) { return 19; }
    if(read_row(3) != 0b000010) { return 20; }

    // 3) Run until game_over and check the "X" overlay.
    set_in(in_left, false);
    set_in(in_rotate, false);
    set_in(in_right, true);  // bias towards a quick loss

    bool saw_game_over{};
    for(std::size_t i{}; i < 4096; ++i)
    {
        tick();
        auto const go = read_port_bit(*game_over_port);
        if(!go) { return 21; }
        if(*go)
        {
            saw_game_over = true;
            break;
        }
    }
    if(!saw_game_over) { return 21; }

    if(read_row(0) != 0b100001) { return 22; }
    if(read_row(1) != 0b010010) { return 23; }
    if(read_row(2) != 0b001100) { return 24; }
    if(read_row(3) != 0b001100) { return 25; }
    if(read_row(4) != 0b010010) { return 26; }
    if(read_row(5) != 0b100001) { return 27; }

    // 4) Gate count check (after passing the sim checks).
    auto const gates = ::phy_engine::verilog::digital::details::count_logic_gates(nl);
    if(gates > 5000u) { return 28; }

    return 0;
}
