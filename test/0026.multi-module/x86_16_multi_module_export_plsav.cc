#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>
#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>

#include <cstddef>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
using ::phy_engine::phy_lab_wrapper::position;
using u8sv = ::fast_io::u8string_view;

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

static std::string base_name(std::string_view port_name)
{
    auto pos = port_name.find('[');
    if(pos == std::string_view::npos) { return std::string(port_name); }
    return std::string(port_name.substr(0, pos));
}

static std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
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

struct group_layout
{
    std::unordered_map<std::string, std::size_t> row_by_base{};
    std::unordered_map<std::string, std::size_t> width_by_base{};
    std::vector<std::string> order{};
};

static group_layout make_groups(std::vector<std::string> const& names)
{
    group_layout gl{};
    for(auto const& pn : names)
    {
        auto const base = base_name(pn);
        if(gl.width_by_base.find(base) == gl.width_by_base.end())
        {
            gl.order.push_back(base);
            gl.width_by_base.emplace(base, 1);
        }
        if(auto idx = parse_bit_index(pn, base))
        {
            auto& w = gl.width_by_base[base];
            w = std::max(w, *idx + 1);
        }
    }
    gl.row_by_base.reserve(gl.order.size());
    for(std::size_t i{}; i < gl.order.size(); ++i) { gl.row_by_base.emplace(gl.order[i], i); }
    return gl;
}

static double row_center_y(std::size_t row, std::size_t nrows, double y_min, double y_max)
{
    if(nrows <= 1) { return 0.5 * (y_min + y_max); }
    // Use row-centers (avoid placing IO exactly on the boundary of the top/bottom third).
    double const t = (static_cast<double>(row) + 0.5) / static_cast<double>(nrows);
    return y_max - (y_max - y_min) * t;
}

static double x_for_bit_lsb_right(std::size_t bit, std::size_t width, double x_min, double x_max)
{
    if(width <= 1) { return 0.5 * (x_min + x_max); }
    auto const ridx = (width - 1) - bit;
    double const t = static_cast<double>(ridx) / static_cast<double>(width - 1);
    return x_min + (x_max - x_min) * t;
}

static bool is_logic_output_element(::phy_engine::phy_lab_wrapper::element const& e)
{
    return e.data().value("ModelID", "") == ::phy_engine::phy_lab_wrapper::pl_model_id::logic_output;
}

static bool is_named_pin_element(::phy_engine::phy_lab_wrapper::element const& e, std::unordered_set<std::string> const& pin_names)
{
    if(!is_logic_output_element(e)) { return false; }
    auto it_label = e.data().find("Label");
    if(it_label == e.data().end() || !it_label->is_string()) { return false; }
    return pin_names.find(it_label->get<std::string>()) != pin_names.end();
}

static std::vector<::phy_engine::model::node_t*> make_bus(::phy_engine::netlist::netlist& nl, std::size_t width)
{
    std::vector<::phy_engine::model::node_t*> bus{};
    bus.reserve(width);
    for(std::size_t i{}; i < width; ++i) { bus.push_back(__builtin_addressof(::phy_engine::netlist::create_node(nl))); }
    return bus;
}

static std::vector<::phy_engine::model::model_base const*> snapshot_digital_models(::phy_engine::netlist::netlist const& nl)
{
    std::vector<::phy_engine::model::model_base const*> out{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal) { continue; }
            if(m->ptr == nullptr) { continue; }
            if(m->ptr->get_device_type() != ::phy_engine::model::model_device_type::digital) { continue; }
            out.push_back(m);
        }
    }
    return out;
}

struct compiled_top
{
    ::phy_engine::verilog::digital::compiled_design design{};
    ::phy_engine::verilog::digital::instance_state top{};
};

static compiled_top compile_top(std::filesystem::path const& path, ::fast_io::u8string_view top_name)
{
    auto const s = read_file_text(path);
    auto const src = u8sv{reinterpret_cast<char8_t const*>(s.data()), s.size()};

    ::phy_engine::verilog::digital::compile_options copt{};
    auto cr = ::phy_engine::verilog::digital::compile(src, copt);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        throw std::runtime_error("verilog compile failed: no modules");
    }

    auto design = ::phy_engine::verilog::digital::build_design(std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, top_name);
    if(mod == nullptr) { throw std::runtime_error("top module not found"); }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { throw std::runtime_error("elaborate failed"); }

    return compiled_top{std::move(design), std::move(top_inst)};
}

static void synth_one(::phy_engine::netlist::netlist& nl,
                      ::phy_engine::verilog::digital::instance_state const& top,
                      std::vector<::phy_engine::model::node_t*> const& port_nodes)
{
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_mul2 = true,
        .optimize_adders = true,
    };

    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top, port_nodes, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }
}

static void add_pin_outputs(::phy_engine::netlist::netlist& nl,
                            std::vector<std::string> const& pins,
                            std::unordered_map<std::string, ::phy_engine::model::node_t*>& node_by_pin)
{
    for(auto const& name : pins)
    {
        auto it = node_by_pin.find(name);
        if(it == node_by_pin.end() || it->second == nullptr) { throw std::runtime_error("missing node for pin: " + name); }

        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr || m->ptr == nullptr) { throw std::runtime_error("failed to add OUTPUT"); }
        m->name = ::fast_io::u8string{u8sv{reinterpret_cast<char8_t const*>(name.data()), name.size()}};
        if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *it->second)) { throw std::runtime_error("add_to_node failed"); }
    }
}
}  // namespace

int main()
{
    // Layout/placement constraints (requested):
    // - internal region per layer: (-1, -1/3, z) .. (1, +1/3, z)
    // - z increases by 0.1 per layer
    // - IO: top 1/3 = inputs, bottom 1/3 = outputs
    // - IO bit order: right->left is low->high
    constexpr double extent = 1.0;
    constexpr double third = 1.0 / 3.0;
    constexpr double z_step = 0.1;
    constexpr double layout_step = 0.01;
    constexpr double layout_min_step = 0.001;

    auto const dir = std::filesystem::path(__FILE__).parent_path();

    // PE circuit container.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    // Shared signals (vector buses stored MSB -> LSB to match port expansion order).
    auto& nclk = ::phy_engine::netlist::create_node(nl);
    auto& nrstn = ::phy_engine::netlist::create_node(nl);

    auto pc = make_bus(nl, 8);
    auto pc_next = make_bus(nl, 8);
    auto rom_data = make_bus(nl, 16);
    auto ir = make_bus(nl, 16);
    auto opcode = make_bus(nl, 4);
    auto reg_dst = make_bus(nl, 2);
    auto reg_src = make_bus(nl, 2);
    auto imm8 = make_bus(nl, 8);
    auto imm16 = make_bus(nl, 16);
    auto rf_waddr = make_bus(nl, 2);
    auto rf_raddr_a = make_bus(nl, 2);
    auto rf_raddr_b = make_bus(nl, 2);
    auto rf_rdata_a = make_bus(nl, 16);
    auto rf_rdata_b = make_bus(nl, 16);
    auto alu_b = make_bus(nl, 16);
    auto alu_y = make_bus(nl, 16);
    auto alu_op = make_bus(nl, 3);

    auto& n_pc_we = ::phy_engine::netlist::create_node(nl);
    auto& n_reg_we = ::phy_engine::netlist::create_node(nl);
    auto& n_alu_b_sel = ::phy_engine::netlist::create_node(nl);
    auto& n_flags_we_z = ::phy_engine::netlist::create_node(nl);
    auto& n_flags_we_c = ::phy_engine::netlist::create_node(nl);
    auto& n_flags_we_s = ::phy_engine::netlist::create_node(nl);
    auto& n_halt = ::phy_engine::netlist::create_node(nl);
    auto& n_alu_zf = ::phy_engine::netlist::create_node(nl);
    auto& n_alu_cf = ::phy_engine::netlist::create_node(nl);
    auto& n_alu_sf = ::phy_engine::netlist::create_node(nl);
    auto& n_flag_z = ::phy_engine::netlist::create_node(nl);
    auto& n_flag_c = ::phy_engine::netlist::create_node(nl);
    auto& n_flag_s = ::phy_engine::netlist::create_node(nl);

    auto dbg_r0 = make_bus(nl, 16);
    auto dbg_r1 = make_bus(nl, 16);
    auto dbg_r2 = make_bus(nl, 16);
    auto dbg_r3 = make_bus(nl, 16);

    // Convert each Verilog module independently to PE primitives and link via shared nodes.
    struct layer
    {
        std::string name{};
        std::vector<::phy_engine::model::model_base const*> pe_models{};
    };
    std::vector<layer> layers{};
    layers.reserve(12);

    auto synth_layer = [&](char const* layer_name,
                           std::filesystem::path const& vpath,
                           ::fast_io::u8string_view top_name,
                           std::vector<::phy_engine::model::node_t*> const& port_nodes) {
        auto before = snapshot_digital_models(nl);
        std::unordered_set<::phy_engine::model::model_base const*> before_set(before.begin(), before.end());

        auto ct = compile_top(vpath, top_name);
        synth_one(nl, ct.top, port_nodes);

        auto after = snapshot_digital_models(nl);
        layer lay{};
        lay.name = layer_name;
        for(auto* m : after)
        {
            if(before_set.find(m) == before_set.end()) { lay.pe_models.push_back(m); }
        }
        layers.push_back(std::move(lay));
    };

    // pc8(clk,rst_n,we,d[7:0],q[7:0])
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(19);
        ports.push_back(&nclk);
        ports.push_back(&nrstn);
        ports.push_back(&n_pc_we);
        for(auto* n : pc_next) { ports.push_back(n); }
        for(auto* n : pc) { ports.push_back(n); }
        synth_layer("pc8", dir / "pc8.v", u8"pc8", ports);
    }

    // rom256x16(addr[7:0], data[15:0])
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(24);
        for(auto* n : pc) { ports.push_back(n); }
        for(auto* n : rom_data) { ports.push_back(n); }
        synth_layer("rom256x16", dir / "rom256x16.v", u8"rom256x16", ports);
    }

    // ir16(clk,rst_n,d[15:0],q[15:0])
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(34);
        ports.push_back(&nclk);
        ports.push_back(&nrstn);
        for(auto* n : rom_data) { ports.push_back(n); }
        for(auto* n : ir) { ports.push_back(n); }
        synth_layer("ir16", dir / "ir16.v", u8"ir16", ports);
    }

    // decode16(instr[15:0], opcode[3:0], reg_dst[1:0], reg_src[1:0], imm8[7:0])
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(32);
        for(auto* n : ir) { ports.push_back(n); }
        for(auto* n : opcode) { ports.push_back(n); }
        for(auto* n : reg_dst) { ports.push_back(n); }
        for(auto* n : reg_src) { ports.push_back(n); }
        for(auto* n : imm8) { ports.push_back(n); }
        synth_layer("decode16", dir / "decode16.v", u8"decode16", ports);
    }

    // control16(opcode, reg_dst, reg_src, imm8, pc, flag_z, flag_c, flag_s,
    //           pc_next, pc_we, reg_we, rf_waddr, rf_raddr_a, rf_raddr_b, alu_b_sel,
    //           flags_we_z, flags_we_c, flags_we_s, alu_op, halt)
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(51);
        for(auto* n : opcode) { ports.push_back(n); }
        for(auto* n : reg_dst) { ports.push_back(n); }
        for(auto* n : reg_src) { ports.push_back(n); }
        for(auto* n : imm8) { ports.push_back(n); }
        for(auto* n : pc) { ports.push_back(n); }
        ports.push_back(&n_flag_z);
        ports.push_back(&n_flag_c);
        ports.push_back(&n_flag_s);
        for(auto* n : pc_next) { ports.push_back(n); }
        ports.push_back(&n_pc_we);
        ports.push_back(&n_reg_we);
        for(auto* n : rf_waddr) { ports.push_back(n); }
        for(auto* n : rf_raddr_a) { ports.push_back(n); }
        for(auto* n : rf_raddr_b) { ports.push_back(n); }
        ports.push_back(&n_alu_b_sel);
        ports.push_back(&n_flags_we_z);
        ports.push_back(&n_flags_we_c);
        ports.push_back(&n_flags_we_s);
        for(auto* n : alu_op) { ports.push_back(n); }
        ports.push_back(&n_halt);
        synth_layer("control16", dir / "control16.v", u8"control16", ports);
    }

    // imm_ext8_to_16(imm8[7:0], imm16[15:0])
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(24);
        for(auto* n : imm8) { ports.push_back(n); }
        for(auto* n : imm16) { ports.push_back(n); }
        synth_layer("imm_ext8_to_16", dir / "imm_ext8_to_16.v", u8"imm_ext8_to_16", ports);
    }

    // regfile4x16(clk,rst_n,we,waddr[1:0],wdata[15:0],raddr_a[1:0],raddr_b[1:0],
    //            rdata_a[15:0],rdata_b[15:0],dbg_r0,dbg_r1,dbg_r2,dbg_r3)
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(121);
        ports.push_back(&nclk);
        ports.push_back(&nrstn);
        ports.push_back(&n_reg_we);
        for(auto* n : rf_waddr) { ports.push_back(n); }
        for(auto* n : alu_y) { ports.push_back(n); }
        for(auto* n : rf_raddr_a) { ports.push_back(n); }
        for(auto* n : rf_raddr_b) { ports.push_back(n); }
        for(auto* n : rf_rdata_a) { ports.push_back(n); }
        for(auto* n : rf_rdata_b) { ports.push_back(n); }
        for(auto* n : dbg_r0) { ports.push_back(n); }
        for(auto* n : dbg_r1) { ports.push_back(n); }
        for(auto* n : dbg_r2) { ports.push_back(n); }
        for(auto* n : dbg_r3) { ports.push_back(n); }
        synth_layer("regfile4x16", dir / "regfile4x16.v", u8"regfile4x16", ports);
    }

    // mux16(sel, a[15:0], b[15:0], y[15:0])
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(49);
        ports.push_back(&n_alu_b_sel);
        for(auto* n : imm16) { ports.push_back(n); }
        for(auto* n : rf_rdata_b) { ports.push_back(n); }
        for(auto* n : alu_b) { ports.push_back(n); }
        synth_layer("mux16", dir / "mux16.v", u8"mux16", ports);
    }

    // flag1(clk,rst_n,we,d,q)  (Z/C/S flags)
    {
        {
            std::vector<::phy_engine::model::node_t*> ports{};
            ports.reserve(5);
            ports.push_back(&nclk);
            ports.push_back(&nrstn);
            ports.push_back(&n_flags_we_z);
            ports.push_back(&n_alu_zf);
            ports.push_back(&n_flag_z);
            synth_layer("flag_z", dir / "flag1.v", u8"flag1", ports);
        }
        {
            std::vector<::phy_engine::model::node_t*> ports{};
            ports.reserve(5);
            ports.push_back(&nclk);
            ports.push_back(&nrstn);
            ports.push_back(&n_flags_we_c);
            ports.push_back(&n_alu_cf);
            ports.push_back(&n_flag_c);
            synth_layer("flag_c", dir / "flag1.v", u8"flag1", ports);
        }
        {
            std::vector<::phy_engine::model::node_t*> ports{};
            ports.reserve(5);
            ports.push_back(&nclk);
            ports.push_back(&nrstn);
            ports.push_back(&n_flags_we_s);
            ports.push_back(&n_alu_sf);
            ports.push_back(&n_flag_s);
            synth_layer("flag_s", dir / "flag1.v", u8"flag1", ports);
        }
    }

    // alu16(op[2:0],a[15:0],b[15:0],y[15:0],zf,cf,sf)
    {
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(54);
        for(auto* n : alu_op) { ports.push_back(n); }
        for(auto* n : rf_rdata_a) { ports.push_back(n); }
        for(auto* n : alu_b) { ports.push_back(n); }
        for(auto* n : alu_y) { ports.push_back(n); }
        ports.push_back(&n_alu_zf);
        ports.push_back(&n_alu_cf);
        ports.push_back(&n_alu_sf);
        synth_layer("alu16", dir / "alu16.v", u8"alu16", ports);
    }

    // Add chip-level pins as PE OUTPUT models (so PL uses "Logic Output" for both inputs and outputs).
    std::vector<std::string> input_pins{
        "clk",
        "rst_n",
    };
    std::vector<std::string> output_pins{
        "halt",
    };
    for(int bit = 0; bit < 16; ++bit) { output_pins.push_back("dbg_r0[" + std::to_string(bit) + "]"); }
    for(int bit = 0; bit < 16; ++bit) { output_pins.push_back("dbg_r1[" + std::to_string(bit) + "]"); }

    std::unordered_map<std::string, ::phy_engine::model::node_t*> node_by_pin{};
    node_by_pin["clk"] = &nclk;
    node_by_pin["rst_n"] = &nrstn;
    node_by_pin["halt"] = &n_halt;
    for(int bit = 0; bit < 16; ++bit)
    {
        // bus vectors stored MSB->LSB, but pin names are [bit] index (LSB=0).
        node_by_pin["dbg_r0[" + std::to_string(bit) + "]"] = dbg_r0[static_cast<std::size_t>(15 - bit)];
        node_by_pin["dbg_r1[" + std::to_string(bit) + "]"] = dbg_r1[static_cast<std::size_t>(15 - bit)];
    }

    add_pin_outputs(nl, input_pins, node_by_pin);
    add_pin_outputs(nl, output_pins, node_by_pin);

    // Convert to PhysicsLab experiment with 3D coordinates + wires.
    ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
    popt.fixed_pos = {0.0, 0.0, 0.0};
    // Use native coordinates so IO spans the full [-1,1] range as requested; PhysicsLab stores positions as "x,z,y".
    popt.element_xyz_coords = false;
    popt.generate_wires = true;
    popt.keep_pl_macros = true;

    auto const input_groups = make_groups(input_pins);
    auto const output_groups = make_groups(output_pins);
    std::unordered_set<std::string> all_pins{};
    all_pins.reserve(input_pins.size() + output_pins.size());
    for(auto const& s : input_pins) { all_pins.insert(s); }
    for(auto const& s : output_pins) { all_pins.insert(s); }

    popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx)
        -> std::optional<position> {
        // All pins are exported as "Logic Output". Place them by name membership.
        std::string_view const pn = ctx.pe_instance_name;
        if(pn.empty()) { return std::nullopt; }

        std::string s(pn);
        if(all_pins.find(s) == all_pins.end()) { return std::nullopt; }

        auto const base = base_name(s);
        std::size_t bit{};
        if(auto idx = parse_bit_index(s, base)) { bit = *idx; }

        // IO regions (z fixed at 0.0 for pins).
        double const in_y_min = extent * third;
        double const in_y_max = extent;
        double const out_y_min = -extent;
        double const out_y_max = -extent * third;

        if(input_groups.row_by_base.find(base) != input_groups.row_by_base.end())
        {
            auto const row = input_groups.row_by_base.at(base);
            auto const nrows = std::max<std::size_t>(1, input_groups.order.size());
            auto const y = row_center_y(row, nrows, in_y_min, in_y_max);
            auto const width = input_groups.width_by_base.at(base);
            auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);
            return position{x, y, 0.0};
        }

        if(output_groups.row_by_base.find(base) != output_groups.row_by_base.end())
        {
            auto const row = output_groups.row_by_base.at(base);
            auto const nrows = std::max<std::size_t>(1, output_groups.order.size());
            auto const y = row_center_y(row, nrows, out_y_min, out_y_max);
            auto const width = output_groups.width_by_base.at(base);
            auto const x = x_for_bit_lsb_right(bit, width, -extent, extent);
            return position{x, y, 0.0};
        }

        return std::nullopt;
    };

    auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
    if(!r.warnings.empty())
    {
        std::unordered_set<std::string> uniq{};
        for(auto const& w : r.warnings)
        {
            if(uniq.insert(w).second) { std::fprintf(stderr, "[pe_to_pl] %s\n", w.c_str()); }
        }
    }

    // Fix IO pins (do not participate in layout).
    for(auto const& e : r.ex.elements())
    {
        if(is_named_pin_element(e, all_pins))
        {
            r.ex.get_element(e.identifier()).set_participate_in_layout(false);
        }
    }

    // Per-module 3D layout: stack each synthesized module on its own Z plane.
    ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
    aopt.mode = ::phy_engine::phy_lab_wrapper::auto_layout::mode::hierarchical;
    aopt.respect_fixed_elements = false;  // do not treat other layers as obstacles
    aopt.small_element = {1, 1};
    aopt.big_element = {2, 2};
    aopt.margin_x = 1e-6;
    aopt.margin_y = 1e-6;
    aopt.step_x = layout_step;
    aopt.step_y = layout_step;

    // Collect element ids for each layer (PE-model set -> PL element ids).
    std::vector<std::vector<std::string>> layer_element_ids{};
    layer_element_ids.reserve(layers.size());
    for(auto const& lay : layers)
    {
        std::vector<std::string> ids{};
        ids.reserve(lay.pe_models.size());
        for(auto const* m : lay.pe_models)
        {
            auto it = r.element_by_pe_model.find(m);
            if(it != r.element_by_pe_model.end()) { ids.push_back(it->second); }
        }
        layer_element_ids.push_back(std::move(ids));
    }

    auto set_participation = [&](std::vector<std::string> const& allow_ids) {
        std::unordered_set<std::string> allow{};
        allow.reserve(allow_ids.size());
        for(auto const& id : allow_ids) { allow.insert(id); }
        for(auto const& e : r.ex.elements())
        {
            auto const id = e.identifier();
            if(is_named_pin_element(e, all_pins))
            {
                r.ex.get_element(id).set_participate_in_layout(false);
                continue;
            }
            r.ex.get_element(id).set_participate_in_layout(allow.find(id) != allow.end());
        }
    };

    for(std::size_t i{}; i < layer_element_ids.size(); ++i)
    {
        double const z = z_step * static_cast<double>(i + 1);  // start above IO plane
        set_participation(layer_element_ids[i]);
        // Run layout on this layer.
        ::phy_engine::phy_lab_wrapper::auto_layout::stats st{};
        for(int attempt = 0; attempt < 8; ++attempt)
        {
            st = ::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex,
                                                                    position{-extent, -extent * third, 0.0},
                                                                    position{extent, extent * third, 0.0},
                                                                    z,
                                                                    aopt);
            if(st.skipped == 0) { break; }
            aopt.step_x = std::max(aopt.step_x * 0.9, layout_min_step);
            aopt.step_y = std::max(aopt.step_y * 0.9, layout_min_step);
        }
    }

    // Save.
    // Keep numbers short to shrink .sav size (layout grid is at most 0.001 anyway).
    r.ex.set_xyz_precision(2);
    auto const out_path = std::filesystem::path("x86_16_multi_module_3d.sav");
    // Keep the .sav small for sharing/publishing.
    r.ex.save(out_path, -1);
    if(!std::filesystem::exists(out_path))
    {
        std::fprintf(stderr, "error: failed to write %s\n", out_path.string().c_str());
        return 2;
    }
    std::fprintf(stderr, "wrote %s\n", out_path.string().c_str());
    return 0;
}
