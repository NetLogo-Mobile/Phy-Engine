#include <phy_engine/phy_engine.h>

#include <phy_engine/model/models/digital/logical/yes.h>

#include <fast_io/fast_io.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
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

std::vector<::phy_engine::model::node_t*> make_bus(::phy_engine::netlist::netlist& nl, std::size_t width)
{
    std::vector<::phy_engine::model::node_t*> bus{};
    bus.reserve(width);
    for(std::size_t i{}; i < width; ++i) { bus.push_back(__builtin_addressof(create_node(nl))); }
    return bus;
}

void connect_bus(::phy_engine::netlist::netlist& nl,
                 ::phy_engine::model::model_base& mb,
                 std::size_t pin_start,
                 std::vector<::phy_engine::model::node_t*> const& bus)
{
    for(std::size_t i{}; i < bus.size(); ++i) { add_to_node(nl, mb, pin_start + i, *bus[i]); }
}

std::uint16_t read_u16(std::vector<::phy_engine::model::node_t*> const& bus_msb_to_lsb)
{
    std::uint16_t v{};
    auto const n = bus_msb_to_lsb.size();
    for(std::size_t i{}; i < n; ++i)
    {
        auto const bit = static_cast<unsigned>((n - 1) - i);
        auto const st = bus_msb_to_lsb[i]->node_information.dn.state;
        if(st == ::phy_engine::model::digital_node_statement_t::true_state) { v |= static_cast<std::uint16_t>(1u << bit); }
    }
    return v;
}

std::uint16_t read_u16_n(std::vector<::phy_engine::model::node_t*> const& bus_msb_to_lsb, std::uint16_t mask)
{
    return static_cast<std::uint16_t>(read_u16(bus_msb_to_lsb) & mask);
}

bool read_bool(::phy_engine::model::node_t const& n)
{
    return n.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state;
}

bool read_bool_strict(::phy_engine::model::node_t const& n, char const* name)
{
    auto const st = n.node_information.dn.state;
    if(st != ::phy_engine::model::digital_node_statement_t::true_state && st != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        std::fprintf(stderr, "%s is not binary (X/Z)\n", (name == nullptr ? "node" : name));
        std::abort();
    }
    return st == ::phy_engine::model::digital_node_statement_t::true_state;
}

std::optional<bool> try_read_bool_binary(::phy_engine::model::node_t const& n) noexcept
{
    auto const st = n.node_information.dn.state;
    if(st == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
    if(st == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
    return std::nullopt;
}

std::optional<std::uint16_t> try_read_u16_binary(std::vector<::phy_engine::model::node_t*> const& bus_msb_to_lsb) noexcept
{
    std::uint16_t v{};
    auto const n = bus_msb_to_lsb.size();
    for(std::size_t i{}; i < n; ++i)
    {
        auto const bit = static_cast<unsigned>((n - 1) - i);
        auto const st = bus_msb_to_lsb[i]->node_information.dn.state;
        if(st == ::phy_engine::model::digital_node_statement_t::true_state) { v |= static_cast<std::uint16_t>(1u << bit); }
        else if(st == ::phy_engine::model::digital_node_statement_t::false_state) { /* ok */ }
        else { return std::nullopt; }
    }
    return v;
}

std::optional<std::uint8_t> try_read_u8_binary(std::vector<::phy_engine::model::node_t*> const& bus_msb_to_lsb) noexcept
{
    auto v = try_read_u16_binary(bus_msb_to_lsb);
    if(!v) { return std::nullopt; }
    return static_cast<std::uint8_t>(*v & 0xffu);
}

static void add_yes_buffer(::phy_engine::netlist::netlist& nl, ::phy_engine::model::node_t& in, ::phy_engine::model::node_t& out)
{
    auto [m, pos] = add_model(nl, ::phy_engine::model::YES{});
    (void)pos;
    if(m == nullptr || m->ptr == nullptr) { throw std::runtime_error("failed to add YES"); }
    add_to_node(nl, *m, 0, in);
    add_to_node(nl, *m, 1, out);
}

static void add_yes_buffer_bus(::phy_engine::netlist::netlist& nl,
                               std::vector<::phy_engine::model::node_t*> const& in_msb_to_lsb,
                               std::vector<::phy_engine::model::node_t*> const& out_msb_to_lsb)
{
    if(in_msb_to_lsb.size() != out_msb_to_lsb.size()) { throw std::runtime_error("bus width mismatch"); }
    for(std::size_t i{}; i < in_msb_to_lsb.size(); ++i)
    {
        if(in_msb_to_lsb[i] == nullptr || out_msb_to_lsb[i] == nullptr) { throw std::runtime_error("null bus node"); }
        add_yes_buffer(nl, *in_msb_to_lsb[i], *out_msb_to_lsb[i]);
    }
}

std::string bin16(std::uint16_t v)
{
    std::string s;
    s.resize(16);
    for(int i = 15; i >= 0; --i) { s[15 - i] = ((v >> i) & 1u) ? '1' : '0'; }
    return s;
}

std::uint64_t timestamp_to_ns(::fast_io::unix_timestamp ts)
{
    auto const seconds = static_cast<std::uint64_t>(ts.seconds);
    auto const subseconds = static_cast<std::uint64_t>(ts.subseconds);

    constexpr std::uint64_t ns_per_s = 1000'000'000ull;
    constexpr std::uint64_t subsec_per_s = static_cast<std::uint64_t>(::fast_io::uint_least64_subseconds_per_second);

    auto const ns_from_seconds = static_cast<__uint128_t>(seconds) * ns_per_s;
    auto const ns_from_subseconds = (static_cast<__uint128_t>(subseconds) * ns_per_s) / subsec_per_s;
    return static_cast<std::uint64_t>(ns_from_seconds + ns_from_subseconds);
}
}  // namespace

int main()
{
    // This executable is meant for correlating the exported PhysicsLab (.sav) pins
    // with per-clock-step behavior. Enable printing with:
    //   PHY_ENGINE_TRACE_0026_PLSAV=1
    bool const trace = (std::getenv("PHY_ENGINE_TRACE_0026_PLSAV") != nullptr);
    bool const trace_layers = (std::getenv("PHY_ENGINE_TRACE_0026_PLSAV_LAYERS") != nullptr);
    bool const expect = (std::getenv("PHY_ENGINE_TRACE_0026_PLSAV_EXPECT") != nullptr);
    bool const probe_pc_path = (std::getenv("PHY_ENGINE_TRACE_0026_PLSAV_PROBE_PC_NEXT") != nullptr) ||
                               (std::getenv("PHY_ENGINE_TRACE_0026_EXPORT_PROBE_PC_NEXT") != nullptr);

    auto const dir = std::filesystem::path(__FILE__).parent_path();

    auto const pc8_s = read_file_text(dir / "pc8.v");
    auto const ir16_s = read_file_text(dir / "ir16.v");
    auto const rom_s = read_file_text(dir / "rom256x16.v");
    auto const dec_s = read_file_text(dir / "decode16.v");
    auto const ctl_s = read_file_text(dir / "control16.v");
    auto const imm_s = read_file_text(dir / "imm_ext8_to_16.v");
    auto const rf_s = read_file_text(dir / "regfile4x16.v");
    auto const mux_s = read_file_text(dir / "mux16.v");
    auto const alu_addsub_s = read_file_text(dir / "alu16_addsub.v");
    auto const alu_and_s = read_file_text(dir / "alu16_and.v");
    auto const alu_or_s = read_file_text(dir / "alu16_or.v");
    auto const alu_xor_s = read_file_text(dir / "alu16_xor.v");
    auto const alu_mov_s = read_file_text(dir / "alu16_mov.v");
    auto const alu_shl_s = read_file_text(dir / "alu16_shl.v");
    auto const alu_shr_s = read_file_text(dir / "alu16_shr.v");
    auto const alu_sub_dec_s = read_file_text(dir / "alu16_sub_decode.v");
    auto const alu_sel_s = read_file_text(dir / "alu16_select.v");
    auto const alu_top_s = read_file_text(dir / "alu16.v");

    std::string alu_s{};
    alu_s.reserve(alu_addsub_s.size() + alu_and_s.size() + alu_or_s.size() + alu_xor_s.size() + alu_mov_s.size() +
                  alu_shl_s.size() + alu_shr_s.size() + alu_sel_s.size() + alu_top_s.size() + 16);
    alu_s.append(alu_addsub_s).append("\n");
    alu_s.append(alu_and_s).append("\n");
    alu_s.append(alu_or_s).append("\n");
    alu_s.append(alu_xor_s).append("\n");
    alu_s.append(alu_mov_s).append("\n");
    alu_s.append(alu_shl_s).append("\n");
    alu_s.append(alu_shr_s).append("\n");
    alu_s.append(alu_sub_dec_s).append("\n");
    alu_s.append(alu_sel_s).append("\n");
    alu_s.append(alu_top_s).append("\n");

    auto const flg_s = read_file_text(dir / "flag1.v");

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    c.get_analyze_setting().tr.t_step = 1e-9;
    c.get_analyze_setting().tr.t_stop = 1e-9;

    auto& nl = c.get_netlist();

    auto [in_clk, p0] = add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_rstn, p1] = add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    (void)p0;
    (void)p1;
    if(in_clk == nullptr || in_rstn == nullptr) { return 1; }

    auto [m_rom, p_rom] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                       ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(rom_s.data()), rom_s.size()},
                                       u8"rom256x16"));
    auto [m_ir, p_ir] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(ir16_s.data()), ir16_s.size()},
                                      u8"ir16"));
    auto [m_dec, p_dec] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                       ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(dec_s.data()), dec_s.size()},
                                       u8"decode16"));
    auto [m_ctl, p_ctl] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                       ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(ctl_s.data()), ctl_s.size()},
                                       u8"control16"));
    auto [m_imm, p_imm] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                       ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(imm_s.data()), imm_s.size()},
                                       u8"imm_ext8_to_16"));
    auto [m_mux, p_mux] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                       ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(mux_s.data()), mux_s.size()},
                                       u8"mux16"));
    auto [m_rf, p_rf] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(rf_s.data()), rf_s.size()},
                                      u8"regfile4x16"));
    auto [m_flg_z, p_flg_z] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                            ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(flg_s.data()), flg_s.size()},
                                            u8"flag1"));
    auto [m_flg_c, p_flg_c] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                            ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(flg_s.data()), flg_s.size()},
                                            u8"flag1"));
    auto [m_flg_s, p_flg_s] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                            ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(flg_s.data()), flg_s.size()},
                                            u8"flag1"));
    // Ensure flag registers sample the *pre-edge* ALU flags (before ALU recomputes from post-write state).
    auto [m_alu, p_alu] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                       ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(alu_s.data()), alu_s.size()},
                                       u8"alu16"));
    auto [m_pc, p_pc] = add_model(nl, ::phy_engine::model::make_verilog_module(
                                      ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(pc8_s.data()), pc8_s.size()},
                                      u8"pc8"));
    (void)p_pc;
    (void)p_rom;
    (void)p_ir;
    (void)p_dec;
    (void)p_ctl;
    (void)p_imm;
    (void)p_mux;
    (void)p_rf;
    (void)p_flg_z;
    (void)p_flg_c;
    (void)p_flg_s;
    (void)p_alu;

    if(m_pc == nullptr || m_rom == nullptr || m_ir == nullptr || m_dec == nullptr || m_ctl == nullptr || m_imm == nullptr || m_mux == nullptr ||
       m_rf == nullptr || m_alu == nullptr || m_flg_z == nullptr || m_flg_c == nullptr || m_flg_s == nullptr)
    {
        return 2;
    }

    auto& nclk = create_node(nl);
    auto& nrstn = create_node(nl);

    auto pc = make_bus(nl, 8);
    auto pc_next = make_bus(nl, 8);  // pc8.d side (post-probe if enabled)
    auto pc_ctl = pc;                // control16.pc side (post-probe if enabled)
    auto pc_next_ctl = pc_next;      // control16.pc_next side (pre-probe if enabled)
    if(probe_pc_path)
    {
        pc_ctl = make_bus(nl, 8);
        pc_next_ctl = make_bus(nl, 8);
    }
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

    auto& n_pc_we = create_node(nl);
    auto& n_reg_we = create_node(nl);
    auto& n_alu_b_sel = create_node(nl);
    auto& n_flags_we_z = create_node(nl);
    auto& n_flags_we_c = create_node(nl);
    auto& n_flags_we_s = create_node(nl);
    auto& n_halt = create_node(nl);
    auto& n_alu_zf = create_node(nl);
    auto& n_alu_cf = create_node(nl);
    auto& n_alu_sf = create_node(nl);
    auto& n_flag_z = create_node(nl);
    auto& n_flag_c = create_node(nl);
    auto& n_flag_s = create_node(nl);

    auto dbg_r0 = make_bus(nl, 16);
    auto dbg_r1 = make_bus(nl, 16);
    auto dbg_r2 = make_bus(nl, 16);
    auto dbg_r3 = make_bus(nl, 16);

    add_to_node(nl, *in_clk, 0, nclk);
    add_to_node(nl, *in_rstn, 0, nrstn);

    // pc8(clk,rst_n,we,d[7:0],q[7:0])
    add_to_node(nl, *m_pc, 0, nclk);
    add_to_node(nl, *m_pc, 1, nrstn);
    add_to_node(nl, *m_pc, 2, n_pc_we);
    connect_bus(nl, *m_pc, 3, pc_next);
    connect_bus(nl, *m_pc, 11, pc);

    if(probe_pc_path)
    {
        // Non-intrusive (identity) probe buffers at the PC/control boundary:
        //   pc -> pc_ctl -> control16 -> pc_next_ctl -> pc_next -> pc8.d
        add_yes_buffer_bus(nl, pc, pc_ctl);
        add_yes_buffer_bus(nl, pc_next_ctl, pc_next);
    }

    // rom256x16(addr[7:0], data[15:0])
    connect_bus(nl, *m_rom, 0, pc);
    connect_bus(nl, *m_rom, 8, rom_data);

    // ir16(clk,rst_n,d[15:0],q[15:0])
    add_to_node(nl, *m_ir, 0, nclk);
    add_to_node(nl, *m_ir, 1, nrstn);
    connect_bus(nl, *m_ir, 2, rom_data);
    connect_bus(nl, *m_ir, 18, ir);

    // decode16(instr[15:0], opcode[3:0], reg_dst[1:0], reg_src[1:0], imm8[7:0])
    connect_bus(nl, *m_dec, 0, ir);
    connect_bus(nl, *m_dec, 16, opcode);
    connect_bus(nl, *m_dec, 20, reg_dst);
    connect_bus(nl, *m_dec, 22, reg_src);
    connect_bus(nl, *m_dec, 24, imm8);

    // control16(...)
    connect_bus(nl, *m_ctl, 0, opcode);
    connect_bus(nl, *m_ctl, 4, reg_dst);
    connect_bus(nl, *m_ctl, 6, reg_src);
    connect_bus(nl, *m_ctl, 8, imm8);
    connect_bus(nl, *m_ctl, 16, pc_ctl);
    add_to_node(nl, *m_ctl, 24, n_flag_z);
    add_to_node(nl, *m_ctl, 25, n_flag_c);
    add_to_node(nl, *m_ctl, 26, n_flag_s);
    connect_bus(nl, *m_ctl, 27, pc_next_ctl);
    add_to_node(nl, *m_ctl, 35, n_pc_we);
    add_to_node(nl, *m_ctl, 36, n_reg_we);
    connect_bus(nl, *m_ctl, 37, rf_waddr);
    connect_bus(nl, *m_ctl, 39, rf_raddr_a);
    connect_bus(nl, *m_ctl, 41, rf_raddr_b);
    add_to_node(nl, *m_ctl, 43, n_alu_b_sel);
    add_to_node(nl, *m_ctl, 44, n_flags_we_z);
    add_to_node(nl, *m_ctl, 45, n_flags_we_c);
    add_to_node(nl, *m_ctl, 46, n_flags_we_s);
    connect_bus(nl, *m_ctl, 47, alu_op);
    add_to_node(nl, *m_ctl, 50, n_halt);

    // imm_ext8_to_16(imm8[7:0], imm16[15:0])
    connect_bus(nl, *m_imm, 0, imm8);
    connect_bus(nl, *m_imm, 8, imm16);

    // regfile4x16(...)
    add_to_node(nl, *m_rf, 0, nclk);
    add_to_node(nl, *m_rf, 1, nrstn);
    add_to_node(nl, *m_rf, 2, n_reg_we);
    connect_bus(nl, *m_rf, 3, rf_waddr);
    connect_bus(nl, *m_rf, 5, alu_y);
    connect_bus(nl, *m_rf, 21, rf_raddr_a);
    connect_bus(nl, *m_rf, 23, rf_raddr_b);
    connect_bus(nl, *m_rf, 25, rf_rdata_a);
    connect_bus(nl, *m_rf, 41, rf_rdata_b);
    connect_bus(nl, *m_rf, 57, dbg_r0);
    connect_bus(nl, *m_rf, 73, dbg_r1);
    connect_bus(nl, *m_rf, 89, dbg_r2);
    connect_bus(nl, *m_rf, 105, dbg_r3);

    // mux16(sel, a[15:0], b[15:0], y[15:0])
    add_to_node(nl, *m_mux, 0, n_alu_b_sel);
    connect_bus(nl, *m_mux, 1, imm16);
    connect_bus(nl, *m_mux, 17, rf_rdata_b);
    connect_bus(nl, *m_mux, 33, alu_b);

    // alu16(op[2:0],a[15:0],b[15:0],y[15:0],zf,cf,sf)
    connect_bus(nl, *m_alu, 0, alu_op);
    connect_bus(nl, *m_alu, 3, rf_rdata_a);
    connect_bus(nl, *m_alu, 19, alu_b);
    connect_bus(nl, *m_alu, 35, alu_y);
    add_to_node(nl, *m_alu, 51, n_alu_zf);
    add_to_node(nl, *m_alu, 52, n_alu_cf);
    add_to_node(nl, *m_alu, 53, n_alu_sf);

    // flag1(clk,rst_n,we,d,q)  (Z/C/S flags)
    add_to_node(nl, *m_flg_z, 0, nclk);
    add_to_node(nl, *m_flg_z, 1, nrstn);
    add_to_node(nl, *m_flg_z, 2, n_flags_we_z);
    add_to_node(nl, *m_flg_z, 3, n_alu_zf);
    add_to_node(nl, *m_flg_z, 4, n_flag_z);

    add_to_node(nl, *m_flg_c, 0, nclk);
    add_to_node(nl, *m_flg_c, 1, nrstn);
    add_to_node(nl, *m_flg_c, 2, n_flags_we_c);
    add_to_node(nl, *m_flg_c, 3, n_alu_cf);
    add_to_node(nl, *m_flg_c, 4, n_flag_c);

    add_to_node(nl, *m_flg_s, 0, nclk);
    add_to_node(nl, *m_flg_s, 1, nrstn);
    add_to_node(nl, *m_flg_s, 2, n_flags_we_s);
    add_to_node(nl, *m_flg_s, 3, n_alu_sf);
    add_to_node(nl, *m_flg_s, 4, n_flag_s);

    if(!c.analyze()) { return 3; }

    auto set_clk = [&](bool v) {
        in_clk->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                           : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto set_rstn = [&](bool v) {
        in_rstn->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto settle = [&](int iters) {
        for(int i = 0; i < iters; ++i) { c.digital_clk(); }
    };

    int step{};
    auto const t0 = ::fast_io::posix_clock_gettime(::fast_io::posix_clock_id::monotonic_raw);
    std::uint64_t sim_step_ns{};

    struct snap
    {
        std::optional<bool> clk{};
        std::optional<bool> rst_n{};
        std::optional<bool> halt{};
        std::optional<bool> pc_we{};
        std::optional<std::uint8_t> pc{};
        std::optional<std::uint8_t> pc_ctl{};
        std::optional<std::uint8_t> pc_next_ctl{};
        std::optional<std::uint8_t> pc_next{};
        std::optional<std::uint16_t> rom_data{};
        std::optional<std::uint16_t> ir{};
    };

    auto snapshot = [&]() -> snap {
        snap s{};
        s.clk = try_read_bool_binary(nclk);
        s.rst_n = try_read_bool_binary(nrstn);
        s.halt = try_read_bool_binary(n_halt);
        s.pc_we = try_read_bool_binary(n_pc_we);
        s.pc = try_read_u8_binary(pc);
        s.pc_ctl = try_read_u8_binary(pc_ctl);
        s.pc_next_ctl = try_read_u8_binary(pc_next_ctl);
        s.pc_next = try_read_u8_binary(pc_next);
        s.rom_data = try_read_u16_binary(rom_data);
        s.ir = try_read_u16_binary(ir);
        return s;
    };

    auto dump = [&](char const* tag) {
        if(!trace) { return; }
        auto const t_now = ::fast_io::posix_clock_gettime(::fast_io::posix_clock_id::monotonic_raw);
        auto const t_ns = timestamp_to_ns(t_now - t0);

        auto const clk = read_bool(nclk);
        auto const rst_n = read_bool(nrstn);
        auto const halt = read_bool(n_halt);
        auto const pc_v = static_cast<unsigned>(read_u16(pc) & 0xffu);
        auto const ir_v = static_cast<unsigned>(read_u16(ir));
        auto const r0 = read_u16(dbg_r0);
        auto const r1 = read_u16(dbg_r1);
        std::fprintf(stdout,
                     "[%s] step=%03d sim_ns=%llu t_ns=%llu clk=%d rst_n=%d halt=%d pc=%02x ir=%04x dbg_r0=%04x %s dbg_r1=%04x %s\n",
                     tag,
                     step++,
                     static_cast<unsigned long long>(sim_step_ns),
                     static_cast<unsigned long long>(t_ns),
                     clk ? 1 : 0,
                     rst_n ? 1 : 0,
                     halt ? 1 : 0,
                     pc_v,
                     ir_v,
                     static_cast<unsigned>(r0),
                     bin16(r0).c_str(),
                     static_cast<unsigned>(r1),
                     bin16(r1).c_str());

        if(trace_layers)
        {
            // Keep this to <=10 "layer groups" for readability.
            auto const pc_ctl_v = static_cast<unsigned>(read_u16_n(pc_ctl, 0xffu));
            auto const pc_next_ctl_v = static_cast<unsigned>(read_u16_n(pc_next_ctl, 0xffu));
            auto const pc_next_v = static_cast<unsigned>(read_u16_n(pc_next, 0xffu));
            auto const pc_we = read_bool(n_pc_we);
            auto const rom_v = static_cast<unsigned>(read_u16(rom_data));

            auto const op_v = static_cast<unsigned>(read_u16_n(opcode, 0x0fu));
            auto const dst_v = static_cast<unsigned>(read_u16_n(reg_dst, 0x03u));
            auto const src_v = static_cast<unsigned>(read_u16_n(reg_src, 0x03u));
            auto const imm8_v = static_cast<unsigned>(read_u16_n(imm8, 0xffu));

            auto const reg_we = read_bool(n_reg_we);
            auto const waddr_v = static_cast<unsigned>(read_u16_n(rf_waddr, 0x03u));
            auto const ra_v = static_cast<unsigned>(read_u16_n(rf_raddr_a, 0x03u));
            auto const rb_v = static_cast<unsigned>(read_u16_n(rf_raddr_b, 0x03u));
            auto const bsel = read_bool(n_alu_b_sel);
            auto const alu_op_v = static_cast<unsigned>(read_u16_n(alu_op, 0x07u));
            auto const fwez = read_bool(n_flags_we_z);
            auto const fwec = read_bool(n_flags_we_c);
            auto const fwes = read_bool(n_flags_we_s);

            auto const imm16_v = static_cast<unsigned>(read_u16(imm16));
            auto const rda_v = static_cast<unsigned>(read_u16(rf_rdata_a));
            auto const rdb_v = static_cast<unsigned>(read_u16(rf_rdata_b));
            auto const alu_b_v = static_cast<unsigned>(read_u16(alu_b));
            auto const alu_y_v = static_cast<unsigned>(read_u16(alu_y));
            auto const zf = read_bool(n_alu_zf);
            auto const cf = read_bool(n_alu_cf);
            auto const sf = read_bool(n_alu_sf);
            auto const fz = read_bool(n_flag_z);
            auto const fc = read_bool(n_flag_c);
            auto const fs = read_bool(n_flag_s);

            if(probe_pc_path)
            {
                std::fprintf(stdout,
                             "  [pc/probe] pc_ctl=%02x | [ctl->pc] next_ctl=%02x next=%02x we=%d | [rom] data=%04x | [dec] op=%x dst=%u src=%u imm8=%02x |"
                             " [ctl] reg_we=%d wa=%u ra=%u rb=%u bsel=%d alu_op=%u fwe(zcs)=%d%d%d |"
                             " [imm] imm16=%04x | [rf] rA=%04x rB=%04x | [mux] b=%04x | [alu] y=%04x zcs=%d%d%d | [flg] zcs=%d%d%d\n",
                             pc_ctl_v,
                             pc_next_ctl_v,
                             pc_next_v,
                             pc_we ? 1 : 0,
                             rom_v,
                             op_v,
                             dst_v,
                             src_v,
                             imm8_v,
                             reg_we ? 1 : 0,
                             waddr_v,
                             ra_v,
                             rb_v,
                             bsel ? 1 : 0,
                             alu_op_v,
                             fwez ? 1 : 0,
                             fwec ? 1 : 0,
                             fwes ? 1 : 0,
                             imm16_v,
                             rda_v,
                             rdb_v,
                             alu_b_v,
                             alu_y_v,
                             zf ? 1 : 0,
                             cf ? 1 : 0,
                             sf ? 1 : 0,
                             fz ? 1 : 0,
                             fc ? 1 : 0,
                             fs ? 1 : 0);
            }
            else
            {
                std::fprintf(stdout,
                             "  [pc8] next=%02x we=%d | [rom] data=%04x | [dec] op=%x dst=%u src=%u imm8=%02x |"
                             " [ctl] reg_we=%d wa=%u ra=%u rb=%u bsel=%d alu_op=%u fwe(zcs)=%d%d%d |"
                             " [imm] imm16=%04x | [rf] rA=%04x rB=%04x | [mux] b=%04x | [alu] y=%04x zcs=%d%d%d | [flg] zcs=%d%d%d\n",
                             pc_next_v,
                             pc_we ? 1 : 0,
                             rom_v,
                             op_v,
                             dst_v,
                             src_v,
                             imm8_v,
                             reg_we ? 1 : 0,
                             waddr_v,
                             ra_v,
                             rb_v,
                             bsel ? 1 : 0,
                             alu_op_v,
                             fwez ? 1 : 0,
                             fwec ? 1 : 0,
                             fwes ? 1 : 0,
                             imm16_v,
                             rda_v,
                             rdb_v,
                             alu_b_v,
                             alu_y_v,
                             zf ? 1 : 0,
                             cf ? 1 : 0,
                             sf ? 1 : 0,
                             fz ? 1 : 0,
                             fc ? 1 : 0,
                             fs ? 1 : 0);
            }
        }
    };

    auto do_step = [&](char const* tag, std::optional<bool> expected_clk, auto&& apply_inputs) {
        auto const t_step0 = ::fast_io::posix_clock_gettime(::fast_io::posix_clock_id::monotonic_raw);
        apply_inputs();
        settle(4);
        if(expected_clk)
        {
            auto const got = read_bool_strict(nclk, "clk");
            if(got != *expected_clk) { throw std::runtime_error(std::string("clk check failed: ") + tag); }
        }
        auto const t_step1 = ::fast_io::posix_clock_gettime(::fast_io::posix_clock_id::monotonic_raw);
        sim_step_ns = timestamp_to_ns(t_step1 - t_step0);
        dump(tag);
    };

    auto check_probe_equal = [&](snap const& s, char const* where) {
        if(!probe_pc_path) { return; }
        if(s.pc && s.pc_ctl && *s.pc != *s.pc_ctl)
        {
            std::fprintf(stderr, "[expect] %s: pc_ctl mismatch: pc=%02x pc_ctl=%02x\n", where, *s.pc, *s.pc_ctl);
            std::abort();
        }
        if(s.pc_next && s.pc_next_ctl && *s.pc_next != *s.pc_next_ctl)
        {
            std::fprintf(stderr, "[expect] %s: pc_next_ctl mismatch: pc_next=%02x pc_next_ctl=%02x\n", where, *s.pc_next, *s.pc_next_ctl);
            std::abort();
        }
    };

    auto require_u8 = [&](std::optional<std::uint8_t> v, char const* name) -> std::uint8_t {
        if(!v)
        {
            std::fprintf(stderr, "[expect] %s is X/Z\n", name);
            std::abort();
        }
        return *v;
    };
    auto require_u16 = [&](std::optional<std::uint16_t> v, char const* name) -> std::uint16_t {
        if(!v)
        {
            std::fprintf(stderr, "[expect] %s is X/Z\n", name);
            std::abort();
        }
        return *v;
    };
    auto require_b = [&](std::optional<bool> v, char const* name) -> bool {
        if(!v)
        {
            std::fprintf(stderr, "[expect] %s is X/Z\n", name);
            std::abort();
        }
        return *v;
    };

    // Reset and initial fetch (same sequence as x86_16_multi_module.cc):
    do_step("reset", false, [&] {
        set_rstn(false);
        set_clk(false);
    });
    if(expect)
    {
        auto s = snapshot();
        check_probe_equal(s, "reset");
        // Note: initial states may be X/Z until a triggering edge occurs (rst_n starts low in INPUT).
    }

    do_step("clk=1", true, [&] { set_clk(true); });
    if(expect)
    {
        auto s = snapshot();
        check_probe_equal(s, "clk=1");
        if(s.rst_n && *s.rst_n == false)
        {
            if(require_u8(s.pc, "pc") != 0) { std::abort(); }
        }
    }
    do_step("rst=1", true, [&] { set_rstn(true); });
    if(expect)
    {
        auto s = snapshot();
        check_probe_equal(s, "rst=1");
        // No clock edge here; just ensure we didn't introduce X/Z on the probe nets.
    }

    // Fetch on negedge: IR should latch ROM data for the *current PC*.
    auto pre_fetch0 = snapshot();
    do_step("fetch", false, [&] { set_clk(false); });
    if(expect)
    {
        auto post = snapshot();
        check_probe_equal(post, "fetch0");
        if(pre_fetch0.rst_n && post.rst_n && *pre_fetch0.rst_n && *post.rst_n)
        {
            // PC does not change on negedge.
            auto const pre_pc = require_u8(pre_fetch0.pc, "pre_pc");
            auto const post_pc = require_u8(post.pc, "post_pc");
            if(pre_pc != post_pc)
            {
                std::fprintf(stderr, "[expect] fetch0: pc changed on negedge: pre=%02x post=%02x\n", pre_pc, post_pc);
                std::abort();
            }
            // IR captures ROM data seen just before the negedge.
            auto const pre_rom = require_u16(pre_fetch0.rom_data, "pre_rom_data");
            auto const post_ir = require_u16(post.ir, "post_ir");
            if(pre_rom != post_ir)
            {
                std::fprintf(stderr, "[expect] fetch0: ir != pre_rom_data: ir=%04x pre_rom=%04x\n",
                             static_cast<unsigned>(post_ir),
                             static_cast<unsigned>(pre_rom));
                std::abort();
            }
        }
    }

    bool halted{};
    for(int cycle = 0; cycle < 64; ++cycle)
    {
        // Execute on posedge: PC should capture pc_next (as seen on pc8.d) from the prior fetch phase.
        auto pre_exec = snapshot();
        do_step("exec", true, [&] { set_clk(true); });
        if(expect)
        {
            auto post = snapshot();
            check_probe_equal(post, "exec");
        if(pre_exec.rst_n && post.rst_n && *pre_exec.rst_n && *post.rst_n)
        {
            auto const pc_we = require_b(pre_exec.pc_we, "pc_we");
            auto const pre_pc = require_u8(pre_exec.pc, "pre_pc");
            auto const pre_next = require_u8(pre_exec.pc_next, "pre_pc_next");
            auto const post_pc = require_u8(post.pc, "post_pc");

            if(pc_we)
            {
                if(post_pc != pre_next)
                {
                    std::fprintf(stderr,
                                 "[expect] exec: pc != pre_pc_next: pre_pc=%02x pre_next=%02x post_pc=%02x\n",
                                 pre_pc,
                                 pre_next,
                                 post_pc);
                    std::abort();
                }
            }
            else
            {
                if(post_pc != pre_pc)
                {
                    std::fprintf(stderr,
                                 "[expect] exec: pc changed while pc_we=0: pre_pc=%02x post_pc=%02x\n",
                                 pre_pc,
                                 post_pc);
                    std::abort();
                }
            }
        }
    }
        halted = read_bool(n_halt);
        if(halted) { break; }

        // Fetch next instruction on negedge: IR should capture current ROM data (for current PC).
        auto pre_fetch = snapshot();
        do_step("fetch", false, [&] { set_clk(false); });
        if(expect)
        {
            auto post = snapshot();
            check_probe_equal(post, "fetch");
        if(pre_fetch.rst_n && post.rst_n && *pre_fetch.rst_n && *post.rst_n)
        {
            auto const pre_pc = require_u8(pre_fetch.pc, "pre_pc");
            auto const post_pc = require_u8(post.pc, "post_pc");
            if(pre_pc != post_pc)
            {
                std::fprintf(stderr,
                             "[expect] fetch: pc changed on negedge: pre=%02x post=%02x\n",
                             pre_pc,
                             post_pc);
                std::abort();
            }

            auto const pre_rom = require_u16(pre_fetch.rom_data, "pre_rom_data");
            auto const post_ir = require_u16(post.ir, "post_ir");
            if(pre_rom != post_ir)
            {
                std::fprintf(stderr,
                             "[expect] fetch: ir != pre_rom_data: pc=%02x ir=%04x pre_rom=%04x\n",
                             static_cast<unsigned>(pre_pc),
                             static_cast<unsigned>(post_ir),
                             static_cast<unsigned>(pre_rom));
                std::abort();
            }
        }
    }
    }

    assert(halted && "CPU did not reach HLT within cycle budget");

    // Program should leave R0 == 0, and keep R1 == 7 (the MOVI 0x55 is skipped).
    auto const r0 = read_u16(dbg_r0);
    auto const r1 = read_u16(dbg_r1);
    if(r0 != 0x0000 || r1 != 0x0007)
    {
        std::fprintf(stderr, "dbg_r0=0x%04x dbg_r1=0x%04x\n", static_cast<unsigned>(r0), static_cast<unsigned>(r1));
    }
    assert(r0 == 0x0000);
    assert(r1 == 0x0007);
    (void)dbg_r2;
    (void)dbg_r3;

    return 0;
}
