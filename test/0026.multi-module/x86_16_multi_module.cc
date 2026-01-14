#include <phy_engine/phy_engine.h>

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
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
}  // namespace

int main()
{
    auto const dir = std::filesystem::path(__FILE__).parent_path();

    auto const pc8_s = read_file_text(dir / "pc8.v");
    auto const ir16_s = read_file_text(dir / "ir16.v");
    auto const rom_s = read_file_text(dir / "rom256x16.v");
    auto const dec_s = read_file_text(dir / "decode16.v");
    auto const ctl_s = read_file_text(dir / "control16.v");
    auto const imm_s = read_file_text(dir / "imm_ext8_to_16.v");
    auto const rf_s = read_file_text(dir / "regfile4x16.v");
    auto const mux_s = read_file_text(dir / "mux16.v");
    auto const alu_s = read_file_text(dir / "alu16.v");
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

    // Expect a very small PE netlist (each Verilog module is one element).
    if(::phy_engine::netlist::get_num_of_model(nl) > 32) { return 3; }

    // Shared wires (vector nodes are stored MSB -> LSB to match port expansion order).
    auto& nclk = create_node(nl);
    auto& nrstn = create_node(nl);

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

    // control16(opcode, reg_dst, reg_src, imm8, pc, flag_z, flag_c, flag_s,
    //           pc_next, pc_we, reg_we, rf_waddr, rf_raddr_a, rf_raddr_b, alu_b_sel,
    //           flags_we_z, flags_we_c, flags_we_s, alu_op, halt)
    connect_bus(nl, *m_ctl, 0, opcode);
    connect_bus(nl, *m_ctl, 4, reg_dst);
    connect_bus(nl, *m_ctl, 6, reg_src);
    connect_bus(nl, *m_ctl, 8, imm8);
    connect_bus(nl, *m_ctl, 16, pc);
    add_to_node(nl, *m_ctl, 24, n_flag_z);
    add_to_node(nl, *m_ctl, 25, n_flag_c);
    add_to_node(nl, *m_ctl, 26, n_flag_s);
    connect_bus(nl, *m_ctl, 27, pc_next);
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

    // regfile4x16(clk,rst_n,we,waddr[1:0],wdata[15:0],raddr_a[1:0],raddr_b[1:0],
    //            rdata_a[15:0],rdata_b[15:0],dbg_r0,dbg_r1,dbg_r2,dbg_r3)
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

    if(!c.analyze()) { return 4; }

    auto set_clk = [&](bool v) {
        in_clk->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                           : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto set_rstn = [&](bool v) {
        in_rstn->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    // VERILOG_MODULE runs in the `before_all_clk` phase. When composing multiple such modules,
    // a small fixed-point iteration (multiple `digital_clk()` calls with inputs held constant)
    // is needed to propagate combinational signals across module boundaries.
    auto settle = [&](int iters) {
        for(int i = 0; i < iters; ++i) { c.digital_clk(); }
    };

    bool const trace = (std::getenv("PHY_ENGINE_TRACE_0026_MULTI") != nullptr);

    // Reset and initial fetch:
    // - Hold reset low.
    // - Deassert reset while clk is high.
    // - Perform a negedge so the IR latches the first instruction at PC=0 before the first execute posedge.
    set_rstn(false);
    set_clk(false);
    settle(4);
    set_clk(true);
    settle(4);
    set_rstn(true);
    settle(4);
    set_clk(false);
    settle(4);

    bool halted{};
    for(int cycle = 0; cycle < 32; ++cycle)
    {
        // Pre-settle combinational paths at clk=0 (compute next_pc, alu_y, reg_we, etc).
        set_clk(false);
        settle(4);
        if(trace)
        {
            std::fprintf(stderr,
                         "[pre ] cyc=%d pc=%02x ir=%04x rom=%04x we=%d alu=%04x z=%d c=%d s=%d r0=%04x r1=%04x\n",
                         cycle,
                         static_cast<unsigned>(read_u16(pc)),
                         static_cast<unsigned>(read_u16(ir)),
                         static_cast<unsigned>(read_u16(rom_data)),
                         n_reg_we.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         static_cast<unsigned>(read_u16(alu_y)),
                         n_flag_z.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         n_flag_c.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         n_flag_s.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         static_cast<unsigned>(read_u16(dbg_r0)),
                         static_cast<unsigned>(read_u16(dbg_r1)));
        }

        set_clk(true);
        settle(4);
        halted = (n_halt.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state);
        if(trace)
        {
            std::fprintf(stderr,
                         "[pos ] cyc=%d pc=%02x ir=%04x rom=%04x we=%d alu=%04x z=%d c=%d s=%d r0=%04x r1=%04x halt=%d\n",
                         cycle,
                         static_cast<unsigned>(read_u16(pc)),
                         static_cast<unsigned>(read_u16(ir)),
                         static_cast<unsigned>(read_u16(rom_data)),
                         n_reg_we.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         static_cast<unsigned>(read_u16(alu_y)),
                         n_flag_z.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         n_flag_c.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         n_flag_s.node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state,
                         static_cast<unsigned>(read_u16(dbg_r0)),
                         static_cast<unsigned>(read_u16(dbg_r1)),
                         halted);
        }
        if(halted) { break; }

        set_clk(false);
        settle(4);
    }

    assert(halted && "CPU did not reach HLT within cycle budget");

    // Program should leave R0 == 0, and keep R1 == 7 (the MOVI 0x55 is skipped).
    auto const r0 = read_u16(dbg_r0);
    auto const r1 = read_u16(dbg_r1);
    if(r0 != 0x0000 || r1 != 0x0007)
    {
        std::fprintf(stderr, "dbg_r0=0x%04x dbg_r1=0x%04x\n", r0, r1);
    }
    assert(r0 == 0x0000);
    assert(r1 == 0x0007);
    (void)dbg_r2;
    (void)dbg_r3;

    return 0;
}
