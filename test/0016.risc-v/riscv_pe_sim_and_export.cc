#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>
#include <cstddef>
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
}  // namespace

int main()
{
    // Load Verilog source next to this test file.
    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "risc-v.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    // PE circuit container.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;

    auto& nl = c.get_netlist();

    // Compile + elaborate.
    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        // Surface the first error (best-effort).
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        return 1;
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"riscv_top");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    // Port nodes in module port order: clk, rst_n, done.
    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }
    if(ports.size() != 3) { return 4; }

    // External IO models.
    auto [in_clk, p0] =
        ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [in_rstn, p1] =
        ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    auto [out_done, p2] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)p0;
    (void)p1;
    (void)p2;
    if(in_clk == nullptr || in_rstn == nullptr || out_done == nullptr) { return 5; }

    if(!::phy_engine::netlist::add_to_node(nl, *in_clk, 0, *ports[0])) { return 6; }
    if(!::phy_engine::netlist::add_to_node(nl, *in_rstn, 0, *ports[1])) { return 7; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_done, 0, *ports[2])) { return 8; }

    // Synthesize to PE netlist (digital primitives).
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    // Export PE->PL (.sav) (layout only; wires intentionally omitted for now).
    {
        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl);
        auto const out_path = std::filesystem::path("riscv_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 9; }
        if(std::filesystem::file_size(out_path) < 128) { return 10; }
    }

    // Run PE simulation.
    if(!c.analyze()) { return 11; }

    auto set_clk = [&](bool v) {
        (void)in_clk->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                                : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto set_rstn = [&](bool v) {
        (void)in_rstn->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                                 : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    // Reset sequence: hold rst_n low for a couple of edges.
    set_rstn(false);
    set_clk(false);
    c.digital_clk();
    set_clk(true);
    c.digital_clk();
    set_clk(false);
    c.digital_clk();

    set_rstn(true);

    // Run a few cycles; `done` should become 1 after the program reaches the "pass" instruction.
    bool done{};
    for(int cycle = 0; cycle < 32; ++cycle)
    {
        set_clk(true);
        c.digital_clk();  // posedge
        done = (ports[2]->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state);
        if(done) { break; }

        set_clk(false);
        c.digital_clk();  // negedge/hold
    }

    assert(done && "riscv program did not set done within cycle budget");
    return 0;
}

