#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

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
    using namespace phy_engine::phy_lab_wrapper;

    static constexpr std::size_t kW = 8;
    static constexpr std::size_t kH = 8;
    static_assert(kW * kH == 64);

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "tetris.v";
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
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        return 1;
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"tetris_top");
    if(mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    // Port nodes in module port order.
    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    auto find_port = [&](char const* name) -> std::size_t {
        for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto const& pn = top_inst.mod->ports.index_unchecked(i).name;
            std::string s(reinterpret_cast<char const*>(pn.data()), pn.size());
            if(s == name) { return i; }
        }
        return static_cast<std::size_t>(-1);
    };

    std::unordered_map<std::string, ::phy_engine::model::model_base*> input_by_name{};
    input_by_name.reserve(top_inst.mod->ports.size());

    std::vector<std::size_t> pix_port_indices{};
    pix_port_indices.reserve(64);

    // External IO models.
    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == ::phy_engine::verilog::digital::port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 4; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 5; }
            if(port_name == "clk" || port_name == "rst_n")
            {
                auto [pm, ppos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
                (void)ppos;
                if(pm == nullptr || pm->ptr == nullptr) { return 6; }
                pm->name.clear();
                pm->name.append(u8"probe_");
                pm->name.append(p.name);
                if(!::phy_engine::netlist::add_to_node(nl, *pm, 0, *ports[pi])) { return 7; }
            }

            input_by_name.emplace(std::move(port_name), m);
            continue;
        }

            

        if(p.dir == ::phy_engine::verilog::digital::port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }

            if(port_name.starts_with("pix["))
            {
                pix_port_indices.push_back(pi);
            }
            continue;
        }

        // No inout at top-level for this test.
        return 8;
    }

    if(pix_port_indices.size() != 64) { return 9; }
    if(!input_by_name.contains("clk") || !input_by_name.contains("rst_n")) { return 10; }

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

    // Export PE->PL (.sav) with deterministic IO placement (no auto placement).
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            // Table boundary is a square: x,z in [-1, 1]. Keep y=0.
            // - Logic outputs (pix[0..63]) occupy the right half.
            // - Logic inputs (buttons) occupy the left half.

            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(ctx.pe_instance_name == "probe_clk") return position{-0.7, 1.0, 0.0};
                if(ctx.pe_instance_name == "probe_rst_n") return position{-0.7, 0.85, 0.0};

                auto idx = parse_bit_index(ctx.pe_instance_name, "pix");
                if(!idx || *idx >= 64) { return std::nullopt; }

                auto const col = static_cast<double>(*idx % kW);
                auto const row = static_cast<double>(*idx / kW);

                // Map to x in [0, 1], y in [1, -1] (z is height)
                double const x = (kW <= 1) ? 1.0 : (col / static_cast<double>(kW - 1));
                double const y = (kH <= 1) ? 0.0 : (1.0 - 2.0 * (row / static_cast<double>(kH - 1)));

                return position{x, y, 0.0};
            }

            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                // Clock/reset inputs (top-left)
                if(ctx.pe_instance_name == "clk") return position{-1.0, 1.0, 0.0};
                if(ctx.pe_instance_name == "rst_n") return position{-1.0, 0.85, 0.0};

                // Buttons on the left half (x=-1), y from top to bottom (z is height).
                if(ctx.pe_instance_name == "btn_left") return position{-1.0, 0.6, 0.0};
                if(ctx.pe_instance_name == "btn_right") return position{-1.0, 0.2, 0.0};
                if(ctx.pe_instance_name == "btn_rot") return position{-1.0, -0.2, 0.0};
                if(ctx.pe_instance_name == "btn_drop") return position{-1.0, -0.6, 0.0};
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);

        // Must be a real netlist (nodes are linked), even if element placement is fixed.
        assert(!r.ex.wires().empty());

        auto const out_path = std::filesystem::path("tetris_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 11; }
        if(std::filesystem::file_size(out_path) < 128) { return 12; }

        // Validate deterministic IO placement.
        struct key
        {
            std::string model_id;
            std::string name;
        };
        struct key_hash
        {
            std::size_t operator()(key const& k) const noexcept
            {
                return std::hash<std::string>{}(k.model_id) ^ (std::hash<std::string>{}(k.name) << 1);
            }
        };
        struct key_eq
        {
            bool operator()(key const& a, key const& b) const noexcept { return a.model_id == b.model_id && a.name == b.name; }
        };

        std::unordered_map<key, position, key_hash, key_eq> pos_by_kind_and_name{};
        pos_by_kind_and_name.reserve(r.ex.elements().size());
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            std::string name;
            if(auto it = e.data().find("Label"); it != e.data().end() && it->is_string())
            {
                name = it->get<std::string>();
            }
            else
            {
                name.clear();
            }
            if(!mid.empty() && !name.empty())
            {
                pos_by_kind_and_name.emplace(key{mid, name}, e.element_position());
            }
        }

        for(std::size_t idx{}; idx < 64; ++idx)
        {
            auto const col = static_cast<double>(idx % kW);
            auto const row = static_cast<double>(idx / kW);
            double const expected_x = (kW <= 1) ? 1.0 : (col / static_cast<double>(kW - 1));
            double const expected_y = (kH <= 1) ? 0.0 : (1.0 - 2.0 * (row / static_cast<double>(kH - 1)));
            auto const expected = position{expected_x, expected_y, 0.0};
            auto const name = "pix[" + std::to_string(idx) + "]";

            auto it = pos_by_kind_and_name.find(key{std::string(pl_model_id::logic_output), name});
            if(it == pos_by_kind_and_name.end()) { return 13; }

            auto const p = it->second;
            assert(p.x == expected.x && p.y == expected.y && p.z == expected.z);
        }

        struct io_expect
        {
            char const* name;
            position pos;
        };
        constexpr io_expect ios[] = {
            {"btn_left", {-1.0, 0.6, 0.0}},
            {"btn_right", {-1.0, 0.2, 0.0}},
            {"btn_rot", {-1.0, -0.2, 0.0}},
            {"btn_drop", {-1.0, -0.6, 0.0}},
        };
        for(auto const& io : ios)
        {
            auto it = pos_by_kind_and_name.find(key{std::string(pl_model_id::logic_input), io.name});
            if(it == pos_by_kind_and_name.end()) { return 14; }
            auto const p = it->second;
            assert(p.x == io.pos.x && p.y == io.pos.y && p.z == io.pos.z);
        }
    }

    // Run PE simulation.
    if(!c.analyze()) { return 15; }

    auto* in_clk = input_by_name.at("clk");
    auto* in_rstn = input_by_name.at("rst_n");
    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    set_in(in_rstn, false);
    set_in(in_clk, false);
    c.digital_clk();
    set_in(in_clk, true);
    c.digital_clk();
    set_in(in_clk, false);
    c.digital_clk();
    set_in(in_rstn, true);

    // Press buttons for a couple of cycles to ensure activity.
    auto* in_left = input_by_name.at("btn_left");
    auto* in_rot = input_by_name.at("btn_rot");
    auto* in_drop = input_by_name.at("btn_drop");

    set_in(in_left, true);
    set_in(in_rot, true);
    for(int i = 0; i < 4; ++i)
    {
        set_in(in_clk, true);
        c.digital_clk();
        set_in(in_clk, false);
        c.digital_clk();
    }
    set_in(in_left, false);
    set_in(in_rot, false);
    set_in(in_drop, true);
    for(int i = 0; i < 2; ++i)
    {
        set_in(in_clk, true);
        c.digital_clk();
        set_in(in_clk, false);
        c.digital_clk();
    }
    set_in(in_drop, false);

    // At least one pixel should be asserted (this design always asserts 2 pixels).
    std::size_t on{};
    for(auto pi : pix_port_indices)
    {
        if(ports[pi]->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state)
        {
            ++on;
        }
    }
    assert(on >= 1);

    return 0;
}

