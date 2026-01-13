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
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
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
    using namespace phy_engine::phy_lab_wrapper;

    static constexpr std::size_t kW = 8;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "adder8.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"adder8_top");
    if(top_mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::array<::phy_engine::model::model_base*, kW> in_a{};
    std::array<::phy_engine::model::model_base*, kW> in_b{};
    ::phy_engine::model::model_base* in_c{};
    std::array<std::size_t, kW> out_s{};
    std::size_t out_cout{static_cast<std::size_t>(-1)};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    in_c = nullptr;
    out_s.fill(static_cast<std::size_t>(-1));

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

            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kW) { in_a[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kW) { in_b[*idx] = m; }
            else if(port_name == "c") { in_c = m; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }

            if(auto idx = parse_bit_index(port_name, "s"); idx && *idx < kW) { out_s[*idx] = pi; }
            else if(port_name == "cout") { out_cout = pi; }
        }
        else
        {
            return 8;
        }
    }

    for(auto* m : in_a) { if(m == nullptr) { return 9; } }
    for(auto* m : in_b) { if(m == nullptr) { return 10; } }
    if(in_c == nullptr) { return 11; }
    for(auto pi : out_s) { if(pi == static_cast<std::size_t>(-1)) { return 12; } }
    if(out_cout == static_cast<std::size_t>(-1)) { return 13; }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
        .assume_binary_inputs = true,
        .optimize_wires = true,
        .optimize_adders = true,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt)) { return 14; }
    if(!c.analyze()) { return 15; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) noexcept {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto set_inputs = [&](std::uint8_t a, std::uint8_t b, bool cin) noexcept {
        for(std::size_t i{}; i < kW; ++i) { set_in(in_a[i], ((a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kW; ++i) { set_in(in_b[i], ((b >> i) & 1u) != 0); }
        set_in(in_c, cin);
    };
    auto read_bit = [&](std::size_t pi) noexcept -> std::optional<bool> {
        auto const s = ports[pi]->node_information.dn.state;
        if(s == ::phy_engine::model::digital_node_statement_t::false_state) { return false; }
        if(s == ::phy_engine::model::digital_node_statement_t::true_state) { return true; }
        return std::nullopt;
    };
    auto read_u8 = [&]() noexcept -> std::optional<std::uint8_t> {
        std::uint8_t v{};
        for(std::size_t i{}; i < kW; ++i)
        {
            auto b = read_bit(out_s[i]);
            if(!b) { return std::nullopt; }
            if(*b) { v |= static_cast<std::uint8_t>(1u << i); }
        }
        return v;
    };

    // Quick sanity.
    for(std::uint8_t a : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{2}, std::uint8_t{7}, std::uint8_t{0x55}, std::uint8_t{0xFF}})
    {
        for(std::uint8_t b : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{3}, std::uint8_t{9}, std::uint8_t{0x0F}, std::uint8_t{0xFF}})
        {
            for(bool cin : {false, true})
            {
                set_inputs(a, b, cin);
                c.digital_clk();
                auto s = read_u8();
                auto co = read_bit(out_cout);
                if(!s || !co) { return 16; }

                std::uint16_t const ref = static_cast<std::uint16_t>(a) + static_cast<std::uint16_t>(b) + static_cast<std::uint16_t>(cin ? 1u : 0u);
                if(*s != static_cast<std::uint8_t>(ref & 0xFFu)) { return 17; }
                if(*co != ((ref >> 8) & 1u)) { return 18; }
            }
        }
    }

    // Export PE->PL (.sav) placement: row1=a, row2=b, row3=c, row4=outputs.
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        auto const row_y_a = 1.0;
        auto const row_y_b = 0.5;
        auto const row_y_c = 0.0;
        auto const row_y_out = -0.5;

        auto const x_for_bit = [](std::size_t idx, std::size_t n, double xmin, double xmax) noexcept -> double {
            if(n <= 1) { return (xmin + xmax) * 0.5; }
            double const t = static_cast<double>(idx) / static_cast<double>(n - 1);
            return xmin + (xmax - xmin) * t;
        };

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "a"); idx && *idx < kW)
                {
                    return position{x_for_bit(*idx, kW, -1.0, 1.0), row_y_a, 0.0};
                }
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "b"); idx && *idx < kW)
                {
                    return position{x_for_bit(*idx, kW, -1.0, 1.0), row_y_b, 0.0};
                }
                if(ctx.pe_instance_name == "c") { return position{0.0, row_y_c, 0.0}; }
            }

            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                if(auto idx = parse_bit_index(ctx.pe_instance_name, "s"); idx && *idx < kW)
                {
                    // Leave space for cout at far right.
                    return position{x_for_bit(*idx, kW, -1.0, 0.75), row_y_out, 0.0};
                }
                if(ctx.pe_instance_name == "cout") { return position{1.0, row_y_out, 0.0}; }
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
        assert(!r.ex.wires().empty());

        auto const out_path = std::filesystem::path("adder8_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 19; }
        if(std::filesystem::file_size(out_path) < 128) { return 20; }
    }

    return 0;
}
