#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

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

    static constexpr std::size_t kA = 16;
    static constexpr std::size_t kB = 16;
    static constexpr std::size_t kY = 16;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "fp16_addsub.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    auto cr = ::phy_engine::verilog::digital::compile(src);
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"fp16_addsub_unit");
    if(mod == nullptr) { return 2; }

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
    if(top_inst.mod == nullptr) { return 3; }

    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::array<::phy_engine::model::model_base*, kA> in_a{};
    std::array<::phy_engine::model::model_base*, kB> in_b{};
    ::phy_engine::model::model_base* in_sub{};
    std::array<std::size_t, kY> out_y{};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    out_y.fill(static_cast<std::size_t>(-1));

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

            if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kA) { in_a[*idx] = m; }
            else if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kB) { in_b[*idx] = m; }
            else if(port_name == "sub") { in_sub = m; }
        }
        else if(p.dir == port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 6; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 7; }

            if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kY) { out_y[*idx] = pi; }
        }
        else
        {
            return 8;
        }
    }

    for(auto* m : in_a) { if(m == nullptr) { return 9; } }
    for(auto* m : in_b) { if(m == nullptr) { return 10; } }
    if(in_sub == nullptr) { return 11; }
    for(auto pi : out_y) { if(pi == static_cast<std::size_t>(-1)) { return 12; } }

    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        return 13;
    }

    if(!c.analyze()) { return 14; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };

    auto settle = [&]() noexcept
    {
        c.digital_clk();
        c.digital_clk();
    };

    auto read_y_if_binary = [&]() -> std::optional<std::uint16_t> {
        std::uint16_t y{};
        for(std::size_t i{}; i < kY; ++i)
        {
            auto const s = ports[out_y[i]]->node_information.dn.state;
            if(s != ::phy_engine::model::digital_node_statement_t::false_state &&
               s != ::phy_engine::model::digital_node_statement_t::true_state)
            {
                return std::nullopt;
            }
            if(s == ::phy_engine::model::digital_node_statement_t::true_state) { y |= static_cast<std::uint16_t>(1u << i); }
        }
        return y;
    };

    struct tc
    {
        std::uint16_t a;
        std::uint16_t b;
        bool sub;
        std::uint16_t exp;
    };
    // Same smoke vectors as 0020.
    constexpr tc cases[] = {
        {0x3E00u, 0x4000u, false, 0x4300u},  // 1.5 + 2.0 = 3.5
        {0x4000u, 0x3E00u, true, 0x3800u},   // 2.0 - 1.5 = 0.5
        {0xBC00u, 0x3800u, false, 0xB800u},  // -1.0 + 0.5 = -0.5
    };

    for(auto const& t : cases)
    {
        for(std::size_t i{}; i < kA; ++i) { set_in(in_a[i], ((t.a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kB; ++i) { set_in(in_b[i], ((t.b >> i) & 1u) != 0); }
        set_in(in_sub, t.sub);
        settle();

        auto y = read_y_if_binary();
        if(!y) { return 15; }
        if(*y != t.exp) { return 16; }
    }

    return 0;
}

