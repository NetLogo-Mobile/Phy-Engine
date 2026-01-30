#include <cstdint>
#include <string>
#include <string_view>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

namespace
{
std::string to_string(char8_t const* p, std::size_t n) { return std::string(reinterpret_cast<char const*>(p), n); }

bool starts_with(std::string const& s, std::string_view prefix)
{
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

std::size_t bit_index_from_port_name(std::string const& pn)
{
    auto const lb = pn.find('[');
    if(lb == std::string::npos) { return 0; }
    return static_cast<std::size_t>(std::stoi(pn.substr(lb + 1, pn.size() - lb - 2)));
}

void set_u1(::phy_engine::verilog::digital::instance_state& top, std::string_view base, bool v)
{
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::input) { continue; }
        std::size_t const idx = bit_index_from_port_name(pn);
        if(idx != 0) { continue; }
        top.state.values.index_unchecked(p.signal) =
            v ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state;
    }
}

int get_u1(::phy_engine::verilog::digital::instance_state& top, std::string_view base)
{
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }
        std::size_t const idx = bit_index_from_port_name(pn);
        if(idx != 0) { continue; }
        auto const v = top.state.values.index_unchecked(p.signal);
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { return 1; }
        if(v == ::phy_engine::verilog::digital::logic_t::false_state) { return 0; }
        return -1;
    }
    return -1;
}
}  // namespace

int main()
{
    // Regression: unsized decimal literals should behave as 32-bit signed integers in Verilog/SV.
    decltype(auto) src = u8R"(
module t(
    input  logic d,
    output logic y
);
    assign y = (-1 < 0);
endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"t");
    if(top_mod == nullptr) { return 2; }
    auto top = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top.mod == nullptr) { return 3; }

    set_u1(top, "d", false);
    ::phy_engine::verilog::digital::simulate(top, 0);

    int const y = get_u1(top, "y");
    if(y < 0) { return 5; }
    return (y == 1) ? 0 : 4;
}
