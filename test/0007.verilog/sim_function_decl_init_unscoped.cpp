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

void set_u4(::phy_engine::verilog::digital::instance_state& top, std::string_view base, std::uint8_t v)
{
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::input) { continue; }
        std::size_t const idx = bit_index_from_port_name(pn);
        bool const bit = ((v >> idx) & 1u) != 0u;
        top.state.values.index_unchecked(p.signal) =
            bit ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state;
    }
}

std::uint8_t get_u4(::phy_engine::verilog::digital::instance_state& top, std::string_view base)
{
    std::uint8_t out{};
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }
        std::size_t const idx = bit_index_from_port_name(pn);
        auto const v = top.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return 0xFFu; }
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { out |= static_cast<std::uint8_t>(1u << idx); }
    }
    return out;
}
}  // namespace

int main()
{
    // Regression: decl initializer inside an inlined function body.
    // The function body is parsed via the "unscoped" procedural parser path (scope == nullptr) during inlining.
    decltype(auto) src = u8R"(
module t(
    input  logic [3:0] a,
    output logic [3:0] y
);
    function automatic logic [3:0] f(input logic [3:0] x);
        logic [3:0] t0 = x; // decl init must execute
        begin
            f = t0;
        end
    endfunction

    always_comb begin
        y = f(a);
    end
endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty())
    {
        auto const msg = ::phy_engine::verilog::digital::format_compile_errors(cr, ::fast_io::u8string_view{src, sizeof(src) - 1});
        ::fast_io::io::perr(std::string(reinterpret_cast<char const*>(msg.data()), msg.size()));
        return 1;
    }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"t");
    if(top_mod == nullptr) { return 2; }
    auto top = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top.mod == nullptr) { return 3; }

    set_u4(top, "a", 0b1010);
    ::phy_engine::verilog::digital::simulate(top, 0);
    std::uint8_t const y = get_u4(top, "y");
    if(y == 0xFFu) { return 4; }
    return (y == 0b1010) ? 0 : 5;
}
