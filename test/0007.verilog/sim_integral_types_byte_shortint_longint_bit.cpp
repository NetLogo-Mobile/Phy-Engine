#include <cstdint>
#include <string>
#include <string_view>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

namespace
{
std::string to_string(char8_t const* p, std::size_t n)
{
    return std::string(reinterpret_cast<char const*>(p), n);
}

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

void set_u64(::phy_engine::verilog::digital::instance_state& top, std::string_view base, std::uint64_t v)
{
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::input) { continue; }

        std::size_t const idx = bit_index_from_port_name(pn);
        bool const bit = ((v >> idx) & 1ull) != 0;
        top.state.values.index_unchecked(p.signal) =
            bit ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state;
    }
}

std::uint64_t get_u64(::phy_engine::verilog::digital::instance_state& top, std::string_view base)
{
    std::uint64_t out{};
    for(auto const& p : top.mod->ports)
    {
        std::string const pn = to_string(p.name.data(), p.name.size());
        if(!starts_with(pn, base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }

        std::size_t const idx = bit_index_from_port_name(pn);
        auto const v = top.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return UINT64_MAX; }
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { out |= (1ull << idx); }
    }
    return out;
}
}  // namespace

int main()
{
    decltype(auto) src = u8R"(
module t (
    input  byte            a_s,
    input  byte unsigned   a_u,
    input  shortint        s_s,
    input  shortint unsigned s_u,
    input  longint         l_s,
    input  longint unsigned l_u,
    input  bit             d,
    output logic [15:0]    y_as,
    output logic [15:0]    y_au,
    output logic [31:0]    y_ss,
    output logic [31:0]    y_su,
    output logic [63:0]    y_ls,
    output logic [63:0]    y_lu,
    output bit             y_d
);
    // Module-scope integral declarations (subset: lowered to packed vectors)
    byte unsigned bu;
    shortint unsigned su2;
    longint unsigned lu2;

    always_comb begin
        // Procedural decl+init (subset): `type name = expr;` lowered to blocking assignment.
        byte unsigned tmp8 = a_u;
        shortint unsigned tmp16 = s_u;
        longint unsigned tmp64 = l_u;

        bu  = tmp8;
        su2 = tmp16;
        lu2 = tmp64;

        y_as = a_s;   // signed extend: 8 -> 16
        y_ss = s_s;   // signed extend: 16 -> 32
        y_ls = l_s;   // 64-bit

        y_au = bu;    // zero extend: 8 -> 16
        y_su = su2;   // zero extend: 16 -> 32
        y_lu = lu2;   // 64-bit

        y_d = d;
    end
endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"t");
    if(top_mod == nullptr) { return 2; }
    auto top = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top.mod == nullptr) { return 3; }

    set_u64(top, "a_s", 0xFFu);
    set_u64(top, "a_u", 0xFFu);
    set_u64(top, "s_s", 0x8001u);
    set_u64(top, "s_u", 0x8001u);
    set_u64(top, "l_s", 0x0123456789ABCDEFull);
    set_u64(top, "l_u", 0xFEDCBA9876543210ull);
    set_u64(top, "d", 1u);

    ::phy_engine::verilog::digital::simulate(top, 0);

    std::uint64_t const y_as = get_u64(top, "y_as");
    std::uint64_t const y_au = get_u64(top, "y_au");
    std::uint64_t const y_ss = get_u64(top, "y_ss");
    std::uint64_t const y_su = get_u64(top, "y_su");
    std::uint64_t const y_ls = get_u64(top, "y_ls");
    std::uint64_t const y_lu = get_u64(top, "y_lu");
    std::uint64_t const y_d = get_u64(top, "y_d");

    if(y_as == UINT64_MAX || y_au == UINT64_MAX || y_ss == UINT64_MAX || y_su == UINT64_MAX || y_ls == UINT64_MAX || y_lu == UINT64_MAX || y_d == UINT64_MAX)
    {
        return 4;
    }

    if(y_as != 0xFFFFu) { return 5; }
    if(y_au != 0x00FFu) { return 6; }
    if(y_ss != 0xFFFF8001u) { return 7; }
    if(y_su != 0x00008001u) { return 8; }
    if(y_ls != 0x0123456789ABCDEFull) { return 9; }
    if(y_lu != 0xFEDCBA9876543210ull) { return 10; }
    if((y_d & 1ull) != 1ull) { return 11; }

    return 0;
}
