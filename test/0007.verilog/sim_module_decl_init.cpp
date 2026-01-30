#include <cstdint>
#include <string>
#include <string_view>

#include <phy_engine/verilog/digital/digital.h>

int main()
{
    using namespace phy_engine::verilog::digital;

    decltype(auto) src = u8R"(
module top(
    input  logic a,
    output logic y_reg,
    output logic [3:0] y_vec,
    output wire  y_wire
);
    logic       r  = 1'b1;
    logic [3:0] rv = 4'ha;
    wire        w  = a;

    always_comb begin
        y_reg = r;
        y_vec = rv;
    end

    assign y_wire = w;
endmodule
)";

    auto cr = compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }

    auto design = build_design(::std::move(cr));
    auto const* top_mod = find_module(design, u8"top");
    if(top_mod == nullptr) { return 2; }

    auto inst = elaborate(design, *top_mod);
    if(inst.mod == nullptr) { return 3; }

    auto set_in = [&](::fast_io::u8string_view nm, bool v) noexcept
    {
        for(auto const& p: inst.mod->ports)
        {
            if(p.dir != port_dir::input) { continue; }
            if(p.name != nm) { continue; }
            inst.state.values.index_unchecked(p.signal) = v ? logic_t::true_state : logic_t::false_state;
        }
    };

    auto read_out = [&](::fast_io::u8string_view nm, logic_t& out) noexcept -> bool
    {
        for(auto const& p: inst.mod->ports)
        {
            if(p.dir != port_dir::output) { continue; }
            if(p.name != nm) { continue; }
            out = inst.state.values.index_unchecked(p.signal);
            return true;
        }
        return false;
    };

    auto read_u4 = [&](::fast_io::u8string_view base, std::uint64_t& out) noexcept -> bool
    {
        out = 0;
        bool any{};
        for(auto const& p: inst.mod->ports)
        {
            if(p.dir != port_dir::output) { continue; }
            std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
            std::string_view b(reinterpret_cast<char const*>(base.data()), base.size());
            if(!pn.starts_with(b)) { continue; }
            auto lb = pn.find('[');
            if(lb == std::string_view::npos) { continue; }
            auto rb = pn.find(']', lb);
            if(rb == std::string_view::npos || rb <= lb + 1) { continue; }

            int idx{};
            auto const idx_sv = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(pn.data() + lb + 1), rb - lb - 1};
            if(!::phy_engine::verilog::digital::details::parse_dec_int(idx_sv, idx)) { return false; }
            if(idx < 0 || idx > 63) { return false; }

            any = true;
            auto const v = inst.state.values.index_unchecked(p.signal);
            if(v == logic_t::true_state) { out |= (1ull << static_cast<std::uint64_t>(idx)); }
            else if(v != logic_t::false_state) { return false; }
        }
        return any;
    };

    // Tick 0 should apply declaration initializers (via implicit `initial`) and run always_comb once.
    set_in(u8"a", false);
    simulate(inst, 0);

    logic_t yreg{};
    logic_t ywire{};
    if(!read_out(u8"y_reg", yreg) || !read_out(u8"y_wire", ywire)) { return 4; }
    if(yreg != logic_t::true_state) { return 5; }
    if(ywire != logic_t::false_state) { return 6; }

    std::uint64_t yv{};
    if(!read_u4(u8"y_vec", yv)) { return 7; }
    if((yv & 0xFULL) != 0xAULL) { return 8; }

    // Wire initializer should behave like continuous assignment.
    set_in(u8"a", true);
    simulate(inst, 1);
    if(!read_out(u8"y_wire", ywire)) { return 9; }
    if(ywire != logic_t::true_state) { return 10; }

    return 0;
}
