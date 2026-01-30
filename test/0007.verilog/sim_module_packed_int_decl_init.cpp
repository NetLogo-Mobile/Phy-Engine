#include <cstdint>
#include <string>

#include <phy_engine/verilog/digital/digital.h>

int main()
{
    using namespace phy_engine::verilog::digital;

    decltype(auto) src = u8R"(
module top(output logic y, output logic [7:0] yb);
  int unsigned i = 32'd1;
  byte b = 8'hA5;

  always_comb begin
    y  = i[0];
    yb = b;
  end
endmodule
)";

    auto cr = compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }

    auto design = build_design(::std::move(cr));
    auto const* top_mod = find_module(design, u8"top");
    if(top_mod == nullptr) { return 2; }

    auto inst = elaborate(design, *top_mod);
    if(inst.mod == nullptr) { return 3; }

    auto read_out_scalar = [&](::fast_io::u8string_view nm, logic_t& out) noexcept -> bool
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

    auto read_u8 = [&](::fast_io::u8string_view base, std::uint64_t& out) noexcept -> bool
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

    // Tick 0: initializer should apply via implicit `initial`, and always_comb should run once.
    simulate(inst, 0);

    logic_t y{};
    if(!read_out_scalar(u8"y", y)) { return 4; }
    if(y != logic_t::true_state) { return 5; }

    std::uint64_t yb{};
    if(!read_u8(u8"yb", yb)) { return 6; }
    if((yb & 0xFFu) != 0xA5u) { return 7; }

    return 0;
}

