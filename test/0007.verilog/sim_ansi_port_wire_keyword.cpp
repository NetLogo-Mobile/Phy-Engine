#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

int main()
{
    decltype(auto) src = u8R"(
module mul8x8 (
    input  wire [7:0]  a,
    input  wire [7:0]  b,
    output wire [15:0] p
);
    assign p = a * b;
endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"mul8x8");
    if(top_mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    // Basic smoke: drive a=3, b=7 => p=21.
    auto set_u8 = [&](std::string_view base, std::uint8_t v) {
        for(auto const& p : top_inst.mod->ports)
        {
            std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
            if(!pn.starts_with(base)) { continue; }
            if(p.dir != ::phy_engine::verilog::digital::port_dir::input) { continue; }
            auto lb = pn.find('[');
            std::size_t idx{};
            if(lb != std::string_view::npos)
            {
                idx = static_cast<std::size_t>(std::stoi(std::string(pn.substr(lb + 1, pn.size() - lb - 2))));
            }
            bool bit = ((v >> idx) & 1u) != 0;
            top_inst.state.values.index_unchecked(p.signal) =
                bit ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state;
        }
    };

    set_u8("a", 3);
    set_u8("b", 7);
    ::phy_engine::verilog::digital::simulate(top_inst, 0);

    std::uint16_t out{};
    for(auto const& p : top_inst.mod->ports)
    {
        std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(!pn.starts_with("p")) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }
        auto lb = pn.find('[');
        std::size_t idx{};
        if(lb != std::string_view::npos)
        {
            idx = static_cast<std::size_t>(std::stoi(std::string(pn.substr(lb + 1, pn.size() - lb - 2))));
        }
        auto v = top_inst.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return 4; }
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { out |= static_cast<std::uint16_t>(1u << idx); }
    }

    return (out == 21) ? 0 : 5;
}

