#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

int main()
{
    decltype(auto) src = u8R"(
module tmod (
    input  logic [3:0] a,
    output logic [3:0] y
);
    function automatic logic [3:0] f(input logic [3:0] in);
        begin
            return in ^ 4'ha;
        end
    endfunction

    always_comb begin
        y = f(a);
    end
endmodule
)";

    auto cr = ::phy_engine::verilog::digital::compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    if(!cr.errors.empty() || cr.modules.empty()) { return 1; }
    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"tmod");
    if(top_mod == nullptr) { return 2; }
    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 3; }

    auto set_u4 = [&](std::string_view base, std::uint8_t v) {
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

    std::uint8_t const a{0b1100};
    set_u4("a", a);
    ::phy_engine::verilog::digital::simulate(top_inst, 0);

    std::uint8_t out{};
    for(auto const& p : top_inst.mod->ports)
    {
        std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(!pn.starts_with("y")) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }
        auto lb = pn.find('[');
        std::size_t idx{};
        if(lb != std::string_view::npos)
        {
            idx = static_cast<std::size_t>(std::stoi(std::string(pn.substr(lb + 1, pn.size() - lb - 2))));
        }
        auto v = top_inst.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return 4; }
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { out |= static_cast<std::uint8_t>(1u << idx); }
    }

    return (out == static_cast<std::uint8_t>(a ^ 0b1010)) ? 0 : 5;
}

