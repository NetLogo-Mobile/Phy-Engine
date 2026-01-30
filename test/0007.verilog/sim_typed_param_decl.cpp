#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

static std::uint8_t get_u4(::phy_engine::verilog::digital::instance_state const& top_inst, std::string_view base)
{
    std::uint8_t out{};
    for(auto const& p : top_inst.mod->ports)
    {
        std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(!pn.starts_with(base)) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }
        auto lb = pn.find('[');
        std::size_t idx{};
        if(lb != std::string_view::npos)
        {
            idx = static_cast<std::size_t>(std::stoi(std::string(pn.substr(lb + 1, pn.size() - lb - 2))));
        }
        auto v = top_inst.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return 0xff; }
        if(v == ::phy_engine::verilog::digital::logic_t::true_state) { out |= static_cast<std::uint8_t>(1u << idx); }
    }
    return out;
}

int main()
{
    decltype(auto) src = u8R"(
module tmod #(
    parameter int unsigned W = 4,
    localparam int unsigned X = 3,
    parameter logic [W-1:0] MASK = 4'ha
)(
    output logic [3:0] y
);
    always_comb begin
        y = MASK + X;
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

    ::phy_engine::verilog::digital::simulate(top_inst, 0);
    if(get_u4(top_inst, "y") != 0b1101) { return 4; }  // 0xA + 3 = 0xD
    return 0;
}

