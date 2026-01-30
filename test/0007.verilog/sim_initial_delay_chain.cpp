#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>

static std::uint8_t get_u1(::phy_engine::verilog::digital::instance_state const& top_inst, std::string_view base)
{
    for(auto const& p : top_inst.mod->ports)
    {
        std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(pn != base) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::output) { continue; }
        auto v = top_inst.state.values.index_unchecked(p.signal);
        if(v != ::phy_engine::verilog::digital::logic_t::true_state && v != ::phy_engine::verilog::digital::logic_t::false_state) { return 0xff; }
        return v == ::phy_engine::verilog::digital::logic_t::true_state ? 1 : 0;
    }
    return 0xff;
}

int main()
{
    decltype(auto) src = u8R"(
module tmod (
    output logic y
);
    initial begin
        y = 1'b0;
        #5 y = 1'b1;
        #5 y = 1'b0;
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
    if(get_u1(top_inst, "y") != 0) { return 4; }

    ::phy_engine::verilog::digital::simulate(top_inst, 5);
    if(get_u1(top_inst, "y") != 1) { return 5; }

    ::phy_engine::verilog::digital::simulate(top_inst, 10);
    if(get_u1(top_inst, "y") != 0) { return 6; }

    return 0;
}

