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

static void set_u1(::phy_engine::verilog::digital::instance_state& top_inst, std::string_view base, bool v)
{
    for(auto const& p : top_inst.mod->ports)
    {
        std::string pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
        if(pn != base) { continue; }
        if(p.dir != ::phy_engine::verilog::digital::port_dir::input) { continue; }
        top_inst.state.values.index_unchecked(p.signal) =
            v ? ::phy_engine::verilog::digital::logic_t::true_state : ::phy_engine::verilog::digital::logic_t::false_state;
    }
}

int main()
{
    decltype(auto) src = u8R"(
module tmod (
    input  logic a,
    output logic y_init,
    output logic y_latch,
    output logic y_case
);
    initial begin
        y_init = 1'b0;
        y_latch = 1'b0;
        #5 y_init = 1'b1;
    end

    // latch intent: no else branch
    always_latch begin
        if(a) y_latch = 1'b1;
    end

    always_comb begin
        unique case(a)
            1'b0: y_case = 1'b0;
            default: y_case = 1'b1;
        endcase
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

    set_u1(top_inst, "a", false);
    ::phy_engine::verilog::digital::simulate(top_inst, 0);

    if(get_u1(top_inst, "y_init") != 0) { return 4; }
    if(get_u1(top_inst, "y_latch") != 0) { return 5; }
    if(get_u1(top_inst, "y_case") != 0) { return 6; }

    set_u1(top_inst, "a", true);
    ::phy_engine::verilog::digital::simulate(top_inst, 0);
    if(get_u1(top_inst, "y_latch") != 1) { return 7; }
    if(get_u1(top_inst, "y_case") != 1) { return 8; }

    // delayed initial assignment
    ::phy_engine::verilog::digital::simulate(top_inst, 5);
    if(get_u1(top_inst, "y_init") != 1) { return 9; }

    return 0;
}

