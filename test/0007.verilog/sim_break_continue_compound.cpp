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
module tmod (
    input  logic [3:0] a,
    output logic [3:0] y_break,
    output logic [3:0] y_cont,
    output logic [3:0] y_comp,
    output logic [3:0] y_inc
);
    always_comb begin
        int i;

        y_break = '0;
        for (i = 0; i < 4; i++) begin
            if (i == 2) break;
            y_break[i] = 1'b1;
        end

        y_cont = '0;
        for (i = 0; i < 4; i++) begin
            if (i == 1) continue;
            y_cont[i] = 1'b1;
        end

        begin
            logic [3:0] tmp;
            tmp = 4'h1;
            tmp += 4'h2;
            tmp <<= 1;
            y_comp = tmp;
        end

        begin
            logic [3:0] incv;
            incv = '0;
            incv++;
            y_inc = incv;
        end
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

    auto const y_break = get_u4(top_inst, "y_break");
    auto const y_cont = get_u4(top_inst, "y_cont");
    auto const y_comp = get_u4(top_inst, "y_comp");
    auto const y_inc = get_u4(top_inst, "y_inc");

    if(y_break != 0b0011) { return 4; }
    if(y_cont != 0b1101) { return 5; }
    if(y_comp != 0b0110) { return 6; }
    if(y_inc != 0b0001) { return 7; }
    return 0;
}
