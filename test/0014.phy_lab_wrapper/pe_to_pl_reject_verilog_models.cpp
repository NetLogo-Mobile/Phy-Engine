#include <phy_engine/model/models/digital/verilog_module.h>
#include <phy_engine/netlist/operation.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>
#include <string>

int main()
{
    using namespace phy_engine;

    netlist::netlist nl{};
    auto [m, pos] = netlist::add_model(nl, model::VERILOG_MODULE{});
    (void)pos;
    assert(m != nullptr);

    try
    {
        (void)phy_lab_wrapper::pe_to_pl::convert(nl);
        assert(false && "expected exception");
    }
    catch(std::exception const& ex)
    {
        std::string msg = ex.what();
        assert(msg.find("VERILOG_MODULE") != std::string::npos);
    }

    return 0;
}

