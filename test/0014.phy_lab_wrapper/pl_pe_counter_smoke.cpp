#include <phy_engine/phy_lab_wrapper/pe_sim.h>

// Bring in the C ABI implementation for this test binary.
#include "../../src/dll_main.cpp"

#include <cassert>

int main()
{
    using namespace phy_engine::phy_lab_wrapper;

    auto ex = experiment::create(experiment_type::circuit);
    ex.entitle("pl_pe_counter_smoke");

    auto clk = ex.add_circuit_element("Logic Input", position{0.0, 0.0, 0.0});
    auto ctr = ex.add_circuit_element("Counter", position{1.0, 0.0, 0.0});

    auto o0 = ex.add_circuit_element("Logic Output", position{2.0, 0.0, 0.0}); // MSB
    auto o3 = ex.add_circuit_element("Logic Output", position{2.0, 0.0, 1.0}); // LSB

    // clk.o -> ctr.i_up (pin4)
    ex.get_element(clk).data()["Properties"]["开关"] = 0;
    ex.connect(clk, 0, ctr, 4);
    // observe MSB and LSB
    ex.connect(ctr, 0, o0, 0);
    ex.connect(ctr, 3, o3, 0);

    phy_engine::phy_lab_wrapper::pe::circuit c = phy_engine::phy_lab_wrapper::pe::circuit::build_from(ex);
    c.set_analyze_type(PHY_ENGINE_ANALYZE_TR);
    c.set_tr(1e-9, 1e-9);
    c.analyze();

    auto tick = [&] {
        ex.get_element(clk).data()["Properties"]["开关"] = 0;
        c.sync_inputs_from_pl(ex);
        c.digital_clk();

        ex.get_element(clk).data()["Properties"]["开关"] = 1;
        c.sync_inputs_from_pl(ex);
        c.digital_clk();

        auto s = c.sample_now();
        c.write_back_to_pl(ex, s);
    };

    // After 1 tick, LSB should be 1 (since it starts at 0 and toggles).
    tick();
    assert(ex.get_element(o3).data()["Properties"].value("状态", 0.0) == 1.0);

    // After 2 ticks, LSB should be 0.
    tick();
    assert(ex.get_element(o3).data()["Properties"].value("状态", 0.0) == 0.0);

    // After 8 ticks total, MSB should have toggled once in a 4-bit ripple counter.
    for (int i = 0; i < 6; ++i)
    {
        tick();
    }
    assert(ex.get_element(o0).data()["Properties"].value("状态", 0.0) == 1.0);
}
