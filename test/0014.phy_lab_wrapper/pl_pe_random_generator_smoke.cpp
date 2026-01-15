#include <phy_engine/phy_lab_wrapper/pe_sim.h>

// Bring in the C ABI implementation for this test binary.
#include "../../src/dll_main.cpp"

#include <cassert>

int main()
{
    using namespace phy_engine::phy_lab_wrapper;

    auto ex = experiment::create(experiment_type::circuit);
    ex.entitle("pl_pe_random_generator_smoke");

    auto clk = ex.add_circuit_element("Logic Input", position{0.0, 0.0, 0.0});
    auto rstn = ex.add_circuit_element("Logic Input", position{0.0, 0.0, 1.0});
    auto rng = ex.add_circuit_element("Random Generator", position{1.0, 0.0, 0.0});
    auto out_lsb = ex.add_circuit_element("Logic Output", position{2.0, 0.0, 0.0});

    ex.get_element(clk).data()["Properties"]["开关"] = 0;
    ex.get_element(rstn).data()["Properties"]["开关"] = 0; // reset asserted (active-low)

    ex.connect(clk, 0, rng, 4);
    ex.connect(rstn, 0, rng, 5);

    // Observe LSB (pin3 in PL -> internal bit0)
    ex.connect(rng, 3, out_lsb, 0);

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

    // While in reset, output must be 0.
    for (int i = 0; i < 2; ++i) { tick(); }
    auto r0 = ex.get_element(out_lsb).data()["Properties"].value("状态", 0.0);
    assert(r0 == 0.0);

    // Release reset and expect the output bit to change at least once.
    ex.get_element(rstn).data()["Properties"]["开关"] = 1;
    tick();
    auto first = ex.get_element(out_lsb).data()["Properties"].value("状态", 0.0);

    bool changed = false;
    for (int i = 0; i < 8; ++i)
    {
        tick();
        auto now = ex.get_element(out_lsb).data()["Properties"].value("状态", 0.0);
        if (now != first)
        {
            changed = true;
            break;
        }
    }

    assert(changed);
}
