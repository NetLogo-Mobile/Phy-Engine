#include <phy_engine/phy_lab_wrapper/pe_sim.h>

// Bring in the C ABI implementation for this test binary.
// In real usage you should link against the built `phyengine` shared/static library instead.
#include "../../src/dll_main.cpp"

#include <cassert>

int main()
{
    using namespace phy_engine::phy_lab_wrapper;

    auto ex = experiment::create(experiment_type::circuit);
    ex.entitle("pl_pe_sim_smoke");

    auto a = ex.add_circuit_element("Logic Input", position{-1.0, 0.0, 0.0});
    auto b = ex.add_circuit_element("Logic Input", position{1.0, 0.0, 0.0});
    auto g = ex.add_circuit_element("And Gate", position{0.0, 0.0, 0.0});
    auto y = ex.add_circuit_element("Logic Output", position{0.0, 0.0, 1.0});

    ex.get_element(a).data()["Properties"]["开关"] = 0;
    ex.get_element(b).data()["Properties"]["开关"] = 0;

    // And Gate pins: i_up=0, i_low=1, o=2
    // Logic Input pin: o=0
    // Logic Output pin: i=0
    ex.connect(a, 0, g, 0);
    ex.connect(b, 0, g, 1);
    ex.connect(g, 2, y, 0);

    phy_engine::phy_lab_wrapper::pe::circuit c = phy_engine::phy_lab_wrapper::pe::circuit::build_from(ex);
    c.set_analyze_type(PHY_ENGINE_ANALYZE_TR);
    c.set_tr(1e-9, 1e-9);
    c.analyze();

    auto step = [&](int av, int bv) -> int {
        ex.get_element(a).data()["Properties"]["开关"] = av;
        ex.get_element(b).data()["Properties"]["开关"] = bv;
        c.sync_inputs_from_pl(ex);
        c.digital_clk();
        auto s = c.sample_now();
        c.write_back_to_pl(ex, s);
        auto const& out_props = ex.get_element(y).data()["Properties"];
        return out_props.value("状态", 0.0) != 0.0 ? 1 : 0;
    };

    assert(step(0, 0) == 0);
    assert(step(0, 1) == 0);
    assert(step(1, 0) == 0);
    assert(step(1, 1) == 1);
}

