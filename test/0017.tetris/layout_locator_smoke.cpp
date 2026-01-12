#include <phy_engine/phy_lab_wrapper/layout_locator.h>

#include <cassert>

int main()
{
    using namespace phy_engine::phy_lab_wrapper;
    using namespace phy_engine::phy_lab_wrapper::layout;

    auto ex = experiment::create(experiment_type::circuit);

    // Rectangle corners in native coords.
    (void)ex.add_circuit_element("Student Source", {-1.0, 1.0, 0.0});        // left_top
    (void)ex.add_circuit_element("Incandescent Lamp", {-1.0, -1.0, 0.0});    // left_bottom
    (void)ex.add_circuit_element("Simple Switch", {1.0, 1.0, 0.0});          // right_top
    (void)ex.add_circuit_element("Battery Source", {1.0, -1.0, 0.0});        // right_bottom

    auto loc = corner_locator::from_experiment(ex,
                                               corner_markers{
                                                   .left_top_model_id = "Student Source",
                                                   .left_bottom_model_id = "Incandescent Lamp",
                                                   .right_top_model_id = "Simple Switch",
                                                   .right_bottom_model_id = "Battery Source",
                                               });

    // Center should map to (0,0,0).
    {
        auto p = loc.map_uv(0.5, 0.5);
        assert(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
    }

    // Grid corners.
    {
        auto p00 = loc.map_grid(0, 0, 8, 8, true);   // top-left
        auto p70 = loc.map_grid(7, 0, 8, 8, true);   // top-right
        auto p07 = loc.map_grid(0, 7, 8, 8, true);   // bottom-left
        auto p77 = loc.map_grid(7, 7, 8, 8, true);   // bottom-right
        assert(p00.x == -1.0 && p00.y == 1.0);
        assert(p70.x == 1.0 && p70.y == 1.0);
        assert(p07.x == -1.0 && p07.y == -1.0);
        assert(p77.x == 1.0 && p77.y == -1.0);
    }

    return 0;
}

