#include <phy_engine/phy_lab_wrapper/physicslab.h>

#include <cassert>

int main()
{
    using namespace phy_engine::phy_lab_wrapper;

    auto ex = experiment::create(experiment_type::circuit);

    auto js = ex.to_plsav_json();
    assert(js.contains("Experiment"));
    assert(js["Experiment"].contains("CameraSave"));

    auto camera_str = js["Experiment"]["CameraSave"].get<std::string>();
    auto camera = json::parse(camera_str, nullptr, true, true);
    assert(camera.contains("Mode"));
    assert(camera.contains("Distance"));
    assert(camera.contains("VisionCenter"));
    assert(camera.contains("TargetRotation"));

    auto id = ex.add_circuit_element("Logic Input", {0.0, 0.0, 0.0});
    auto const& el = ex.get_element(id).data();
    assert(el.contains("Label"));
    assert(el.contains("Properties"));
    assert(el["Properties"].contains("高电平"));
    assert(el["Properties"].contains("低电平"));
    assert(el["Properties"].contains("开关"));

    return 0;
}
