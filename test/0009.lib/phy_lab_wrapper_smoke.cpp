#include <phy_engine/phy_lab_wrapper/physicslab.h>

#include <cassert>
#include <iostream>
#include <string>

int main()
{
    using namespace phy_engine::phy_lab_wrapper;

    int step = 0;
    try
    {
        step = 1;
        auto ex = experiment::create(experiment_type::circuit);
        step = 2;
        ex.entitle("smoke");
        step = 3;
        ex.set_element_xyz(true, position{1.0, 2.0, 3.0});

        step = 4;
        auto a_id = ex.add_circuit_element("Simple Switch", position{0.0, 0.0, 0.0}, std::nullopt);
        step = 5;
        auto b_id = ex.add_circuit_element("Simple Switch", position{1.0, 0.0, 0.0}, std::nullopt);

        step = 6;
        ex.get_element(a_id).data()["Properties"]["开关"] = 1;
        step = 7;
        ex.get_element(b_id).data()["Properties"]["开关"] = 0;

        step = 8;
        ex.connect(a_id, 0, b_id, 1, wire_color::blue);

        step = 9;
        auto root = ex.to_plsav_json();
        step = 10;
        assert(root["Type"] == 0);
        step = 11;
        assert(root["Experiment"]["StatusSave"].is_string());
        step = 12;
        assert(root["Experiment"]["CameraSave"].is_string());

        step = 13;
        auto status = nlohmann::json::parse(root["Experiment"]["StatusSave"].get<std::string>());
        step = 14;
        assert(status["Elements"].size() == 2);
        step = 15;
        assert(status["Wires"].size() == 1);
    }
    catch (std::exception const& e)
    {
        std::cerr << "phy_lab_wrapper_smoke exception at step " << step << ": " << e.what() << "\n";
        return 1;
    }
}
