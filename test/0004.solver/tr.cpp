#include <complex>
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_driver/timer.h>
#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c.get_analyze_setting()};

    setting.tr.t_step = 0.0001;
    setting.tr.t_stop = 0.001;

    auto& nl{c.get_netlist()};

    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
    auto [C1, C1_pos]{add_model(nl, ::phy_engine::model::inductor{.m_kZimag{0.1}})};
    auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};

    auto& node1{create_node(nl)};
    add_to_node(nl, *R1, 1, node1);
    add_to_node(nl, *C1, 0, node1);
    auto& node2{create_node(nl)};
    add_to_node(nl, *VDC, 0, node2);
    add_to_node(nl, *R1, 0, node2);
    auto& node3{get_ground_node(nl)};  // ground
    add_to_node(nl, *VDC, 1, node3);
    add_to_node(nl, *C1, 1, node3);

    double start{};
    ::fast_io::timer t{u8"time"};
    for(::std::size_t i = 0; i < 1000; i++)
    {
        if(!c.analyze()) [[unlikely]]
        {
            ::fast_io::io::perr("analyze error\n");
            return -1;
        }
    }
}
