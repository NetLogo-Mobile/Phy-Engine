#include <complex>
#include <random>
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_driver/timer.h>
#include <phy_engine/phy_engine.h>

int main() {
    ::fast_io::ibuf_white_hole_engine eng;
    ::std::uniform_real_distribution<double> ud(1e-5, 1e5);

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl{c.get_netlist()};

    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = ud(eng)})};
    auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = ud(eng)})};
    auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = ud(eng)})};

    auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
    auto& node1{nl.ground_node};
    add_to_node(nl, *R1, 1, node1);
    add_to_node(nl, *R2, 0, node1);
    ::phy_engine::model::model_base* model{R2};

    for(::std::size_t i = 0; i < 100'000; i++)
    {
        auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = ud(eng)})};
        auto& node1{create_node(nl)};

        add_to_node(nl, *R3, 0, node1);
        add_to_node(nl, *model, 1, node1);
        model = R3;
    }

    auto& node2{create_node(nl)};
    add_to_node(nl, *VDC, 0, node2);
    add_to_node(nl, *R1, 0, node2);

    auto& node4{create_node(nl)};

    add_to_node(nl, *VDC, 1, node4);
    add_to_node(nl, *model, 1, node4);

    c.prepare();
    auto const node_size{c.node_counter};
    ::std::uniform_int_distribution<::std::size_t> us(0, node_size - 1);

    {
        for(::std::size_t i = 0; i < 10000 - 1000; i++)
        {
            auto n1 = c.size_t_to_node_p.index_unchecked(us(eng));
            auto n2 = c.size_t_to_node_p.index_unchecked(us(eng));

            if(n1->num_of_analog_node && n2->num_of_analog_node) [[likely]] { merge_node(nl, *n1, *n2); }
        }

        ::fast_io::io::perr("start analyzing...\n");

        {
            ::fast_io::timer t{u8"100'000xR + 10000n, time"};
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return 0;
            }
        }
    }
}