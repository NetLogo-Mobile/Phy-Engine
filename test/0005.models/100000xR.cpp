#include <complex>
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_driver/timer.h>
#include <phy_engine/phy_engine.h>

int main()
{
#ifdef EIGEN_USE_MKL
    MKL_Set_Num_Threads(32);
    ::fast_io::io::perr("MKL\n");
#endif
#if 1
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);

        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
        auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
        auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};

        auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
        auto& node1{nl.ground_node};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *R2, 0, node1);
        ::phy_engine::model::model_base* model{R2};

        for(::std::size_t i = 0; i < 10; i++)
        {
            auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
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

        {
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
            ::fast_io::timer t{u8"10xR, time"};
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
        }
    }
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);

        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
        auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
        auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};

        auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
        auto& node1{nl.ground_node};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *R2, 0, node1);
        ::phy_engine::model::model_base* model{R2};

        for(::std::size_t i = 0; i < 100; i++)
        {
            auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
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

        {
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
            ::fast_io::timer t{u8"100xR, time"};
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
        }
    }
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);

        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
        auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
        auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};

        auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
        auto& node1{nl.ground_node};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *R2, 0, node1);
        ::phy_engine::model::model_base* model{R2};

        for(::std::size_t i = 0; i < 1000; i++)
        {
            auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
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

        {
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
            ::fast_io::timer t{u8"1000xR, time"};
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
        }
    }
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);

        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
        auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
        auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};

        auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
        auto& node1{nl.ground_node};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *R2, 0, node1);
        ::phy_engine::model::model_base* model{R2};

        for(::std::size_t i = 0; i < 10000; i++)
        {
            auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
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

        {
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
            ::fast_io::timer t{u8"10000xR, time"};
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
        }
    }
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);

        auto& nl{c.get_netlist()};

        auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
        auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
        auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};

        auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
        auto& node1{nl.ground_node};
        add_to_node(nl, *R1, 1, node1);
        add_to_node(nl, *R2, 0, node1);
        ::phy_engine::model::model_base* model{R2};

        for(::std::size_t i = 0; i < 100000; i++)
        {
            auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
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

        {
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
            ::fast_io::timer t{u8"100000xR, time"};
            if(!c.analyze())
            {
                ::fast_io::io::perr("analyze error\n");
                return -1;
            }
        }
    }
#else
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);

    auto& nl{c.get_netlist()};

    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
    auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
    auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};

    auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
    auto& node1{nl.ground_node};
    add_to_node(nl, *R1, 1, node1);
    add_to_node(nl, *R2, 0, node1);
    ::phy_engine::model::model_base* model{R2};

    for(::std::size_t i = 0; i < 10000; i++)
    {
        auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
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

    {
        if(!c.analyze())
        {
            ::fast_io::io::perr("analyze error\n");
            return -1;
        }
        ::fast_io::timer t{u8"100000xR, time"};
        if(!c.analyze())
        {
            ::fast_io::io::perr("analyze error\n");
            return -1;
        }
    }

#endif
}
