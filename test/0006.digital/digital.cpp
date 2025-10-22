#include <phy_engine/phy_engine.h>

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::ACOP);

    auto& nl{c.get_netlist()};

    auto [o1, o1_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};

    auto [or1, or1_pos]{add_model(nl, ::phy_engine::model::OR{})};

    auto [i1, i1_pos]{add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::H})};
    auto [i2, i2_pos]{add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::L})};

    auto& node3{create_node(nl)};
    add_to_node(nl, *o1, 0, node3);
    add_to_node(nl, *or1, 2, node3);

    auto& node2{create_node(nl)};
    add_to_node(nl, *i2, 0, node2);
    add_to_node(nl, *or1, 1, node2);

    auto& node1{create_node(nl)};
    add_to_node(nl, *i1, 0, node1);
    add_to_node(nl, *or1, 0, node1);

    for(::std::size_t i = 0; i < 3; i++)
    {
        c.analyze();
        c.digital_clk();
        ::fast_io::println("OR: ",
                           i,
                           "\n" "I1: ",
                           i1->ptr->get_attribute(0).digital,
                           ", I2: ",
                           i2->ptr->get_attribute(0).digital,
                           ", O:",
                           o1->ptr->get_attribute(0).digital);
    }
}
