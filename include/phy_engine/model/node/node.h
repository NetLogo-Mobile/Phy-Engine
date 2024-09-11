#pragma once
#include <cstdint>
#include <set>

namespace phy_engine::model
{
    struct artifical_node_t
    {
        double voltage{};
    };

    enum digital_node_statement_t : ::std::uint_fast8_t
    {
        false_state = 0,
        true_state = 1,
        high_impedence_state = 2,
        indeterminate_state = 3
    };

    struct digital_node_t
    {
        digital_node_statement_t state{};
    };

    enum node_type_t : ::std::uint_fast8_t
    {
        artifical = 0,
        digital = 1,
    };

    union node_information_union
    {
        artifical_node_t an;
        digital_node_t dn;
    };

    struct pin;

    struct node_t
    {
        node_information_union node_information{};
        ::std::set<pin*> pins{};
        node_type_t node_type{};
    };
}  // namespace phy_engine::model
