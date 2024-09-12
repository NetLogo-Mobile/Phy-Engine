#pragma once
#include <cstdint>
#include <set>
#include "../pin/pin.h"

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

    struct node_t
    {
        node_information_union node_information{};
        ::std::set<::phy_engine::model::pin*> pins{};
        node_type_t node_type{};

        ~node_t() { clear(); }

        void clear_node() noexcept
        {
            for(auto i: pins) { i->nodes = nullptr; }
            pins.clear();
        }

        void clear() noexcept
        {
            clear_node();
            node_information.an.voltage = {};
            node_type = node_type_t::artifical;
        }
    };
}  // namespace phy_engine::model
