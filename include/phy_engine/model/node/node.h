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
        digital = 0,
        artifical = 1,
    };

    union node_information_union
    {
        digital_node_t dn;
        artifical_node_t an;
    };

    struct node_t
    {
        node_information_union node_information{};
        ::std::set<::phy_engine::model::pin*> pins{};
        ::std::size_t num_of_artifical_node{};
        node_type_t node_type{};

        node_t() noexcept = default;

        // copy and disconnect form the model 
        node_t(node_t const& others) noexcept : node_information{others.node_information}, node_type{others.node_type} {}

        // copy and disconnect form the model 
        node_t& operator= (node_t const& others) noexcept
        {
            node_information = others.node_information;
            pins.clear();
            node_type = others.node_type;
        }

        ~node_t() { clear(); }

        node_t& copy_with_model(node_t const& others) noexcept
        {
            node_information = others.node_information;
            pins = others.pins;
            num_of_artifical_node = others.num_of_artifical_node;
            node_type = others.node_type;

            return *this;
        }

        void clear_node() noexcept
        {
            for(auto i: pins) { i->nodes = nullptr; }
            pins.clear();
            num_of_artifical_node = 0;
        }

        void clear() noexcept
        {
            clear_node();
            node_information.dn.state = {};
            node_type = node_type_t::digital;
        }
    };
}  // namespace phy_engine::model
