#pragma once
#include <set>

#include "../../model/node/node.h"

namespace phy_engine::digital
{
    struct digital_node_update_state
    {
        ::phy_engine::model::node_t* node{};
        ::phy_engine::model::digital_node_statement_t update_statement{};
    };

    struct digital_node_update_table
    {
        ::std::set<::phy_engine::model::node_t*> tables{};
    };

    struct need_operate_analog_node_t
    {
        double voltage{};
        ::phy_engine::model::node_t* need_to_operate_analog_node{};
    };
}  // namespace phy_engine::digital
