#pragma once

#include <absl/container/btree_set.h>
#include <fast_io/fast_io_dsal/vector.h>

#include <utility>

#include "../../netlist/netlist.h"

namespace phy_engine::floating_subnet
{
    inline void ensure_pin_model_ptr(::phy_engine::netlist::netlist& nl) noexcept
    {
        for(auto& i: nl.models)
        {
            for(auto c{i.begin}; c != i.curr; ++c)
            {
                if(c->type != ::phy_engine::model::model_type::normal || c->ptr == nullptr) [[unlikely]] { continue; }
                auto const pin_view{c->ptr->generate_pin_view()};
                for(auto p{pin_view.pins}; p != pin_view.pins + pin_view.size; ++p) { p->model = c; }
            }
        }
    }

    // Return all floating subnets (each subnet is a list of nodes).
    // Empty means no floating subnet.
    [[nodiscard]] inline ::fast_io::vector<::fast_io::vector<::phy_engine::model::node_t*>>
        detect(::phy_engine::netlist::netlist& nl) noexcept
    {
        ensure_pin_model_ptr(nl);

        ::fast_io::vector<::phy_engine::model::node_t*> nodes{};
        for(auto& i: nl.nodes)
        {
            for(auto c{i.begin}; c != i.curr; ++c)
            {
                if(!c->pins.empty()) { nodes.push_back(c); }
            }
        }

        if(nodes.empty()) { return ::fast_io::vector<::fast_io::vector<::phy_engine::model::node_t*>>{}; }

        ::absl::btree_set<::phy_engine::model::node_t*> processed{};
        ::fast_io::vector<::phy_engine::model::node_t*> stack{};

        // 1) Mark all nodes reachable from ground.
        processed.insert(__builtin_addressof(nl.ground_node));
        stack.push_back(__builtin_addressof(nl.ground_node));

        while(!stack.empty())
        {
            auto* const node{stack.back_unchecked()};
            stack.pop_back();

            for(auto const* const pin: node->pins)
            {
                auto* const model{pin->model};
                if(model == nullptr || model->type != ::phy_engine::model::model_type::normal || model->ptr == nullptr) [[unlikely]] { continue; }

                auto const pin_view{model->ptr->generate_pin_view()};
                for(auto p{pin_view.pins}; p != pin_view.pins + pin_view.size; ++p)
                {
                    auto* const other{p->nodes};
                    if(other == nullptr) [[unlikely]] { continue; }
                    if(processed.insert(other).second) { stack.push_back(other); }
                }
            }
        }

        // 2) Any remaining connected component is a floating subnet.
        ::fast_io::vector<::fast_io::vector<::phy_engine::model::node_t*>> floating_subnets{};
        for(auto* const start: nodes)
        {
            if(processed.find(start) != processed.end()) { continue; }

            ::fast_io::vector<::phy_engine::model::node_t*> subnet{};
            processed.insert(start);
            stack.clear();
            stack.push_back(start);

            while(!stack.empty())
            {
                auto* const node{stack.back_unchecked()};
                stack.pop_back();
                subnet.push_back(node);

                for(auto const* const pin: node->pins)
                {
                    auto* const model{pin->model};
                    if(model == nullptr || model->type != ::phy_engine::model::model_type::normal || model->ptr == nullptr) [[unlikely]] { continue; }

                    auto const pin_view{model->ptr->generate_pin_view()};
                    for(auto p{pin_view.pins}; p != pin_view.pins + pin_view.size; ++p)
                    {
                        auto* const other{p->nodes};
                        if(other == nullptr) [[unlikely]] { continue; }
                        if(other == __builtin_addressof(nl.ground_node)) { continue; }
                        if(processed.insert(other).second) { stack.push_back(other); }
                    }
                }
            }

            floating_subnets.push_back(::std::move(subnet));
        }

        return floating_subnets;
    }
}  // namespace phy_engine::floating_subnet
