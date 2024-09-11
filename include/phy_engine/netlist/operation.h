#pragma once
#include "netlist.h"
#include "concept.h"

namespace phy_engine::netlist
{
    template <bool check = false>
    inline constexpr ::std::size_t get_num_of_model(netlist const& nl) noexcept
    {
        if constexpr(check)
        {
            ::std::size_t res{};
            for(auto const& i: nl.netlist_memory)
            {
                for(::phy_engine::model::model_base* b{i.begin}; b != i.curr; ++b)
                {
                    if(b->type != ::phy_engine::model::model_type::null) [[likely]] { ++res; }
                }
            }
            return res;
        }
        else
        {
            ::std::size_t res{};
            for(auto const& i: nl.netlist_memory) { res += i.get_num_of_model(); }
            return res;
        }
    }

    struct model_pos
    {
        ::std::size_t vec_pos{};
        ::std::size_t chunk_pos{};
    };

    struct add_model_retstr
    {
        ::phy_engine::model::model_base* mod{};
        model_pos mod_pos{};
    };

    template <typename mod>
        requires (::phy_engine::model::model<mod> && ::phy_engine::model::defines::can_iterate_dc<mod> &&
                  ::phy_engine::model::defines::can_generate_pin_view<mod>)
    inline constexpr add_model_retstr add_model(netlist& nl, mod&& m) noexcept
    {
        using rcvmod_type = ::std::remove_cvref_t<mod>;
        auto const pin_view{generate_pin_view_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m))};
        if(nl.netlist_memory.empty()) [[unlikely]]
        {
            auto& nlb{nl.netlist_memory.emplace_back()};
            new(nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
            nl.m_numTermls += pin_view.size;
            return {
                nlb.curr++,
                {0, 0}
            };
        }
        else
        {
            auto& nlb{nl.netlist_memory.back_unchecked()};
            if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
            {
                auto& new_nlb{nl.netlist_memory.emplace_back()};
                new(new_nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
                nl.m_numTermls += pin_view.size;
                return {
                    new_nlb.curr++,
                    {0, nl.netlist_memory.size() - 1}
                };
            }
            else
            {
                new(nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
                nl.m_numTermls += pin_view.size;
                add_model_retstr return_val{
                    nlb.curr,
                    {static_cast<::std::size_t>(nlb.curr - nlb.begin), nl.netlist_memory.size() - 1}
                };
                ++nlb.curr;
                return return_val;
            }
        }
    }

#if 0
template <typename mod>
	requires(::phy_engine::model::model<mod> && (::phy_engine::model::can_generate_netlist<mod> || ::phy_engine::model::can_generate_netlist_const_lvalue<mod>))
inline constexpr model_pos add_model(netlist &nl, mod &&m) noexcept {
	// Managed by circuit
	return {};
}
#endif

    inline constexpr bool delete_model(netlist& nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept
    {
        if(chunk_pos >= nl.netlist_memory.size()) [[unlikely]] { return false; }
        auto& nlb{nl.netlist_memory.index_unchecked(chunk_pos)};

        auto i{nlb.begin + vec_pos};

        if(i >= nlb.curr) [[unlikely]] { return false; }
        else if(i == nlb.curr - 1)
        {
            auto const i_type{i->type};

            if(i_type == ::phy_engine::model::model_type::null) [[unlikely]]
            {
                --nlb.num_of_null_model;
                i->~model_base();
                --nlb.curr;
                return false;
            }

            auto i_pin_view{i->ptr->generate_pin_view()};

            for(auto curr{i_pin_view.pins}; curr != i_pin_view.pins + i_pin_view.size; ++curr)
            {
                auto node{curr->nodes};
                if(node) [[likely]] { node->pins.erase(curr); }
            }

            nl.m_numTermls -= i_pin_view.size;
            i->~model_base();
            --nlb.curr;
        }
        else
        {
            if(i->type != ::phy_engine::model::model_type::null)
            {
                auto i_pin_view{i->ptr->generate_pin_view()};

                for(auto curr{i_pin_view.pins}; curr != i_pin_view.pins + i_pin_view.size; ++curr)
                {
                    auto node{curr->nodes};
                    if(node) [[likely]] { node->pins.erase(curr); }
                }

                ++nlb.num_of_null_model;
                nl.m_numTermls -= i_pin_view.size;
                i->clear();
            }
            else [[unlikely]] { return false; }
        }

        return true;
    }

    inline constexpr bool delete_model(netlist& nl, model_pos pos) noexcept { return delete_model(nl, pos.vec_pos, pos.chunk_pos); }

    inline constexpr ::phy_engine::model::model_base* get_model(netlist const& nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept
    {
        if(chunk_pos >= nl.netlist_memory.size()) [[unlikely]] { return nullptr; }
        auto& nlb{nl.netlist_memory.index_unchecked(chunk_pos)};
        if(vec_pos >= nlb.size()) [[unlikely]] { return nullptr; }
        return nlb.begin + vec_pos;
    }

    inline constexpr ::phy_engine::model::model_base* get_model(netlist const& nl, model_pos pos) noexcept { return get_model(nl, pos.vec_pos, pos.chunk_pos); }

    inline ::phy_engine::model::node_t& create_node(netlist& nl) noexcept { return nl.nodes.emplace_back(); }

    inline constexpr bool add_to_node(netlist& nl, model_pos mp1, ::std::size_t n1, ::phy_engine::model::node_t& node) noexcept
    {
        ::phy_engine::model::model_base* model1{get_model(nl, mp1)};
        if(model1 == nullptr) [[unlikely]] { return false; }

        auto pw1{model1->ptr->generate_pin_view()};
        if(n1 >= pw1.size) [[unlikely]] { return false; }

        auto& p1{pw1.pins[n1]};

        p1.nodes = __builtin_addressof(node);
        node.pins.insert(__builtin_addressof(p1));

        return true;
    }

    inline void delete_node(netlist& nl, ::phy_engine::model::node_t& node) noexcept
    {
        for(auto i: node.pins) { i->nodes = nullptr; }
        node.node_type = ::phy_engine::model::node_type_t::artifical;
        node.node_information.an.voltage = 0.0;
        node.pins.clear();
    }

    inline void optimize_memory(netlist& nl) noexcept {}

#if 0
    inline constexpr bool prepare(netlist& nl) noexcept
    {
        for(auto& i: nl.netlist_memory)
        {
            for(::phy_engine::model::model_base* b{i.begin}; b != i.curr; b++)
            {
                switch(b->type)
                {
                    case ::phy_engine::model::model_type::null:
                    {
                        break;
                    }
                    case ::phy_engine::model::model_type::invalid:
                    {
                        return false;
                    }
                    default:
                    {
                        b->ptr->init_model();
                        break;
                    }
                }
            }
        }

        nl.m_numNodes = 0;
        nl.m_numBranches = 0;

        for(auto& i: nl.netlist_memory)
        {
            for(::phy_engine::model::model_base* b{i.begin}; b != i.curr; b++)
            {
                switch(b->type)
                {
                    case ::phy_engine::model::model_type::null:
                    {
                        break;
                    }
                    case ::phy_engine::model::model_type::invalid:
                    {
                        ::fast_io::unreachable();
                        break;
                    }
                    default:
                    {
                        // nl.m_numNodes += b->nodes.size();
                        // nl.m_numBranches += b->branchs.size();
                        break;
                    }
                }
            }
        }
        return true;
    }
#endif

}  // namespace phy_engine::netlist
