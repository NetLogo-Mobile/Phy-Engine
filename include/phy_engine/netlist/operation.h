#pragma once

#include "netlist.h"
#include "concept.h"

namespace phy_engine::netlist
{
    template <bool check = false>
    inline constexpr ::std::size_t get_num_of_model(netlist const& nl) noexcept
    {
        return nl.netlist_memory.size();
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
        requires (::phy_engine::model::model<mod> && ::phy_engine::model::defines::can_iterate_dc<mod>)
    inline constexpr ::phy_engine::model::model_base* add_model(netlist& nl, mod&& m) noexcept
    {
        using rcvmod_type = ::std::remove_cvref_t<mod>;
        if(nl.netlist_memory.empty()) [[unlikely]]
        {
            auto& nlb{nl.netlist_memory.emplace_back()};
            new(nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
            nl.m_numTermls += rcvmod_type::pins.size;
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
                nl.m_numTermls += rcvmod_type::pins.size;
                return {
                    new_nlb.curr++,
                    {0, nl.netlist_memory.size() - 1}
                };
            }
            else
            {
                new(nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
                nl.m_numTermls += rcvmod_type::pins.size;
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
        if(vec_pos >= nl.netlist_memory.size()) [[unlikely]] { return false; }
        auto& nlb{nl.netlist_memory.index_unchecked(vec_pos)};

        auto i{nlb.begin + chunk_pos};

        if(i >= nlb.curr) [[unlikely]] { return false; }
        else if(i == nlb.curr - 1)
        {
            if(i->type == ::phy_engine::model::model_type::null) { --nlb.num_of_null_model; }
            nl.m_numTermls -= i->ptr->get_pins().size;
            i->~model_base();
            --nlb.curr;
        }
        else
        {
            if(i->type != ::phy_engine::model::model_type::null)
            {
                ++nlb.num_of_null_model;
                nl.m_numTermls -= i->ptr->get_pins().size;
                i->clear();
            }
        }
        // to do (also delete wire)
        return true;
    }

    inline constexpr bool delete_model(netlist& nl, model_pos pos) noexcept { return delete_model(nl, pos.vec_pos, pos.chunk_pos); }

    inline constexpr ::phy_engine::model::model_base* get_model(netlist const& nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept
    {
        if(vec_pos >= nl.netlist_memory.size()) [[unlikely]] { return nullptr; }
        auto& nlb{nl.netlist_memory.index_unchecked(vec_pos)};
        if(chunk_pos >= nlb.size()) [[unlikely]] { return nullptr; }
        return nlb.begin + chunk_pos;
    }

    inline constexpr ::phy_engine::model::model_base* get_model(netlist const& nl, model_pos pos) noexcept { return get_model(nl, pos.vec_pos, pos.chunk_pos); }

    inline constexpr bool add_wire(netlist& nl, model_pos mp1, ::std::size_t n1, model_pos mp2, ::std::size_t n2) noexcept
    {
        ::phy_engine::model::model_base* model1{get_model(nl, mp1)};
        if(model1 == nullptr) [[unlikely]] { return false; }

        auto pw1{model1->ptr->get_pins()};
        if(n1 >= pw1.size) [[unlikely]] { return false; }

        ::phy_engine::model::model_base* model2{get_model(nl, mp2)};
        if(model2 == nullptr) [[unlikely]] { return false; }

        auto pw2{model2->ptr->get_pins()};
        if(n2 >= pw2.size) [[unlikely]] { return false; }
        // to do
        return true;
    }

    inline constexpr bool delete_wire(netlist& nl, model_pos mp1, ::std::size_t n1, model_pos mp2, ::std::size_t n2) noexcept { return {}; }

    inline constexpr bool add_netlist(netlist& nl, netlist const& nl_add) noexcept { return {}; }

    inline constexpr netlist get_netlist(netlist& nl, ::std::size_t* pos_view, ::std::size_t size) noexcept { return {}; }

    inline constexpr void optimize_memory(netlist& nl) noexcept {}

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
                        nl.m_numNodes += b->nodes.size();
                        nl.m_numBranches += b->branchs.size();
                        break;
                    }
                }
            }
        }
        return true;
    }

}  // namespace phy_engine::netlist
