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
            for(auto const& i: nl.models)
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
            for(auto const& i: nl.models) { res += i.get_num_of_model(); }
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
        requires (::phy_engine::model::model<mod> && ::phy_engine::model::defines::can_generate_pin_view<mod> &&
                  (::phy_engine::model::defines::can_iterate_mna<mod> || mod::device_type == ::phy_engine::model::model_device_type::digital))
    inline constexpr add_model_retstr add_model(netlist& nl, mod&& m) noexcept
    {
        using rcvmod_type = ::std::remove_cvref_t<mod>;
        auto const pin_view{generate_pin_view_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m))};
        if(nl.models.empty()) [[unlikely]]
        {
            auto& nlb{nl.models.emplace_back()};
            new(nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
            nl.m_numTermls += pin_view.size;
            if constexpr(::phy_engine::model::defines::can_generate_branch_view<mod>)
            {
                auto const bv{nlb.curr->ptr->generate_branch_view()};
                nl.m_numBranches += bv.size;
            }

            return {
                nlb.curr++,
                {0, 0}
            };
        }
        else
        {
            auto& nlb{nl.models.back_unchecked()};
            if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
            {
                auto& new_nlb{nl.models.emplace_back()};
                new(new_nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
                nl.m_numTermls += pin_view.size;
                if constexpr(::phy_engine::model::defines::can_generate_branch_view<mod>)
                {
                    auto const bv{nlb.curr->ptr->generate_branch_view()};
                    nl.m_numBranches += bv.size;
                }

                return {
                    new_nlb.curr++,
                    {0, nl.models.size() - 1}
                };
            }
            else
            {
                new(nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
                nl.m_numTermls += pin_view.size;
                if constexpr(::phy_engine::model::defines::can_generate_branch_view<mod>)
                {
                    auto const bv{nlb.curr->ptr->generate_branch_view()};
                    nl.m_numBranches += bv.size;
                }

                add_model_retstr return_val{
                    nlb.curr,
                    {static_cast<::std::size_t>(nlb.curr - nlb.begin), nl.models.size() - 1}
                };
                ++nlb.curr;
                return return_val;
            }
        }
    }

    inline constexpr bool delete_model(netlist& nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept
    {
        if(chunk_pos >= nl.models.size()) [[unlikely]] { return false; }
        auto& nlb{nl.models.index_unchecked(chunk_pos)};

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

            auto const i_pin_view{i->ptr->generate_pin_view()};
            nl.m_numTermls -= i_pin_view.size;

            auto const branch_size{i->ptr->generate_branch_view()};
            nl.m_numBranches -= branch_size.size;

            i->~model_base();
            --nlb.curr;
        }
        else
        {
            if(i->type != ::phy_engine::model::model_type::null)
            {
                auto const i_pin_view{i->ptr->generate_pin_view()};

                ++nlb.num_of_null_model;
                nl.m_numTermls -= i_pin_view.size;

                auto const branch_size{i->ptr->generate_branch_view()};
                nl.m_numBranches -= branch_size.size;

                i->clear();
            }
            else [[unlikely]] { return false; }
        }

        return true;
    }

    inline constexpr bool delete_model(netlist& nl, model_pos pos) noexcept { return delete_model(nl, pos.vec_pos, pos.chunk_pos); }

    inline constexpr ::phy_engine::model::model_base* get_model(netlist const& nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept
    {
        if(chunk_pos >= nl.models.size()) [[unlikely]] { return nullptr; }
        auto& nlb{nl.models.index_unchecked(chunk_pos)};
        if(vec_pos >= nlb.size()) [[unlikely]] { return nullptr; }
        return nlb.begin + vec_pos;
    }

    inline constexpr ::phy_engine::model::model_base* get_model(netlist const& nl, model_pos pos) noexcept { return get_model(nl, pos.vec_pos, pos.chunk_pos); }

    inline ::phy_engine::model::node_t& create_node(netlist& nl) noexcept
    {
        if(nl.nodes.empty()) [[unlikely]]
        {
            auto& nlb{nl.nodes.emplace_back()};
            ::std::construct_at(nlb.curr);
            return *(nlb.curr++);
        }
        else
        {
            auto& nlb{nl.nodes.back_unchecked()};
            if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
            {
                auto& new_nlb{nl.nodes.emplace_back()};
                ::std::construct_at(new_nlb.curr);

                return *(new_nlb.curr++);
            }
            else
            {
                ::std::construct_at(nlb.curr);

                return *(nlb.curr++);
            }
        }
    }

    inline constexpr bool add_to_node(netlist& nl, model_pos mp1, ::std::size_t n1, ::phy_engine::model::node_t& node) noexcept
    {
        ::phy_engine::model::model_base* model1{get_model(nl, mp1)};
        if(model1 == nullptr) [[unlikely]] { return false; }

        auto pw1{model1->ptr->generate_pin_view()};
        if(n1 >= pw1.size) [[unlikely]] { return false; }

        auto& p1{pw1.pins[n1]};

        p1.nodes = __builtin_addressof(node);
        node.pins.insert(__builtin_addressof(p1));

        auto const device_type{model1->ptr->get_device_type()};

        if(device_type != ::phy_engine::model::model_device_type::digital) { ++node.num_of_analog_node; }

        return true;
    }

    /* for adl */

    inline constexpr bool
        add_to_node([[maybe_unused]] netlist const& nl, ::phy_engine::model::model_base& model, ::std::size_t n1, ::phy_engine::model::node_t& node) noexcept
    {
        auto pw1{model.ptr->generate_pin_view()};
        if(n1 >= pw1.size) [[unlikely]] { return false; }

        auto& p1{pw1.pins[n1]};

        p1.nodes = __builtin_addressof(node);
        node.pins.insert(__builtin_addressof(p1));

        auto const device_type{model.ptr->get_device_type()};

        if(device_type != ::phy_engine::model::model_device_type::digital) { ++node.num_of_analog_node; }

        return true;
    }

    inline constexpr bool remove_from_node(netlist& nl, model_pos mp1, ::std::size_t n1, ::phy_engine::model::node_t& node) noexcept
    {
        ::phy_engine::model::model_base* model1{get_model(nl, mp1)};
        if(model1 == nullptr) [[unlikely]] { return false; }

        auto pw1{model1->ptr->generate_pin_view()};
        if(n1 >= pw1.size) [[unlikely]] { return false; }

        auto& p1{pw1.pins[n1]};

        p1.nodes = nullptr;
        node.pins.erase(__builtin_addressof(p1));

        auto const device_type{model1->ptr->get_device_type()};

        if(device_type != ::phy_engine::model::model_device_type::digital) { --node.num_of_analog_node; }

        return true;
    }

    /* for adl */

    inline constexpr bool remove_from_node([[maybe_unused]] netlist const& nl,
                                           ::phy_engine::model::model_base& model,
                                           ::std::size_t n1,
                                           ::phy_engine::model::node_t& node) noexcept
    {
        auto pw1{model.ptr->generate_pin_view()};
        if(n1 >= pw1.size) [[unlikely]] { return false; }

        auto& p1{pw1.pins[n1]};

        p1.nodes = nullptr;
        node.pins.erase(__builtin_addressof(p1));

        auto const device_type{model.ptr->get_device_type()};

        if(device_type != ::phy_engine::model::model_device_type::digital) { --node.num_of_analog_node; }

        return true;
    }

    inline void delete_node([[maybe_unused]] netlist const& nl, ::phy_engine::model::node_t& node) noexcept { node.clear(); }

    inline void merge_node([[maybe_unused]] netlist const& nl, ::phy_engine::model::node_t& node, ::phy_engine::model::node_t& other_node) noexcept
    {
        for(auto i: other_node.pins)
        {
            node.pins.insert(i);
            i->nodes = __builtin_addressof(node);
        }
        other_node.clear_node();
    }

    inline void add_netlist(netlist& nl, netlist const& other_nl) noexcept
    {
        ::std::map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> node_map{};

        for(auto& i: other_nl.nodes)
        {
            for(auto c{i.begin}; c != i.curr; ++c)
            {
                if(nl.nodes.empty()) [[unlikely]]
                {
                    auto& nlb{nl.nodes.emplace_back()};
                    // copy and disconnect form the model
                    new(nlb.curr)::phy_engine::model::node_t{*c};
                    node_map[c] = nlb.curr;
                    ++nlb.curr;
                }
                else
                {
                    auto& nlb{nl.nodes.back_unchecked()};
                    if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                    {
                        auto& new_nlb{nl.nodes.emplace_back()};
                        // copy and disconnect form the model
                        new(new_nlb.curr)::phy_engine::model::node_t{*c};
                        node_map[c] = new_nlb.curr;
                        ++new_nlb.curr;
                    }
                    else
                    {
                        // copy and disconnect form the model
                        new(nlb.curr)::phy_engine::model::node_t{*c};
                        node_map[c] = nlb.curr;
                        ++nlb.curr;
                    }
                }
            }
        }

        for(auto& i: other_nl.models)
        {
            for(auto c{i.begin}; c != i.curr; ++c)
            {
                ::phy_engine::model::model_base copy{};
                // copy with node ptr, and can use the mapping table to search for new node
                copy.copy_with_node_ptr(*c);

                auto const copy_pin_view{copy.ptr->generate_pin_view()};

                if(nl.models.empty()) [[unlikely]]
                {
                    auto& nlb{nl.models.emplace_back()};
                    new(nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                }
                else
                {
                    auto& nlb{nl.models.back_unchecked()};
                    if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                    {
                        auto& new_nlb{nl.models.emplace_back()};
                        new(new_nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                    }
                    else { new(nlb.curr++)::phy_engine::model::model_base{::std::move(copy)}; }
                }

                for(auto c{copy_pin_view.pins}; c != copy_pin_view.pins + copy_pin_view.size; ++c)
                {
                    auto& this_node{c->nodes};
                    if(this_node) [[likely]]
                    {
                        auto i{node_map.find(this_node)};
                        // i always != node_map.end()
                        this_node = i->second;
                        i->second->pins.insert(c);
                    }
                }
            }
        }
    }

}  // namespace phy_engine::netlist
