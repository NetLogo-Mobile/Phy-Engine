#pragma once
#include <cstddef>
#include <type_traits>
#include <map>
#include <deque>

#include <fast_io/fast_io_core.h>
#include <fast_io/fast_io_dsal/vector.h>
#include "../model/model_refs/base.h"

namespace phy_engine::netlist
{
    namespace details
    {

        struct netlist_model_base_block
        {
            using Alloc = ::fast_io::native_typed_global_allocator<::phy_engine::model::model_base>;
            inline static constexpr ::std::size_t chunk_size{4096};
            inline static constexpr ::std::size_t chunk_module_size{chunk_size / sizeof(::phy_engine::model::model_base)};

            constexpr netlist_model_base_block() noexcept
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    begin = new ::phy_engine::model::model_base[chunk_module_size];
                }
                else
#endif
                {
                    begin = Alloc::allocate(chunk_module_size);
                }

                curr = begin;
            }

            constexpr netlist_model_base_block(netlist_model_base_block const& other) noexcept
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    begin = new ::phy_engine::model::model_base[chunk_module_size];
                }
                else
#endif
                {
                    begin = Alloc::allocate(chunk_module_size);
                }

                auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
                curr = begin + size;
                num_of_null_model = other.num_of_null_model;
                for(::std::size_t i{}; i < size; i++) { new(begin + i)::phy_engine::model::model_base{other.begin[i]}; }
            }

            constexpr netlist_model_base_block& operator= (netlist_model_base_block const& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::model_base* b{begin}; b != curr; b++) { b->~model_base(); }

                auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
                curr = begin + size;
                num_of_null_model = other.num_of_null_model;
                for(::std::size_t i{}; i < size; i++) { new(begin + i)::phy_engine::model::model_base{other.begin[i]}; }
                return *this;
            }

            constexpr netlist_model_base_block(netlist_model_base_block&& other) noexcept
            {
                begin = other.begin;
                curr = other.curr;
                num_of_null_model = other.num_of_null_model;
                other.begin = nullptr;
                other.curr = nullptr;
                other.num_of_null_model = 0;
            }

            constexpr netlist_model_base_block& operator= (netlist_model_base_block&& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::model_base* b{begin}; b != curr; b++) { b->~model_base(); }

                if(begin != nullptr)
                {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                    if consteval
    #else
                    if(__builtin_is_constant_evaluated())
    #endif
                    {
                        delete[] begin;
                    }
                    else
#endif
                    {
                        Alloc::deallocate(begin);
                    }
                }

                begin = other.begin;
                curr = other.curr;
                num_of_null_model = other.num_of_null_model;
                other.begin = nullptr;
                other.curr = nullptr;
                other.num_of_null_model = 0;
                return *this;
            }

            constexpr netlist_model_base_block& move_without_delete_memory(netlist_model_base_block&& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::model_base* b{begin}; b != curr; b++) { b->~model_base(); }
                ::phy_engine::model::model_base* const temp_begin{begin};

                begin = other.begin;
                curr = other.curr;
                num_of_null_model = other.num_of_null_model;
                other.begin = temp_begin;
                other.curr = temp_begin;
                other.num_of_null_model = 0;
                return *this;
            }

            constexpr ~netlist_model_base_block() noexcept
            {
                for(::phy_engine::model::model_base* b{begin}; b != curr; b++) { b->~model_base(); }

                if(begin != nullptr)
                {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                    if consteval
    #else
                    if(__builtin_is_constant_evaluated())
    #endif
                    {
                        delete[] begin;
                    }
                    else
#endif
                    {
                        Alloc::deallocate(begin);
                    }
                }

                begin = nullptr;
                curr = nullptr;
                num_of_null_model = 0;
            }

            constexpr void clear() noexcept
            {
                for(::phy_engine::model::model_base* b{begin}; b != curr; b++) { b->~model_base(); }
                curr = begin;
                num_of_null_model = 0;
            }

            constexpr ::std::size_t size() const noexcept { return static_cast<::std::size_t>(curr - begin); }

            constexpr ::std::size_t get_num_of_model() const noexcept { return static_cast<::std::size_t>(curr - begin) - num_of_null_model; }

            ::phy_engine::model::model_base* begin{};
            ::phy_engine::model::model_base* curr{};
            // ::fast_io::vector<::phy_engine::model::model_base*> null_model{};
            ::std::size_t num_of_null_model{};
        };

        struct netlist_node_block
        {
            using Alloc = ::fast_io::native_typed_global_allocator<::phy_engine::model::node_t>;
            inline static constexpr ::std::size_t chunk_size{4096};
            inline static constexpr ::std::size_t chunk_module_size{chunk_size / sizeof(::phy_engine::model::node_t)};

            constexpr netlist_node_block() noexcept
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    begin = new ::phy_engine::model::node_t[chunk_module_size];
                }
                else
#endif
                {
                    begin = Alloc::allocate(chunk_module_size);
                }

                curr = begin;
            }

            constexpr netlist_node_block(netlist_node_block const& other) noexcept
            {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                if consteval
    #else
                if(__builtin_is_constant_evaluated())
    #endif
                {
                    begin = new ::phy_engine::model::node_t[chunk_module_size];
                }
                else
#endif
                {
                    begin = Alloc::allocate(chunk_module_size);
                }

                auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
                curr = begin + size;
                for(::std::size_t i{}; i < size; i++) { new(begin + i)::phy_engine::model::node_t{other.begin[i]}; }
            }

            constexpr netlist_node_block& operator= (netlist_node_block const& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::node_t* b{begin}; b != curr; b++) { b->~node_t(); }

                auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
                curr = begin + size;
                for(::std::size_t i{}; i < size; i++) { new(begin + i)::phy_engine::model::node_t{other.begin[i]}; }
                return *this;
            }

            constexpr netlist_node_block(netlist_node_block&& other) noexcept
            {
                begin = other.begin;
                curr = other.curr;
                other.begin = nullptr;
                other.curr = nullptr;
            }

            constexpr netlist_node_block& operator= (netlist_node_block&& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::node_t* b{begin}; b != curr; b++) { b->~node_t(); }

                if(begin != nullptr)
                {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                    if consteval
    #else
                    if(__builtin_is_constant_evaluated())
    #endif
                    {
                        delete[] begin;
                    }
                    else
#endif
                    {
                        Alloc::deallocate(begin);
                    }
                }

                begin = other.begin;
                curr = other.curr;
                other.begin = nullptr;
                other.curr = nullptr;
                return *this;
            }

            constexpr netlist_node_block& move_without_delete_memory(netlist_node_block&& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::node_t* b{begin}; b != curr; b++) { b->~node_t(); }
                ::phy_engine::model::node_t* const temp_begin{begin};

                begin = other.begin;
                curr = other.curr;
                other.begin = temp_begin;
                other.curr = temp_begin;
                return *this;
            }

            constexpr ~netlist_node_block() noexcept
            {
                for(::phy_engine::model::node_t* b{begin}; b != curr; b++) { b->~node_t(); }

                if(begin != nullptr)
                {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
    #if __cpp_if_consteval >= 202106L
                    if consteval
    #else
                    if(__builtin_is_constant_evaluated())
    #endif
                    {
                        delete[] begin;
                    }
                    else
#endif
                    {
                        Alloc::deallocate(begin);
                    }
                }

                begin = nullptr;
                curr = nullptr;
            }

            constexpr void clear() noexcept
            {
                for(::phy_engine::model::node_t* b{begin}; b != curr; b++) { b->~node_t(); }
                curr = begin;
            }

            ::phy_engine::model::node_t* begin{};
            ::phy_engine::model::node_t* curr{};
        };

    }  // namespace details
}  // namespace phy_engine::netlist

namespace fast_io::freestanding
{
    template <>
    struct is_trivially_relocatable<::phy_engine::netlist::details::netlist_model_base_block>
    {
        inline static constexpr bool value = true;
    };

    template <>
    struct is_trivially_relocatable<::phy_engine::netlist::details::netlist_node_block>
    {
        inline static constexpr bool value = true;
    };
}  // namespace fast_io::freestanding

namespace phy_engine::netlist
{
    struct netlist
    {
        using Alloc = ::fast_io::native_global_allocator;
        ::fast_io::vector<::phy_engine::netlist::details::netlist_model_base_block> models{};
        ::fast_io::vector<::phy_engine::netlist::details::netlist_node_block> nodes{};
        //::std::size_t m_numNodes{};
        //::std::size_t m_numBranches{};
        ::std::size_t m_numTermls{};
        // bool m_hasGround{};
        constexpr netlist() noexcept = default;

        // need test !!!
        netlist(netlist const& other) noexcept : m_numTermls{other.m_numTermls}
        {
            ::std::map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> node_map{};

            for(auto& i: other.nodes)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(nodes.empty()) [[unlikely]]
                    {
                        auto& nlb{nodes.emplace_back()};
                        // copy and disconnect form the model
                        new(nlb.curr)::phy_engine::model::node_t{*c};
                        node_map[c] = nlb.curr;
                        ++nlb.curr;
                    }
                    else
                    {
                        auto& nlb{nodes.back_unchecked()};
                        if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                        {
                            auto& new_nlb{nodes.emplace_back()};
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

            for(auto& i: other.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    ::phy_engine::model::model_base copy{};
                    // copy with node ptr, and can use the mapping table to search for new node
                    copy.copy_with_node_ptr(*c);

                    auto const copy_pin_view{copy.ptr->generate_pin_view()};

                    if(models.empty()) [[unlikely]]
                    {
                        auto& nlb{models.emplace_back()};
                        new(nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                    }
                    else
                    {
                        auto& nlb{models.back_unchecked()};
                        if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                        {
                            auto& new_nlb{models.emplace_back()};
                            new(new_nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                        }
                        else
                        {
                            new(nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                        }
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

        netlist& operator= (netlist const& other) noexcept
        {
            if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }
            models.clear();
            nodes.clear();
            m_numTermls = other.m_numTermls;

            // deep copy
            ::std::map<::phy_engine::model::node_t*, ::phy_engine::model::node_t*> node_map{};

            for(auto& i: other.nodes)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(nodes.empty()) [[unlikely]]
                    {
                        auto& nlb{nodes.emplace_back()};
                        // copy and disconnect form the model
                        new(nlb.curr)::phy_engine::model::node_t{*c};
                        node_map[c] = nlb.curr;
                        ++nlb.curr;
                    }
                    else
                    {
                        auto& nlb{nodes.back_unchecked()};
                        if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                        {
                            auto& new_nlb{nodes.emplace_back()};
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

            for(auto& i: other.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    ::phy_engine::model::model_base copy{};
                    // copy with node ptr, and can use the mapping table to search for new node
                    copy.copy_with_node_ptr(*c);

                    auto const copy_pin_view{copy.ptr->generate_pin_view()};

                    if(models.empty()) [[unlikely]]
                    {
                        auto& nlb{models.emplace_back()};
                        new(nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                    }
                    else
                    {
                        auto& nlb{models.back_unchecked()};
                        if(nlb.curr == nlb.begin + nlb.chunk_module_size) [[unlikely]]
                        {
                            auto& new_nlb{models.emplace_back()};
                            new(new_nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                        }
                        else
                        {
                            new(nlb.curr++)::phy_engine::model::model_base{::std::move(copy)};
                        }
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
            return *this;
        }
    };

}  // namespace phy_engine::netlist

