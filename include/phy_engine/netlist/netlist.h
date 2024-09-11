#pragma once
#include <cstddef>
#include <type_traits>
#include <deque>

#include <fast_io/fast_io_core.h>
#include <fast_io/fast_io_dsal/vector.h>
#include "../model/model_refs/base.h"

namespace phy_engine::netlist
{
    namespace details
    {

        struct netlist_block
        {
            using Alloc = ::fast_io::native_typed_global_allocator<::phy_engine::model::model_base>;
            inline static constexpr ::std::size_t chunk_size{4096};
            inline static constexpr ::std::size_t chunk_module_size{chunk_size / sizeof(::phy_engine::model::model_base)};

            constexpr netlist_block() noexcept
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

            constexpr netlist_block(netlist_block const& other) noexcept
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

            constexpr netlist_block& operator= (netlist_block const& other) noexcept
            {
                if(__builtin_addressof(other) == this) [[unlikely]] { return *this; }

                for(::phy_engine::model::model_base* b{begin}; b != curr; b++) { b->~model_base(); }

                auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
                curr = begin + size;
                num_of_null_model = other.num_of_null_model;
                for(::std::size_t i{}; i < size; i++) { new(begin + i)::phy_engine::model::model_base{other.begin[i]}; }
                return *this;
            }

            constexpr netlist_block(netlist_block&& other) noexcept
            {
                begin = other.begin;
                curr = other.curr;
                num_of_null_model = other.num_of_null_model;
                other.begin = nullptr;
                other.curr = nullptr;
                other.num_of_null_model = 0;
            }

            constexpr netlist_block& operator= (netlist_block&& other) noexcept
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

            constexpr netlist_block& move_without_delete_memory(netlist_block&& other) noexcept
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

            constexpr ~netlist_block() noexcept
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
            ::std::size_t num_of_null_model{};
        };
    }  // namespace details

    struct netlist
    {
        using Alloc = ::fast_io::native_global_allocator;
        ::fast_io::vector<details::netlist_block> netlist_memory{};
        ::std::deque<::phy_engine::model::node_t> nodes{};
        ::std::size_t m_numNodes{};
        ::std::size_t m_numBranches{};
        ::std::size_t m_numTermls{};
        bool m_hasGround{};
    };

}  // namespace phy_engine::netlist

namespace fast_io::freestanding
{
    template <>
    struct is_trivially_relocatable<::phy_engine::netlist::details::netlist_block>
    {
        inline static constexpr bool value = true;
    };

}  // namespace fast_io::freestanding
