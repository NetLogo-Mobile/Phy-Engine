#pragma once

#include <type_traits>
#include <concepts>
#include <compare>
#include <algorithm>

#include <fast_io/fast_io.h>
#include <fast_io/fast_io_crypto.h>
#include <fast_io/fast_io_dsal/array.h>
#include <fast_io/fast_io_dsal/vector.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/cstring_view.h>

#include "../../ast/module.h"

namespace phy_engine
{
    namespace verilog::constexpr_hash
    {

        struct pretreatment
        {
            ::fast_io::u8string_view name{};
            void (*callback)(::phy_engine::verilog::Verilog_module& vmod, char8_t const* begin) noexcept {};
        };

        template <::std::size_t N>
        inline
#if __cpp_consteval >= 201811L
            consteval
#else
            constexpr
#endif
            auto
            pretreatment_sort(pretreatment const* const (&punsort)[N]) noexcept
        {
            ::fast_io::array<pretreatment const*, N> res{};
            for(::std::size_t i{}; i < N; ++i) { res.index_unchecked(i) = punsort[i]; }
            ::std::ranges::sort(res,
                                [](pretreatment const* const a, pretreatment const* const b) noexcept -> bool
                                {
#if __cpp_lib_three_way_comparison >= 201907L
                                    return a->name < b->name;
#else
                                    return ::std::lexicographical_compare(a->name.cbegin(), a->name.cend(), b->name.cbegin(), b->name.cend());
#endif
                                });
            return res;
        }

        template <::std::size_t N>
        inline
#if __cpp_consteval >= 201811L
            consteval
#else
            constexpr
#endif
            ::std::size_t
            calculate_max_pretreatment_size(::fast_io::array<pretreatment const*, N> const& punsort) noexcept
        {
            ::std::size_t max_size{};
            for(::std::size_t i{}; i < N; ++i) { max_size = ::std::max(max_size, punsort.index_unchecked(i)->name.size()); }
            return max_size;
        }

        inline constexpr ::std::size_t hash_size_base{4u};  // 2 ^ hash_size_base
        inline constexpr ::std::size_t max_conflict_size{8u};

        struct calculate_hash_table_size_res
        {
            ::std::size_t hash_table_size{};
            ::std::size_t extra_size{};
        };

        template <::std::size_t N>
        inline
#if __cpp_consteval >= 201811L
            consteval
#else
            constexpr
#endif
            calculate_hash_table_size_res
            calculate_hash_table_size(::fast_io::array<pretreatment const*, N> const& ord) noexcept
        {
            constexpr auto sizet_d10{static_cast<::std::size_t>(::std::numeric_limits<::std::size_t>::digits10)};

            ::fast_io::crc32c_context crc32c{};
            for(auto i{hash_size_base}; i < sizet_d10; ++i)
            {
                ::std::size_t const hash_size{static_cast<::std::size_t>(1u) << i};
                bool c{};
                ::std::size_t extra_size{};
                ::std::size_t* const hash_size_array{new ::std::size_t[hash_size]{}};
                for(auto const j: ord)
                {
                    ::std::size_t const j_str_size{j->name.size()};
                    ::std::byte* const ptr{new ::std::byte[j_str_size]{}};
                    for(::std::size_t k{}; k < j_str_size; ++k) { ptr[k] = static_cast<::std::byte>(j->name.index_unchecked(k)); }
                    crc32c.reset();
                    crc32c.update(ptr, ptr + j_str_size);
                    delete[] ptr;
                    auto const val{crc32c.digest_value() % hash_size};
                    ++hash_size_array[val];
                    if(hash_size_array[val] == 2) { ++extra_size; }
                    if(hash_size_array[val] > max_conflict_size) { c = true; }
                }

                delete[] hash_size_array;
                if(!c) { return {hash_size, extra_size}; }
            }
            ::fast_io::fast_terminate();  // error
            return {};
        }

        struct conflict_type_t
        {
            ::fast_io::u8string_view name{};
            pretreatment const* pre{};
        };

        struct conflict_table
        {
            ::fast_io::array<conflict_type_t, max_conflict_size> ctmem{};
        };

        template <::std::size_t hash_table_size, ::std::size_t conflict_size>
        struct pretreatment_hash_table
        {
            static_assert(hash_table_size > 1);

            ::fast_io::array<conflict_type_t, hash_table_size> ht{};
            ::std::conditional_t<static_cast<bool>(conflict_size), ::fast_io::array<conflict_table, conflict_size>, ::std::in_place_t> ct{};
        };

        template <::std::size_t hash_table_size, ::std::size_t conflict_size, ::std::size_t N>
        inline
#if __cpp_consteval >= 201811L
            consteval
#else
            constexpr
#endif
            auto
            generate_hash_table(::fast_io::array<pretreatment const*, N> const& ord) noexcept
        {
            pretreatment_hash_table<hash_table_size, conflict_size> res{};

            ::fast_io::crc32c_context crc32c{};
            ::std::size_t conflictplace{1u};

            for(auto const j: ord)
            {
                ::std::size_t const j_str_size{j->name.size()};
                ::std::byte* const ptr{new ::std::byte[j_str_size]{}};
                for(::std::size_t k{}; k < j_str_size; ++k) { ptr[k] = static_cast<::std::byte>(j->name.index_unchecked(k)); }
                crc32c.reset();
                crc32c.update(ptr, ptr + j_str_size);
                delete[] ptr;
                auto const val{crc32c.digest_value() % hash_table_size};
                if constexpr(conflict_size)
                {
                    if(res.ht.index_unchecked(val).pre == nullptr)
                    {
                        if(!res.ht.index_unchecked(val).name.empty())
                        {
                            for(auto& i: res.ct.index_unchecked(res.ht.index_unchecked(val).name.size() - 1).ctmem)
                            {
                                if(i.pre == nullptr)
                                {
                                    i.pre = j;
                                    i.name = j->name;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            res.ht.index_unchecked(val).pre = j;
                            res.ht.index_unchecked(val).name = j->name;
                        }
                    }
                    else
                    {
                        res.ct.index_unchecked(conflictplace - 1).ctmem.front_unchecked().pre = res.ht.index_unchecked(val).pre;
                        res.ct.index_unchecked(conflictplace - 1).ctmem.front_unchecked().name = res.ht.index_unchecked(val).name;
                        res.ht.index_unchecked(val).pre = nullptr;
                        res.ht.index_unchecked(val).name.ptr = nullptr;
                        res.ht.index_unchecked(val).name.n = conflictplace;
                        res.ct.index_unchecked(conflictplace - 1).ctmem.index_unchecked(1).pre = j;
                        res.ct.index_unchecked(conflictplace - 1).ctmem.index_unchecked(1).name = j->name;
                        ++conflictplace;
                    }
                }
                else
                {
                    res.ht.index_unchecked(val).pre = j;
                    res.ht.index_unchecked(val).name = j->name;
                }
            }
            return res;
        }

        struct has_table_view
        {
            ::std::size_t hash_table_size{};
            ::std::size_t conflict_size{};
            conflict_type_t const* ht_view{};  // hash_table_size
            conflict_table const* ct_view{};   // conflict_size
        };

        template <::std::size_t hash_table_size, ::std::size_t conflict_size>
        inline
#if __cpp_consteval >= 201811L
            consteval
#else
            constexpr
#endif
            has_table_view
            generate_hash_table_view(pretreatment_hash_table<hash_table_size, conflict_size> const& ord) noexcept
        {
            has_table_view ret{hash_table_size, conflict_size, ord.ht.data(), nullptr};
            if constexpr(conflict_size) { ret.ct_view = ord.ct.data(); }
            return ret;
        }

        inline constexpr pretreatment const* find_from_hash_table_view(has_table_view const& ht, ::fast_io::u8string_view str) noexcept
        {
            ::fast_io::crc32c_context crc32c{};

#if __cpp_if_consteval >= 202106L
            if consteval
#else
            if(__builtin_is_constant_evaluated())
#endif
            {
                auto const str_size{str.size()};
                ::std::byte* const ptr{new ::std::byte[str_size]{}};
                for(::std::size_t k{}; k < str_size; ++k) { ptr[k] = static_cast<::std::byte>(str.index_unchecked(k)); }
                crc32c.update(ptr, ptr + str_size);
                delete[] ptr;
            }
            else
            {
                auto const i{reinterpret_cast<::std::byte const*>(str.data())};
                crc32c.update(i, i + str.size());
            }
            auto const val{crc32c.digest_value() % ht.hash_table_size};
            auto const htval{ht.ht_view[val]};
            if(ht.conflict_size)
            {
                if(htval.pre == nullptr)
                {
                    if(!htval.name.empty())
                    {
                        auto const& ct{ht.ct_view[htval.name.size() - 1].ctmem};
                        for(::std::size_t i{}; i < max_conflict_size; ++i)
                        {
                            if(ct.index_unchecked(i).name == str) { return ct.index_unchecked(i).pre; }
                            else if(ct.index_unchecked(i).pre == nullptr) [[unlikely]] { return nullptr; }
                        }
                        return nullptr;
                    }
                    else [[unlikely]] { return nullptr; }
                }
                else
                {
                    if(str == htval.name) { return htval.pre; }
                    else [[unlikely]] { return nullptr; }
                }
            }
            else
            {
                if(htval.pre != nullptr)
                {
                    if(str == htval.name) { return htval.pre; }
                    else [[unlikely]] { return nullptr; }
                }
                else [[unlikely]] { return nullptr; }
            }
        }

    }  // namespace verilog::constexpr_hash
}  // namespace phy_engine
