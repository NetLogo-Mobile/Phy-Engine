#pragma once

#include <cstdint>
#include <cstddef>
#include <concepts>
#include <compare>

#include <fast_io/fast_io.h>
#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine
{
    struct version
    {
        ::std::uint_least32_t x{};
        ::std::uint_least32_t y{};
        ::std::uint_least32_t z{};
        ::std::uint_least32_t state{};
    };

    inline constexpr bool operator== (version v1, version v2) noexcept { return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z && v1.state == v2.state; }

    inline constexpr ::std::strong_ordering operator<=> (version v1, version v2) noexcept
    {
        auto const cx{v1.x <=> v2.x};
        if(cx == 0)
        {
            auto const cy{v1.y <=> v2.y};
            if(cy == 0)
            {
                auto const cz{v1.z <=> v2.z};
                if(cz == 0) { return v1.state <=> v2.state; }
                return cz;
            }
            return cy;
        }
        return cx;
    }

    template <::std::integral char_type>
    inline constexpr ::std::size_t print_reserve_size(::fast_io::io_reserve_type_t<char_type, version>) noexcept
    {
        constexpr ::std::size_t real_size{::fast_io::pr_rsv_size<char_type, ::std::uint_least32_t>};
        constexpr ::std::size_t size{3 + 4 * real_size};
        return size;
    }

    namespace details
    {
        template <::std::integral char_type>
        inline constexpr char_type* version_print_reserve_impl(char_type* iter,
                                                               ::std::uint_least32_t x,
                                                               ::std::uint_least32_t y,
                                                               ::std::uint_least32_t z,
                                                               ::std::uint_least32_t state) noexcept
        {
            constexpr auto point{::fast_io::char_literal_v<u8'.', char_type>};
            char_type* curr_pos{::fast_io::pr_rsv_to_iterator_unchecked(iter, x)};
            *(curr_pos++) = point;
            curr_pos = ::fast_io::pr_rsv_to_iterator_unchecked(curr_pos, y);
            *(curr_pos++) = point;
            curr_pos = ::fast_io::pr_rsv_to_iterator_unchecked(curr_pos, z);
            *(curr_pos++) = point;
            curr_pos = ::fast_io::pr_rsv_to_iterator_unchecked(curr_pos, state);
            return curr_pos;
        }
    }  // namespace details

    template <::std::integral char_type>
    inline constexpr char_type* print_reserve_define(::fast_io::io_reserve_type_t<char_type, version>, char_type* iter, version ver) noexcept
    {
        return details::version_print_reserve_impl(iter, ver.x, ver.y, ver.z, ver.state);
    }

#if defined(UWVM_VERSION_X) && defined(UWVM_VERSION_Y) && defined(UWVM_VERSION_Z) && defined(UWVM_VERSION_S)
    inline constexpr version uwvm_version{UWVM_VERSION_X, UWVM_VERSION_Y, UWVM_VERSION_Z, UWVM_VERSION_S};
    inline constexpr ::fast_io::u8string_view git_fetch_head{
    #include "../../../.tmp/git_commit_hash.h"
    };
#else
    inline constexpr version uwvm_version{};
    inline constexpr ::fast_io::u8string_view git_fetch_head{u8"No FETCH_HEAD information"};
#endif

}  // namespace phy_engine
