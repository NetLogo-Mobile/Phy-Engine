#pragma once

#include <type_traits>
#include <concepts>

#include <fast_io/fast_io.h>
#include <fast_io/fast_io_core.h>
#include <fast_io/fast_io_dsal/array.h>

namespace phy_engine::verilog
{

    template <::std::size_t Len = 0, ::std::integral char_type>
#if __has_cpp_attribute(__gnu__::__const__)
    [[__gnu__::__const__]]
#endif
    inline constexpr ::std::size_t
        shortest_way(char_type const* x, ::std::size_t lena, char_type const* y, ::std::size_t lenb) noexcept
    {
        ::std::size_t* d{
#if __has_builtin(__builtin_alloca)
            __builtin_alloca(lenb + 1)
#elif defined(_WIN32) && !defined(__WINE__) && !defined(__BIONIC__) && !defined(__CYGWIN__)
            _alloca(lenb + 1)
#else
            alloca(lenb + 1)
#endif
        };

        for(::std::size_t j{}; j <= lenb; j++) { d[j] = j; }

        for(::std::size_t i{1}; i <= lena; i++)
        {
            ::std::size_t old{i - 1};
            d[0] = i;
            for(::std::size_t j{1}; j <= lenb; j++)
            {
                ::std::size_t const temp{d[j]};
                if(x[i - 1] == y[j - 1]) { d[j] = old; }
                else
                {
                    ::std::size_t min = d[j] + 1;
                    if(d[j - 1] + 1 < min) { min = d[j - 1] + 1; }
                    if(old + 1 < min) { min = old + 1; }
                    d[j] = min;
                }
                old = temp;
            }
        }

        ::std::size_t const ret{d[lenb]};

        return ret;
    }

}  // namespace phy_engine::verilog
