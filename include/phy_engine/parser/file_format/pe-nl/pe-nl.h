#pragma once

#include <fast_io/fast_io_dsal/string_view.h>

namespace phy_engine::file_format::pe_nl
{
    struct pe_nl
    {
        static constexpr ::fast_io::u8string_view extension{u8"penl"};
    };

    inline constexpr bool load_file(pe_nl const& penl, ::fast_io::u8string_view sv) noexcept { return false; }
}  // namespace phy_engine::file_format::pe_nl
