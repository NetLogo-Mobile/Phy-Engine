#pragma once

#include <string>

namespace phy_engine::pe_nl_fileformat
{
    enum class errc
    {
        ok = 0,
        invalid_argument,
        io_error,
        db_error,
        corrupt,
        unsupported,
        not_found
    };

    struct status
    {
        errc code{errc::ok};
        std::string message{};

        constexpr status() noexcept = default;
        constexpr status(errc c, std::string m) : code{c}, message{std::move(m)} {}

        [[nodiscard]] constexpr bool ok() const noexcept { return code == errc::ok; }
        [[nodiscard]] constexpr explicit operator bool() const noexcept { return ok(); }

        static status success() { return {}; }
    };
}  // namespace phy_engine::pe_nl_fileformat

