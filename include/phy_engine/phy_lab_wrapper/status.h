#pragma once

#include <optional>
#include <string>
#include <system_error>
#include <utility>

namespace phy_engine::phy_lab_wrapper
{
    struct status
    {
        std::errc ec{};
        std::string message{};

        constexpr status() noexcept = default;
        status(std::errc e, std::string m) : ec{e}, message{std::move(m)} {}

        [[nodiscard]] constexpr bool ok() const noexcept { return ec == std::errc{}; }
        [[nodiscard]] constexpr explicit operator bool() const noexcept { return ok(); }

        static status success() { return {}; }
    };

    template <class T>
    struct status_or
    {
        std::optional<T> value{};
        status st{};

        constexpr status_or() noexcept = default;
        status_or(T v) : value{std::move(v)}, st{} {}
        status_or(status s) : value{std::nullopt}, st{std::move(s)} {}

        [[nodiscard]] constexpr bool ok() const noexcept { return value.has_value(); }
        [[nodiscard]] constexpr explicit operator bool() const noexcept { return ok(); }
    };
}  // namespace phy_engine::phy_lab_wrapper
