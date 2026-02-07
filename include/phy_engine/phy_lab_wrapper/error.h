#pragma once

#include <string>
#include <utility>

namespace phy_engine::phy_lab_wrapper::detail
{
    inline thread_local std::string last_error{};

    inline void set_last_error(std::string msg) { last_error = std::move(msg); }
    inline void clear_last_error() noexcept { last_error.clear(); }

    [[nodiscard]] inline char const* last_error_c_str() noexcept { return last_error.c_str(); }
}  // namespace phy_engine::phy_lab_wrapper::detail

