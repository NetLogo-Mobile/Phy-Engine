#pragma once
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_dsal/vector.h>

namespace phy_engine::verilog
{
    enum class token_type_t : ::std::size_t
    {
        null,
        unknown,
        keyword,
        delimiter,
        arithmetic_operator,
        relational_operator,
        identifier

    };

    struct token_t
    {
        // token_type_t token_type{};
        char8_t const* token_begin{};
        char8_t const* token_end{};
    };

    struct syntax_t
    {
        ::fast_io::vector<token_t> tokens{};
    };
}  // namespace phy_engine::verilog

namespace fast_io::freestanding
{
    template <>
    struct is_trivially_relocatable<::phy_engine::verilog::syntax_t>
    {
        inline static constexpr bool value = true;
    };

    template <>
    struct is_zero_default_constructible<::phy_engine::verilog::syntax_t>
    {
        inline static constexpr bool value = true;
    };

}  // namespace fast_io::freestanding

