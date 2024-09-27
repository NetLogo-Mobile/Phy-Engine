#pragma once
#include <cstdint>
#include <cstddef>
#include "../node/node.h"

namespace phy_engine::model
{

    enum class variant_type : ::std::uint_fast8_t
    {
        invalid,

        i8,
        i16,
        i32,
        i64,

        ui8,
        ui16,
        ui32,
        ui64,

        boolean,
        f,
        d,

        digital
    };

    struct variant
    {
        union
        {
            ::std::int_least8_t i8;
            ::std::int_least16_t i16;
            ::std::int_least32_t i32;
            ::std::int_least64_t i64;

            ::std::uint_least8_t ui8;
            ::std::uint_least16_t ui16;
            ::std::uint_least32_t ui32;
            ::std::uint_least64_t ui64{};

            bool boolean;
            float f;
            double d;

            ::phy_engine::model::digital_node_statement_t digital;
        };

        variant_type type{};
    };

}  // namespace phy_engine::model
