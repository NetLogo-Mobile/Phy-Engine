#pragma once
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_crypto.h>

namespace phy_engine
{
    inline constexpr ::std::uint_least32_t generate_crc32c_hash(::std::byte const* begin, ::std::byte const* end) noexcept
    {
        ::fast_io::crc32c_context crc32c{};
        crc32c.update(begin, end);
        return crc32c.digest_value();
    }
}  // namespace phy_engine
