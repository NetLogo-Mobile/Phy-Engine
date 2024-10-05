#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/string.h>
#include <absl/container/hash_container_defaults.h>

namespace absl::container_internal
{
    // define absl
    template <::std::integral char_type, typename allocator>
    struct HashEq<::fast_io::containers::basic_string<char_type, allocator>> : BasicStringHashEq<char_type>
    {
    };

    template <::std::integral char_type>
    struct HashEq<::fast_io::containers::basic_string_view<char_type>> : BasicStringHashEq<char_type>
    {
    };
}  // namespace absl::container_internal
