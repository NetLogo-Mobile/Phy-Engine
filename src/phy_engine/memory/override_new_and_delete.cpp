/**************
 * Phy Engine *
 *************/

#include <new>

#include "../../phy_engine_utils/fast_io/fast_io_core.h"

#if (__cplusplus > 201402L || defined(__cpp_aligned_new))

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(allocator)
#endif
[[nodiscord]] void* operator new(::std::size_t n) noexcept { 
	return ::fast_io::native_global_allocator::allocate(n);
}

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(allocator)
#endif
[[nodiscord]] void* operator new(::std::size_t n, ::std::align_val_t al) noexcept {
	return ::fast_io::native_global_allocator::allocate_aligned(static_cast<::std::size_t>(al), n);
}

void operator delete(void* ptr) noexcept {
	::fast_io::native_global_allocator::deallocate(ptr);
}

#if 0 // not support
void operator delete(void* ptr, ::std::size_t n, ::std::align_val_t al) noexcept {
	::fast_io::native_global_allocator::deallocate_aligned_n(ptr, static_cast<::std::size_t>(al), n);
}

void operator delete(void* ptr, ::std::align_val_t al) noexcept {
	::fast_io::native_global_allocator::deallocate_aligned(ptr, static_cast<::std::size_t>(al));
}
#endif  // 0

#endif