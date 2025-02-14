﻿#pragma once

namespace fast_io
{

namespace win32
{
struct memory_basic_information
{
	void *BaseAddress;
	void *AllocationBase;
	::std::uint_least32_t AllocationProtect;
#if defined(_WIN64)
	::std::uint_least16_t PartitionId;
#endif
	::std::size_t RegionSize;
	::std::uint_least32_t State;
	::std::uint_least32_t Protect;
	::std::uint_least32_t Type;
};
} // namespace win32

namespace win32
{
#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
#if __has_cpp_attribute(__gnu__::__malloc__)
[[__gnu__::__malloc__]]
#endif
extern void *
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	HeapAlloc(void *, ::std::uint_least32_t, ::std::size_t) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("HeapAlloc@12")
#else
	__asm__("_HeapAlloc@12")
#endif
#else
	__asm__("HeapAlloc")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
extern int
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	HeapFree(void *, ::std::uint_least32_t, void *) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("HeapFree@12")
#else
	__asm__("_HeapFree@12")
#endif
#else
	__asm__("HeapFree")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
#if __has_cpp_attribute(__gnu__::__const__)
[[__gnu__::__const__]]
#endif
extern void *
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	GetProcessHeap() noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("GetProcessHeap@0")
#else
	__asm__("_GetProcessHeap@0")
#endif
#else
	__asm__("GetProcessHeap")
#endif
#endif

		;
#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
extern void *
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	HeapReAlloc(void *, ::std::uint_least32_t, void *, ::std::size_t) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("HeapReAlloc@16")
#else
	__asm__("_HeapReAlloc@16")
#endif
#else
	__asm__("HeapReAlloc")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
extern ::std::size_t
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	HeapSize(void *, ::std::uint_least32_t, void const *) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("HeapSize@12")
#else
	__asm__("_HeapSize@12")
#endif
#else
	__asm__("HeapSize")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
#if __has_cpp_attribute(__gnu__::__const__)
[[__gnu__::__const__]]
#endif
extern void *
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	VirtualAlloc(void *, ::std::size_t, ::std::uint_least32_t, ::std::uint_least32_t) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("VirtualAlloc@16")
#else
	__asm__("_VirtualAlloc@16")
#endif
#else
	__asm__("VirtualAlloc")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
#if __has_cpp_attribute(__gnu__::__const__)
[[__gnu__::__const__]]
#endif
extern int
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	VirtualProtect(void *, ::std::size_t, ::std::uint_least32_t, ::std::uint_least32_t *) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("VirtualProtect@16")
#else
	__asm__("_VirtualProtect@16")
#endif
#else
	__asm__("VirtualProtect")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
#if __has_cpp_attribute(__gnu__::__const__)
[[__gnu__::__const__]]
#endif
extern int
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	VirtualFree(void *, ::std::size_t, ::std::uint_least32_t) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("VirtualFree@12")
#else
	__asm__("_VirtualFree@12")
#endif
#else
	__asm__("VirtualFree")
#endif
#endif
		;

#if defined(_MSC_VER) && !defined(__clang__)
__declspec(dllimport)
#elif (__has_cpp_attribute(__gnu__::__dllimport__) && !defined(__WINE__))
[[__gnu__::__dllimport__]]
#endif
#if (__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__))
[[__gnu__::__stdcall__]]
#endif
#if __has_cpp_attribute(__gnu__::__const__)
[[__gnu__::__const__]]
#endif
extern int
#if (!__has_cpp_attribute(__gnu__::__stdcall__) && !defined(__WINE__)) && defined(_MSC_VER)
	__stdcall
#endif
	VirtualQuery(void const *, memory_basic_information *, ::std::size_t) noexcept
#if defined(__clang__) || defined(__GNUC__)
#if SIZE_MAX <= UINT_LEAST32_MAX && (defined(__x86__) || defined(_M_IX86) || defined(__i386__))
#if !defined(__clang__)
	__asm__("VirtualQuery@12")
#else
	__asm__("_VirtualQuery@12")
#endif
#else
	__asm__("VirtualQuery")
#endif
#endif
		;

} // namespace win32

namespace details
{
#if __has_cpp_attribute(__gnu__::__returns_nonnull__)
[[__gnu__::__returns_nonnull__]]
#endif
inline void *win32_heapalloc_handle_common_impl(void *heaphandle, ::std::size_t n, ::std::uint_least32_t flag) noexcept
{
	if (n == 0)
	{
		n = 1;
	}
	auto p{::fast_io::win32::HeapAlloc(heaphandle, flag, n)};
	if (p == nullptr)
	{
		::fast_io::fast_terminate();
	}
	return p;
}

#if __has_cpp_attribute(__gnu__::__returns_nonnull__)
[[__gnu__::__returns_nonnull__]]
#endif
inline void *win32_heaprealloc_handle_common_impl(void *heaphandle, void *addr, ::std::size_t n, ::std::uint_least32_t flag) noexcept
{
	if (n == 0)
	{
		n = 1;
	}
	if (addr == nullptr)
#if __has_cpp_attribute(unlikely)
		[[unlikely]]
#endif
	{
		return win32_heapalloc_handle_common_impl(heaphandle, n, flag);
	}
	auto p{::fast_io::win32::HeapReAlloc(heaphandle, flag, addr, n)};
	if (p == nullptr)
	{
		::fast_io::fast_terminate();
	}
	return p;
}

#if __has_cpp_attribute(__gnu__::__always_inline__)
[[__gnu__::__always_inline__]]
#elif __has_cpp_attribute(msvc::forceinline)
[[msvc::forceinline]]
#endif
[[nodiscard]]
#if __has_cpp_attribute(__gnu__::__artificial__)
[[__gnu__::__artificial__]]
#endif
#if __has_cpp_attribute(__gnu__::__const__)
[[__gnu__::__const__]]
#endif
inline void *win32_get_process_heap() noexcept
{
	constexpr bool intrinsicssupported{
#if (defined(__GNUC__) || defined(__clang__) || defined(_MSC_VER)) &&                     \
	(defined(__i386__) || defined(__x86_64__) || defined(_M_IX86) || defined(_M_AMD64) || \
	 defined(__aarch64__) || defined(__arm64ec__) || defined(_M_ARM64) || defined(_M_ARM64EC))
		true
#endif
	};
	if constexpr (intrinsicssupported)
	{
		return ::fast_io::win32::nt::rtl_get_process_heap();
	}
	else
	{
		return ::fast_io::win32::GetProcessHeap();
	}
}

#if __has_cpp_attribute(__gnu__::__returns_nonnull__)
[[__gnu__::__returns_nonnull__]]
#endif
inline void *win32_heapalloc_common_impl(::std::size_t n, ::std::uint_least32_t flag) noexcept
{
	return ::fast_io::details::win32_heapalloc_handle_common_impl(::fast_io::details::win32_get_process_heap(), n, flag);
}

#if __has_cpp_attribute(__gnu__::__returns_nonnull__)
[[__gnu__::__returns_nonnull__]]
#endif
inline void *win32_heaprealloc_common_impl(void *addr, ::std::size_t n, ::std::uint_least32_t flag) noexcept
{
	return ::fast_io::details::win32_heaprealloc_handle_common_impl(::fast_io::details::win32_get_process_heap(), addr, n, flag);
}

inline ::fast_io::allocation_least_result win32_heapalloc_least_common_impl(::std::size_t n, ::std::uint_least32_t flag) noexcept
{
	auto processheap{::fast_io::details::win32_get_process_heap()};
	auto ptr{::fast_io::details::win32_heapalloc_handle_common_impl(processheap, n, flag)};
	return {ptr, ::fast_io::win32::HeapSize(processheap, 0, ptr)};
}

inline ::fast_io::allocation_least_result win32_heaprealloc_least_common_impl(void *addr, ::std::size_t n, ::std::uint_least32_t flag) noexcept
{
	auto processheap{::fast_io::details::win32_get_process_heap()};
	auto ptr{::fast_io::details::win32_heaprealloc_handle_common_impl(processheap, addr, n, flag)};
	return {ptr, ::fast_io::win32::HeapSize(processheap, 0, ptr)};
}

} // namespace details

class win32_heapalloc_allocator
{
public:
#if __has_cpp_attribute(__gnu__::__malloc__)
	[[__gnu__::__malloc__]]
#endif
	static inline void *allocate(::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heapalloc_common_impl(n, 0u);
	}

#if __has_cpp_attribute(__gnu__::__malloc__)
	[[__gnu__::__malloc__]]
#endif
	static inline void *allocate_zero(::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heapalloc_common_impl(n, 0x00000008u);
	}
	static inline void *reallocate(void *addr, ::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heaprealloc_common_impl(addr, n, 0u);
	}
	static inline void *reallocate_zero(void *addr, ::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heaprealloc_common_impl(addr, n, 0x00000008u);
	}
	static inline void deallocate(void *addr) noexcept
	{
		if (addr == nullptr)
		{
			return;
		}
		::fast_io::win32::HeapFree(::fast_io::details::win32_get_process_heap(), 0u, addr);
	}
#if 0
	static inline ::fast_io::allocation_least_result allocate_at_least(::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heapalloc_least_common_impl(n, 0u);
	}
	static inline ::fast_io::allocation_least_result allocate_zero_at_least(::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heapalloc_least_common_impl(n, 0x00000008u);
	}
	static inline ::fast_io::allocation_least_result reallocate_at_least(void *addr, ::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heaprealloc_least_common_impl(addr, n, 0u);
	}
	static inline ::fast_io::allocation_least_result reallocate_zero_at_least(void *addr, ::std::size_t n) noexcept
	{
		return ::fast_io::details::win32_heaprealloc_least_common_impl(addr, n, 0x00000008u);
	}
#endif
};

} // namespace fast_io
