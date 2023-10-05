#pragma once
#include <cstddef>

#include "../phy_engine_utils/fast_io/fast_io_core.h"
#include "../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

#include "../phy_engine_model/model_refs/base.h"

namespace phy_engine::model {
namespace details {
struct netlist_chunk {
	using Alloc = ::fast_io::native_typed_global_allocator<::phy_engine::model::module_base>;
	static constexpr ::std::size_t chunk_size{4096};

	constexpr netlist_chunk() noexcept {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			begin = new ::phy_engine::model::module_base[chunk_size];
		} else
#endif
		{
			begin = Alloc::allocate(chunk_size);
		}
	}

	constexpr netlist_chunk(const netlist_chunk &other) noexcept {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			begin = new ::phy_engine::model::module_base[chunk_size];
		} else
#endif
		{
			begin = Alloc::allocate(chunk_size);
		}
		auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
		curr = begin + size;
		for (::std::size_t i{}; i < size; i++) {
			begin[i] = other.begin[i];
		}
	}

	constexpr netlist_chunk &operator=(netlist_chunk const &other) noexcept {
		if (__builtin_addressof(other) == this) {
			return *this;
		}
		clean();
		auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
		curr = begin + size;
		for (::std::size_t i{}; i < size; i++) {
			begin[i] = other.begin[i];
		}
		return *this;
	}

	constexpr netlist_chunk(netlist_chunk &&other) noexcept {
		begin = other.begin;
		curr = other.curr;
		other.begin = nullptr;
		other.curr = nullptr;
	}

	constexpr netlist_chunk &operator=(netlist_chunk &&other) noexcept {
		if (__builtin_addressof(other) == this) {
			return *this;
		}
		delete_memory();
		begin = other.begin;
		curr = other.curr;
		other.begin = nullptr;
		other.curr = nullptr;
		return *this;
	}

	constexpr ~netlist_chunk() noexcept {
		delete_memory();
	}

	constexpr void clean() noexcept {
		for (::phy_engine::model::module_base *b{begin}; b != curr; b++) {
			b->~module_base();
		}
		curr = begin;
	}

	constexpr void delete_memory() noexcept {
		clean();
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			delete[] begin;
		} else
#endif
		{
			Alloc::deallocate(begin);
		}
		begin = nullptr;
		curr = nullptr;
	}

	::phy_engine::model::module_base *begin{};
	::phy_engine::model::module_base *curr{};
};
}  

struct netlist {
	using Alloc = ::fast_io::native_global_allocator;
	::fast_io::vector<details::netlist_chunk> netlist_memory{};
};
}