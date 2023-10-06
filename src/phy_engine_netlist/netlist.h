#pragma once
#include <cstddef>

#include "../phy_engine_utils/fast_io/fast_io_core.h"
#include "../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

#include "../phy_engine_model/model_refs/base.h"

namespace phy_engine::model {
namespace details {
struct netlist_block {
	using Alloc = ::fast_io::native_typed_global_allocator<::phy_engine::model::module_base>;
	static constexpr ::std::size_t chunk_size{4096};

	constexpr netlist_block() noexcept {
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

	constexpr netlist_block(const netlist_block &other) noexcept {
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

	constexpr netlist_block &operator=(netlist_block const &other) noexcept {
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

	constexpr netlist_block(netlist_block &&other) noexcept {
		begin = other.begin;
		curr = other.curr;
		other.begin = nullptr;
		other.curr = nullptr;
	}

	constexpr netlist_block &operator=(netlist_block &&other) noexcept {
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

	constexpr ~netlist_block() noexcept {
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
	::fast_io::vector<details::netlist_block> netlist_memory{};
};

inline constexpr ::std::size_t get_component_size() noexcept {
}

template <::phy_engine::model::model mod>
inline constexpr void add_component(netlist& nl, mod &&m) noexcept {
}

inline constexpr bool delete_component(netlist &nl, ::std::size_t pos) noexcept {
}

inline constexpr ::phy_engine::model::module_base *get_component(netlist &nl, ::std::size_t pos) noexcept {
}

inline constexpr bool add_wire(netlist &nl) noexcept {
}

inline constexpr bool delete_wire(netlist &nl) noexcept {
}

inline constexpr bool add_netlist(netlist &nl, netlist const& nl_add) noexcept {
}

inline constexpr netlist get_netlist(netlist &nl, ::std::size_t *pos_view, ::std::size_t size) noexcept {
}

inline constexpr void optimize_memory(netlist &nl) noexcept {
}

}