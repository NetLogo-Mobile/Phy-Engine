#pragma once
#include <cstddef>
#include <type_traits>

#include "../phy_engine_utils/fast_io/fast_io_core.h"
#include "../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

#include "../phy_engine_model/model_refs/base.h"

namespace phy_engine::model {
namespace details {

struct netlist_block {
	using Alloc = ::fast_io::native_typed_global_allocator<::phy_engine::model::model_base>;
	static constexpr ::std::size_t chunk_size{4096};

	constexpr netlist_block() noexcept {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			begin = new ::phy_engine::model::model_base[chunk_size];
		} else
#endif
		{
			begin = Alloc::allocate(chunk_size);
		}
		curr = begin;
	}

	constexpr netlist_block(netlist_block const &other) noexcept {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
		if consteval
#else
		if (__builtin_is_constant_evaluated())
#endif
		{
			begin = new ::phy_engine::model::model_base[chunk_size];
		} else
#endif
		{
			begin = Alloc::allocate(chunk_size);
		}
		auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
		curr = begin + size;
		num_of_null_model = other.num_of_null_model;
		for (::std::size_t i{}; i < size; i++) {
			new (begin + i)::phy_engine::model::model_base{other.begin[i]};
		}
	}

	constexpr netlist_block &operator=(netlist_block const &other) noexcept {
		if (__builtin_addressof(other) == this) {
			return *this;
		}
		clear();
		auto const size{static_cast<::std::size_t>(other.curr - other.begin)};
		curr = begin + size;
		num_of_null_model = other.num_of_null_model;
		for (::std::size_t i{}; i < size; i++) {
			new (begin + i)::phy_engine::model::model_base{other.begin[i]};
		}
		return *this;
	}

	constexpr netlist_block(netlist_block &&other) noexcept {
		begin = other.begin;
		curr = other.curr;
		num_of_null_model = other.num_of_null_model;
		other.begin = nullptr;
		other.curr = nullptr;
		other.num_of_null_model = ::std::size_t{};
	}

	constexpr netlist_block &operator=(netlist_block &&other) noexcept {
		if (__builtin_addressof(other) == this) {
			return *this;
		}
		delete_memory();
		begin = other.begin;
		curr = other.curr;
		num_of_null_model = other.num_of_null_model;
		other.begin = nullptr;
		other.curr = nullptr;
		other.num_of_null_model = ::std::size_t{};
		return *this;
	}

	constexpr ~netlist_block() noexcept {
		delete_memory();
	}

	constexpr void clear() noexcept {
		for (::phy_engine::model::model_base *b{begin}; b != curr; b++) {
			b->~model_base();
		}
		curr = begin;
		num_of_null_model = ::std::size_t{};
	}

	constexpr void delete_memory() noexcept {
		clear();
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

	constexpr void new_memory() noexcept {
		if (begin == nullptr) {
#if (__cpp_if_consteval >= 202106L || __cpp_lib_is_constant_evaluated >= 201811L) && __cpp_constexpr_dynamic_alloc >= 201907L
#if __cpp_if_consteval >= 202106L
			if consteval
#else
			if (__builtin_is_constant_evaluated())
#endif
			{
				begin = new ::phy_engine::model::model_base[chunk_size];
			} else
#endif
			{
				begin = Alloc::allocate(chunk_size);
			}
			curr = begin;
		}
	}

	constexpr ::std::size_t size() noexcept {
		return static_cast<::std::size_t>(curr - begin);
	}

	constexpr ::std::size_t get_num_of_model() noexcept {
		return static_cast<::std::size_t>(curr - begin) - num_of_null_model;
	}

	::phy_engine::model::model_base *begin{};
	::phy_engine::model::model_base *curr{};
	::std::size_t num_of_null_model{};
};
}  

struct netlist {
	using Alloc = ::fast_io::native_global_allocator;
	::fast_io::vector<details::netlist_block> netlist_memory{};
};

template <bool check = false>
inline constexpr ::std::size_t get_num_of_model(netlist &nl) noexcept {
	if constexpr (check) {
		::std::size_t res{};
		for (auto &i : nl.netlist_memory) {
			for (::phy_engine::model::model_base *b{i.begin}; b != i.curr; b++) {
				if (b->type != ::phy_engine::model::model_type::null) [[likely]] {
					res++;
				}
			}
		}
		return res;
	} else {
		::std::size_t res{};
		for (auto &i : nl.netlist_memory) {
			res += i.get_num_of_model();
		}
		return res;
	}
}

struct model_pos {
	::phy_engine::model::model_base *mod{};
	::std::size_t vec_pos{};
	::std::size_t chunk_pos{};
};

template <::phy_engine::model::model mod>
inline constexpr model_pos add_model(netlist &nl, mod &&m) noexcept {
	if (nl.netlist_memory.empty()) [[unlikely]] {
		auto &nlb{nl.netlist_memory.emplace_back()};
		new (nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
		nlb.curr++;
		return {nlb.curr, 0, 0};
	} else {
		auto &nlb{nl.netlist_memory.back()};
		if (nlb.curr == nlb.begin + nlb.chunk_size) [[unlikely]] {
			auto &new_nlb{nl.netlist_memory.emplace_back()};
			new (new_nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
			nlb.curr++;
			return {new_nlb.curr, nl.netlist_memory.size() - 1, 0};
		} else {
			new (nlb.curr)::phy_engine::model::model_base{::std::forward<mod>(m)};
			nlb.curr++;
			return {nlb.curr, nl.netlist_memory.size() - 1, nlb.size() - 1};
		}
	}
}

inline constexpr bool delete_model(netlist &nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept {
	if (vec_pos >= nl.netlist_memory.size())
		return false;
	auto &nlb{nl.netlist_memory[vec_pos]};
	if (chunk_pos >= nlb.size())
		return false;

	if (auto i = nlb.begin + chunk_pos; i == nlb.curr) {
		if (i->type == ::phy_engine::model::model_type::null) {
			nlb.num_of_null_model--;
		}
		i->~model_base();
		nlb.curr--;
	} else {
		if (i->type != ::phy_engine::model::model_type::null) {
			nlb.num_of_null_model++;
			i->clear();
		}
	}
	// to do (also delete wire)
	return true;
}

inline constexpr bool delete_model(netlist &nl, model_pos const &pos) noexcept {
	return delete_model(nl, pos.vec_pos, pos.chunk_pos);
}

inline constexpr bool delete_model(netlist &nl, model_pos &&pos) noexcept {
	return delete_model(nl, pos.vec_pos, pos.chunk_pos);
}

inline constexpr ::phy_engine::model::model_base *get_model(netlist &nl, ::std::size_t vec_pos, ::std::size_t chunk_pos) noexcept {
	if (vec_pos >= nl.netlist_memory.size())
		return nullptr;
	auto &nlb{nl.netlist_memory[vec_pos]};
	if (chunk_pos >= nlb.size()) 
		return nullptr;
	return nlb.begin + chunk_pos;
}

inline constexpr bool add_wire(netlist &nl) noexcept {
	return {};
}

inline constexpr bool delete_wire(netlist &nl) noexcept {
	return {};
}

inline constexpr bool add_netlist(netlist &nl, netlist const &nl_add) noexcept {
	return {};
}

inline constexpr netlist get_netlist(netlist &nl, ::std::size_t *pos_view, ::std::size_t size) noexcept {
	return {};
}

inline constexpr void optimize_memory(netlist &nl) noexcept {
}

}