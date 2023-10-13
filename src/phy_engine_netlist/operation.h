#pragma once

#include "netlist.h"
#include "concept.h"

namespace phy_engine::model {
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

template <typename mod>
	requires(::phy_engine::model::model<mod> && ::phy_engine::model::defines::can_iterate_dc<mod>)
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

#if 0
template <typename mod>
	requires(::phy_engine::model::model<mod> && (::phy_engine::model::can_generate_netlist<mod> || ::phy_engine::model::can_generate_netlist_const_lvalue<mod>))
inline constexpr model_pos add_model(netlist &nl, mod &&m) noexcept {
	// Managed by circuit
	return {};
}
#endif 

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