#pragma once

#include <cstddef>
#include <type_traits>
#include <concepts>
#include <string>
#include <string_view>

#include "../../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

#include "../pin/pin_view.h"
#include "type.h"

namespace phy_engine::model {

template <typename T>
struct model_reserve_type_t {
	explicit constexpr model_reserve_type_t() noexcept = default;
};

template <typename T>
inline constexpr model_reserve_type_t<T> model_reserve_type{};

namespace defines {
template <typename T>
concept can_prepare_ac = requires(T&& t) {
							 { prepare_ac_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_prepare_dc = requires(T&& t) {
							 { prepare_dc_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_prepare_tr = requires(T&& t) {
							 { prepare_tr_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_prepare_op = requires(T&& t) {
							 { prepare_op_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_prepare_trop = requires(T&& t) {
							   { prepare_trop_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_iterate_ac = requires(T&& t) {
							 { iterate_ac_define(model_reserve_type<T>, t, double{}) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_iterate_dc = requires(T&& t) {
							 { iterate_dc_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_iterate_tr = requires(T&& t) {
							 { iterate_tr_define(model_reserve_type<T>, t, double{}) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_iterate_op = requires(T&& t) {
							 { iterate_op_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_iterate_trop = requires(T&& t) {
							   { iterate_trop_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
						   };

}

template <typename T>
concept model = requires(T&& t) {
					requires ::std::same_as<::std::remove_cvref_t<decltype(T::model_name)>, ::std::u8string_view>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(T::type)>, ::phy_engine::model::model_type>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.name)>, ::std::u8string>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.describe)>, ::std::u8string>;

					requires ::std::same_as<::std::remove_cvref_t<decltype(t.identifying)>, ::std::size_t>; // intertype independence
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.nodes)>, ::fast_io::vector<::std::size_t>>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.branchs)>, ::fast_io::vector<::std::size_t>>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.num_termls)>, ::std::size_t>;

					{ get_pin_view(model_reserve_type<T>, t) } -> ::std::same_as<::phy_engine::model::pin_view>;
				};


}