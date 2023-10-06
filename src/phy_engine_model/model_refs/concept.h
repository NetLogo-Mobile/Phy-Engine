#pragma once

#include <cstddef>
#include <type_traits>
#include <concepts>
#include <string>
#include <string_view>

#include "../../phy_engine_utils/fast_io/fast_io_dsal/vector.h"

#include "type.h"
#include "../pin/pin_view.h"

namespace phy_engine::model {

template <typename T>
struct model_reserve_type_t {
	explicit constexpr model_reserve_type_t() noexcept = default;
};

template <typename T>
inline constexpr model_reserve_type_t<T> model_reserve_type{};

namespace defines {
template <typename T>
concept can_init = requires(T&& t) {
					   { init_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
				   };

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

template <typename T>
concept can_save_op = requires(T&& t) {
						  { save_op_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
					  };

template <typename T>
concept can_load_temperature = requires(T&& t) {
								   { load_temperature_define(model_reserve_type<T>, t, double{}) } -> ::std::same_as<bool>;
							   };

template <typename T>
concept can_step_changed_tr = requires(T&& t) {
								  { step_changed_tr_define(model_reserve_type<T>, t, double{}, double{}) } -> ::std::same_as<bool>;
							   };

template <typename T>
concept can_adapt_step = requires(T&& t, double step) {
							 { adapt_step_define(model_reserve_type<T>, t, step) } -> ::std::same_as<bool>;
						 };

template <typename T>
concept can_check_convergence = requires(T&& t) {
									{ check_convergence_define(model_reserve_type<T>, t) } -> ::std::same_as<bool>;
								};
template <typename T>
concept can_query_status = requires(T&& t) {
							   { query_status_define(model_reserve_type<T>, t, ::std::size_t{}) } -> ::std::same_as<bool>;
						   };
}

template <typename T>
concept model = requires(T&& t) {
					requires ::std::same_as<::std::remove_cvref_t<decltype(T::model_name)>, ::std::u8string_view>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(T::type)>, ::phy_engine::model::model_type>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(T::device_type)>, ::phy_engine::model::model_device_type>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.name)>, ::std::u8string>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.describe)>, ::std::u8string>;

					requires ::std::same_as<::std::remove_cvref_t<decltype(T::identification_name)>, ::std::u8string_view>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.identification)>, ::std::size_t>;  // intertype independence

					requires ::std::same_as<::std::remove_cvref_t<decltype(t.nodes)>, ::fast_io::vector<::std::size_t>>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.branchs)>, ::fast_io::vector<::std::size_t>>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(t.num_termls)>, ::std::size_t>;

					requires ::std::same_as<::std::remove_cvref_t<decltype(T::pins)>, ::phy_engine::model::pin_view>;
				};


}