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

template <typename mod>
struct model_reserve_type_t {
	static_assert(::std::is_same_v<::std::remove_cvref_t<mod>, mod>, "model_reserve_type_t: typename 'mod' cannot have refer and const attributes");
	explicit constexpr model_reserve_type_t() noexcept = default;
};

template <typename mod>
inline constexpr model_reserve_type_t<mod> model_reserve_type{};

namespace defines {
template <typename mod>
concept can_init = requires(mod&& t) {
					   { init_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
				   };

template <typename mod>
concept can_prepare_ac = requires(mod&& t) {
							 { prepare_ac_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_prepare_dc = requires(mod&& t) {
							 { prepare_dc_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_prepare_tr = requires(mod&& t) {
							 { prepare_tr_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_prepare_op = requires(mod&& t) {
							 { prepare_op_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_prepare_trop = requires(mod&& t) {
							   { prepare_trop_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_iterate_ac = requires(mod&& t) {
							 { iterate_ac_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, double{}) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_iterate_dc = requires(mod&& t) {
							 { iterate_dc_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_iterate_tr = requires(mod&& t) {
							 { iterate_tr_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, double{}) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_iterate_op = requires(mod&& t) {
							 { iterate_op_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_iterate_trop = requires(mod&& t) {
							   { iterate_trop_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
						   };

template <typename mod>
concept can_save_op = requires(mod&& t) {
						  { save_op_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
					  };

template <typename mod>
concept can_load_temperature = requires(mod&& t) {
								   { load_temperature_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, double{}) } -> ::std::same_as<bool>;
							   };

template <typename mod>
concept can_step_changed_tr = requires(mod&& t) {
								  { step_changed_tr_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, double{}, double{}) } -> ::std::same_as<bool>;
							   };

template <typename mod>
concept can_adapt_step = requires(mod&& t, double step) {
							 { adapt_step_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, step) } -> ::std::same_as<bool>;
						 };

template <typename mod>
concept can_check_convergence = requires(mod&& t) {
									{ check_convergence_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
								};
template <typename mod>
concept can_query_status = requires(mod&& t) {
							   { query_status_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, ::std::size_t{}) } -> ::std::same_as<bool>;
						   };
}

template <typename mod>
concept model = requires(mod&& t) {
					requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::model_name)>, ::std::u8string_view>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::type)>, ::phy_engine::model::model_type>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::device_type)>, ::phy_engine::model::model_device_type>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::identification_name)>, ::std::u8string_view>;
					requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::pins)>, ::phy_engine::model::pin_view>;
				};


}