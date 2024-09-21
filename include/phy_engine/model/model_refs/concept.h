#pragma once

#include <cstddef>
#include <type_traits>
#include <concepts>

#include <fast_io/fast_io_dsal/vector.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/string.h>

#include "type.h"
#include "../pin/pin_view.h"
#include "../node/node.h"
#include "../branch/branch_view.h"
#include "variant.h"
#include "../../circuits/MNA/mna.h"
#include "../../circuits/solver/integral_corrector_gear.h"

namespace phy_engine::model
{

    template <typename mod>
    struct model_reserve_type_t
    {
        static_assert(::std::is_same_v<::std::remove_cvref_t<mod>, mod>, "model_reserve_type_t: typename 'mod' cannot have refer and const attributes");
        explicit constexpr model_reserve_type_t() noexcept = default;
    };

    template <typename mod>
    inline constexpr model_reserve_type_t<mod> model_reserve_type{};

    namespace defines
    {
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
        concept can_prepare_tr = requires(mod&& t, ::phy_engine::solver::integral_corrector_gear& icg) {
            { prepare_tr_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, icg) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_prepare_op = requires(mod&& t) {
            { prepare_op_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_prepare_trop = requires(mod&& t, ::phy_engine::solver::integral_corrector_gear& icg) {
            { prepare_trop_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, icg) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_iterate_ac = requires(mod&& t, ::phy_engine::MNA::MNA& mna) {
            { iterate_ac_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, mna, double{}) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_iterate_dc = requires(mod&& t, ::phy_engine::MNA::MNA& mna) {
            { iterate_dc_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, mna) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_iterate_tr = requires(mod&& t, ::phy_engine::MNA::MNA& mna, ::phy_engine::solver::integral_corrector_gear& icg) {
            { iterate_tr_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, mna, double{}, icg) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_iterate_op = requires(mod&& t, ::phy_engine::MNA::MNA& mna) {
            { iterate_op_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, mna) } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept can_iterate_trop = requires(mod&& t, ::phy_engine::MNA::MNA& mna, ::phy_engine::solver::integral_corrector_gear& icg) {
            { iterate_trop_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, mna, icg) } -> ::std::same_as<bool>;
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

        template <typename mod>
        concept has_set_attribute = requires(mod&& t) {
            {
                set_attribute_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, ::std::size_t{}, ::phy_engine::model::variant{})
            } -> ::std::same_as<bool>;
        };

        template <typename mod>
        concept has_get_attribute = requires(mod&& t) {
            { get_attribute_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, ::std::size_t{}) } -> ::std::same_as<::phy_engine::model::variant>;
        };

        template <typename mod>
        concept has_get_attribute_name = requires(mod&& t) {
            { get_attribute_name_define(model_reserve_type<::std::remove_cvref_t<mod>>, t, ::std::size_t{}) } -> ::std::same_as<::fast_io::u8string_view>;
        };

        template <typename mod>
        concept has_attribute = has_set_attribute<mod> && has_get_attribute_name<mod> && has_get_attribute_name<mod>;

        template <typename mod>
        concept can_generate_pin_view = requires(mod&& t) {
            { generate_pin_view_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<::phy_engine::model::pin_view>;
        };

        template <typename mod>
        concept can_generate_branch_view = requires(mod&& t) {
            { generate_branch_view_define(model_reserve_type<::std::remove_cvref_t<mod>>, t) } -> ::std::same_as<::phy_engine::model::branch_view>;
        };

        template <typename mod>
        concept can_iterate_mna = can_iterate_ac<mod> || can_iterate_dc<mod> || can_iterate_op<mod> || can_iterate_tr<mod> || can_iterate_trop<mod>;

    }  // namespace defines

    template <typename mod>
    concept model = requires(mod&& t) {
        // constexpr static value
        requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::model_name)>, ::fast_io::u8string_view>;
        // requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::type)>, ::phy_engine::model::model_type>;
        requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::device_type)>, ::phy_engine::model::model_device_type>;
        requires ::std::same_as<::std::remove_cvref_t<decltype(::std::remove_cvref_t<mod>::identification_name)>, ::fast_io::u8string_view>;
    };

}  // namespace phy_engine::model
