#pragma once
#include "concept.h"

namespace phy_engine::model
{

    template <::phy_engine::model::model mod>
    inline constexpr bool init_model(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_init<mod>)
        {
            return init_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool prepare_ac(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_prepare_ac<mod>)
        {
            return prepare_ac_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool prepare_dc(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_prepare_dc<mod>)
        {
            return prepare_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool prepare_tr(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_prepare_tr<mod>)
        {
            return prepare_tr_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool prepare_op(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_prepare_op<mod>)
        {
            return prepare_op_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else if constexpr(::phy_engine::model::defines::can_prepare_dc<mod>)
        {
            return prepare_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool prepare_trop(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_prepare_trop<mod>)
        {
            return prepare_trop_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else if constexpr(::phy_engine::model::defines::can_prepare_tr<mod>)
        {
            return prepare_tr_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool iterate_ac(mod&& m, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double omega) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_iterate_ac<mod>)
        {
            return iterate_ac_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna, omega);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
        {
            return iterate_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_mna<mod>) { return true; }
        else { return false; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool iterate_dc(mod&& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
        {
            return iterate_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_mna<mod>) { return true; }
        else { return false; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool iterate_tr(mod&& m, ::phy_engine::MNA::MNA& mna, [[maybe_unused]] double tTime) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_iterate_tr<mod>)
        {
            return iterate_tr_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna, tTime);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
        {
            return iterate_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_mna<mod>) { return true; }
        else { return false; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool iterate_op(mod&& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_iterate_op<mod>)
        {
            return iterate_op_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
        {
            return iterate_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_mna<mod>) { return true; }
        else { return false; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool iterate_trop(mod&& m, ::phy_engine::MNA::MNA& mna) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_iterate_trop<mod>)
        {
            return iterate_trop_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_tr<mod>)
        {
            return iterate_tr_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna, 0.0);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_dc<mod>)
        {
            return iterate_dc_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), mna);
        }
        else if constexpr(::phy_engine::model::defines::can_iterate_mna<mod>) { return true; }
        else { return false; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool save_op(mod&& m) noexcept
    {
        // not a non-linear device and no need to store operating point
        if constexpr(::std::remove_cvref_t<mod>::device_type == ::phy_engine::model::model_device_type::non_linear)
        {
            if constexpr(::phy_engine::model::defines::can_save_op<mod>)
            {
                return save_op_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
            }
            else { return true; }
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool load_temperature(mod&& m, [[maybe_unused]] double temp) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_load_temperature<mod>)
        {
            return load_temperature_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), temp);
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool step_changed_tr(mod&& m, [[maybe_unused]] double tTemp, [[maybe_unused]] double nstep) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_step_changed_tr<mod>)
        {
            return step_changed_tr_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), tTemp, nstep);
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool adapt_step(mod&& m, [[maybe_unused]] double& step) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_adapt_step<mod>)
        {
            return adapt_step_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), step);
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool check_convergence(mod&& m) noexcept
    {
        // no model-specific checks for convergence
        if constexpr(::phy_engine::model::defines::can_check_convergence<mod>)
        {
            return check_convergence_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return true; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr bool set_attribute(mod&& m, ::std::size_t index, ::phy_engine::model::variant vi) noexcept
    {
        // no model-specific checks for convergence
        if constexpr(::phy_engine::model::defines::has_set_attribute<mod>)
        {
            return set_attribute_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), index, vi);
        }
        else { return false; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr ::phy_engine::model::variant get_attribute(mod&& m, ::std::size_t index) noexcept
    {
        if constexpr(::phy_engine::model::defines::has_get_attribute<mod>)
        {
            return get_attribute_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), index);
        }
        else { return {}; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr ::fast_io::u8string_view get_attribute_name(mod&& m, ::std::size_t index) noexcept
    {
        if constexpr(::phy_engine::model::defines::has_get_attribute_name<mod>)
        {
            return get_attribute_name_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m), index);
        }
        else { return {}; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr ::phy_engine::model::pin_view generate_pin_view(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_generate_pin_view<mod>)
        {
            return generate_pin_view_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return {}; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr ::phy_engine::model::branch_view generate_branch_view(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_generate_branch_view<mod>)
        {
            return generate_branch_view_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return {}; }
    }

    template <::phy_engine::model::model mod>
    inline constexpr ::phy_engine::model::node_view generate_internal_node_view(mod&& m) noexcept
    {
        if constexpr(::phy_engine::model::defines::can_generate_internal_node_view<mod>)
        {
            return generate_internal_node_define(::phy_engine::model::model_reserve_type<::std::remove_cvref_t<mod>>, ::std::forward<mod>(m));
        }
        else { return {}; }
    }

}  // namespace phy_engine::model
