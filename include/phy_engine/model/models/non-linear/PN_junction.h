#pragma once
#include <fast_io/fast_io_dsal/string_view.h>
#include "../../model_refs/base.h"

namespace phy_engine::model
{
    struct PN_junction
    {
        inline static constexpr ::fast_io::u8string_view model_name{u8"PN Junction"};

        inline static constexpr ::phy_engine::model::model_device_type device_type{::phy_engine::model::model_device_type::non_linear};
        inline static constexpr ::fast_io::u8string_view identification_name{u8"PN"};

        double Is{1e-14};
        double N{1.0};
        double Isr{0.0};
        double Nr{2.0};
        double Temp{27.0};
        double Ibv{1e-3};
        double Bv{40.0};
        bool Bv_set{true};
        double Area{1.0};

        ::phy_engine::model::pin pins[2]{{{u8"A"}}, {{u8"B"}}};

        // private:
        double Ut{};
        double Uth{};
        double Ud_last{};
        double geq{};
        double Ieq{};
    };

    static_assert(::phy_engine::model::model<PN_junction>);

    inline constexpr double vlimit(PN_junction const& pn, double Ud) noexcept
    {
        bool flag{};
        double Ud_0, Ud_1, Ud_f;
        double const Ute{pn.N * pn.Ut};

        /* Fix the very small voltage */
        if(pn.Bv_set && Ud < ::std::min(0.0, -pn.Bv + 10.0 * Ute))
        {
            Ud_0 = -(Ud + pn.Bv);
            Ud_1 = -(pn.Ud_last + pn.Bv);
            flag = true;
        }
        else
        {
            Ud_0 = Ud;
            Ud_1 = pn.Ud_last;
        }

        /* This code comes from SPICE3F5. Authors: 1985 Thomas L. Quarles, 2000 Alan Gillespie. */
        if(Ud_0 > pn.Uth && ::std::abs(Ud_0 - Ud_1) > 2.0 * Ute)
        {
            if(Ud_1 > 0)
            {
                double arg{(Ud_0 - Ud_1) / Ute};
                if(arg > 0.0) { Ud_f = Ud_1 + Ute * (2.0 + ::std::log(arg - 2.0)); }
                else
                {
                    Ud_f = Ud_1 - Ute * (2.0 + ::std::log(2.0 - arg));
                }
            }
            else
            {
                Ud_f = Ute * ::std::log(Ud_0 / Ute);
            }
        }
        else
        {
            Ud_f = Ud_0; /* Near zero */
            if(Ud_0 < 0.0)
            {
                double arg = Ud_1 > 0.0 ? -1.0 - Ud_1 : 2.0 * Ud_1 - 1;
                if(Ud_0 < arg) { Ud_f = arg; }
            }
        }

        if(flag) { return -(Ud_f + pn.Bv); }
        else
        {
            return Ud_f;
        }
    }

    inline constexpr bool
        set_attribute_define(::phy_engine::model::model_reserve_type_t<PN_junction>, PN_junction& pn, ::std::size_t n, ::phy_engine::model::variant vi) noexcept
    {
        switch(n)
        {
            case 0:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Is = vi.d;
                return true;
            }
            case 1:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.N = vi.d;
                return true;
            }
            case 2:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Isr = vi.d;
                return true;
            }
            case 3:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Nr = vi.d;
                return true;
            }
            case 4:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Temp = vi.d;
                return true;
            }
            case 5:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Ibv = vi.d;
                return true;
            }
            case 6:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Bv = vi.d;
                return true;
            }
            case 7:
            {
                if(vi.type != ::phy_engine::model::variant_type::boolean) [[unlikely]] { return false; }
                pn.Bv_set = vi.boolean;
                return true;
            }
            case 8:
            {
                if(vi.type != ::phy_engine::model::variant_type::d) [[unlikely]] { return false; }
                pn.Area = vi.d;
                return true;
            }
            default:
            {
                return false;
            }
        }
        return false;
    }

    static_assert(::phy_engine::model::defines::has_set_attribute<PN_junction>);

    inline constexpr ::phy_engine::model::variant
        get_attribute_define(::phy_engine::model::model_reserve_type_t<PN_junction>, PN_junction const& pn, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return {.d{pn.Is}, .type{::phy_engine::model::variant_type::d}};
            }
            case 1:
            {
                return {.d{pn.N}, .type{::phy_engine::model::variant_type::d}};
            }
            case 2:
            {
                return {.d{pn.Isr}, .type{::phy_engine::model::variant_type::d}};
            }
            case 3:
            {
                return {.d{pn.Nr}, .type{::phy_engine::model::variant_type::d}};
            }
            case 4:
            {
                return {.d{pn.Temp}, .type{::phy_engine::model::variant_type::d}};
            }
            case 5:
            {
                return {.d{pn.Ibv}, .type{::phy_engine::model::variant_type::d}};
            }
            case 6:
            {
                return {.d{pn.Bv}, .type{::phy_engine::model::variant_type::d}};
            }
            case 7:
            {
                return {.boolean{pn.Bv_set}, .type{::phy_engine::model::variant_type::boolean}};
            }
            case 8:
            {
                return {.d{pn.Area}, .type{::phy_engine::model::variant_type::d}};
            }
            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute<PN_junction>);

    inline constexpr ::fast_io::u8string_view get_attribute_name_define(::phy_engine::model::model_reserve_type_t<PN_junction>, ::std::size_t n) noexcept
    {
        switch(n)
        {
            case 0:
            {
                return u8"Is";
            }
            case 1:
            {
                return u8"N";
            }
            case 2:
            {
                return u8"Isr";
            }
            case 3:
            {
                return u8"Nr";
            }
            case 4:
            {
                return u8"Temp";
            }
            case 5:
            {
                return u8"Ibv";
            }
            case 6:
            {
                return u8"Bv";
            }
            case 7:
            {
                return u8"Bv_set";
            }
            case 8:
            {
                return u8"Area";
            }

            default:
            {
                return {};
            }
        }
        return {};
    }

    static_assert(::phy_engine::model::defines::has_get_attribute_name<PN_junction>);

    inline bool prepare_foundation_define(::phy_engine::model::model_reserve_type_t<PN_junction>, PN_junction& pn) noexcept
    {
        pn.Is *= pn.Area;
        pn.Isr *= pn.Area;

        /* Temperature voltage equivalent */
        constexpr double kKelvin{-273.15};
        constexpr double qElement{1.6021765314e-19};
        constexpr double kBoltzmann{1.380650524e-23};
        constexpr double sqrt2{1.4142135623730950488016887242096981};

        pn.Ut = kBoltzmann * (pn.Temp - kKelvin) / qElement;

        if(pn.Bv_set)
        {
            /*
             * Fit the breakdown voltage from Isr to Is
             *
             * The principle is shown as follows:
             * Let the equations of breakdown current about Is and Isr be equal, i.e.
             * \f[ -Ibv \cdot e^{\frac{-(Bv+Ud)}{N \cdot Ut}} = -Is \cdot e^{\frac{-(Bv'+Ud)}{N \cdot Ut}} \f]
             *
             * Take logarithms on both sides of the equation to get:
             * \f[ ln(\frac{Ibv}{Is})+\frac{-(Bv-Ud)}{N \cdot Ut} = \frac{-(Bv'-Ud)}{N \cdot Ut} $$ \f]
             *
             * Finally, Rewrite the above equation and get the following results:
             * \f[ Bv' = Bv - N \cdot Ut \cdot ln(\frac{Ibv}{Is}) $$ \f]
             */
            pn.Bv = pn.Bv - pn.N * pn.Ut * ::std::log(pn.Ibv / pn.Is);
        }

        /*
         * Compute critical voltage
         *
         * The principle is shown as follows:
         *
         * Critical voltage Uth is defined as the point in V/I curve which makes the curve radius minimum.
         * The curve radius of V/I curve can be written as:
         * \f[ R=\left|\frac{{\left(1+{(\frac{\partial I}{\partial U})}^2\right)}^{\frac{3}{2}}}{\frac{\partial^2 I}{\partial U^2}}\right| \f]
         *
         * Finding the minimum by making the derivative zero:
         * \f[ \frac{\partial R}{\partial U} = 0 \f]
         *
         * Finally, we can find out the Uth which meets the above condition:
         * \f[ Uth = N \cdot Ut \cdot ln(\frac{N \cdot Ut}{\sqrt{2} \cdot Is}) \f]
         */

        pn.Uth = pn.N * pn.Ut * ::std::log(pn.N * pn.Ut / (sqrt2 * pn.Is));

        // last Voltage
        auto const node_0{pn.pins[0].nodes};
        auto const node_1{pn.pins[1].nodes};

        if(node_0 && node_1) [[likely]] { pn.Ud_last = node_0->node_information.an.voltage.real() - node_1->node_information.an.voltage.real(); }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_prepare_foundation<PN_junction>);

    inline constexpr bool iterate_dc_define(::phy_engine::model::model_reserve_type_t<PN_junction>, PN_junction& pn, ::phy_engine::MNA::MNA& mna) noexcept
    {
        auto const node_0{pn.pins[0].nodes};
        auto const node_1{pn.pins[1].nodes};
        if(node_0 && node_1) [[likely]]
        {
            double Ud{node_0->node_information.an.voltage.real() - node_1->node_information.an.voltage.real()};

            Ud = vlimit(pn, Ud);
            pn.Ud_last = Ud;

            double Id;
            double const Ute{pn.N * pn.Ut};
            double const Uter{pn.Nr * pn.Ut};

            if(pn.Bv_set && Ud < -pn.Bv) /* breakdown */
            {
                double e{::std::exp(-(pn.Bv + Ud) / Ute)};
                Id = -pn.Is * e;
                pn.geq = pn.Is * e / Ute;
            }
            else
            {
                double e{::std::exp(Ud / Ute)};
                pn.geq = pn.Is * e / Ute;
                Id = pn.Is * (e - 1.0);

                /* recombination current */
                e = ::std::exp(Ud / Uter);
                pn.geq += pn.Isr * e / Uter;
                Id += pn.Isr * (e - 1.0);
            }

            pn.Ieq = Id - Ud * pn.geq;

            mna.G_ref(node_0->node_index, node_0->node_index) += pn.geq;
            mna.G_ref(node_0->node_index, node_1->node_index) -= pn.geq;
            mna.G_ref(node_1->node_index, node_0->node_index) -= pn.geq;
            mna.G_ref(node_1->node_index, node_1->node_index) += pn.geq;
            mna.I_ref(node_0->node_index) -= pn.Ieq;
            mna.I_ref(node_1->node_index) += pn.Ieq;
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_dc<PN_junction>);

    inline constexpr bool iterate_ac_define(::phy_engine::model::model_reserve_type_t<PN_junction>,
                                            PN_junction& pn,
                                            ::phy_engine::MNA::MNA& mna,
                                            [[maybe_unused]] double omega) noexcept
    {
        auto const node_0{pn.pins[0].nodes};
        auto const node_1{pn.pins[1].nodes};
        if(node_0 && node_1) [[likely]]
        {

            mna.G_ref(node_0->node_index, node_0->node_index) += pn.geq;
            mna.G_ref(node_0->node_index, node_1->node_index) -= pn.geq;
            mna.G_ref(node_1->node_index, node_0->node_index) -= pn.geq;
            mna.G_ref(node_1->node_index, node_1->node_index) += pn.geq;
            // In small-signal AC analysis, only the incremental conductance is stamped.
            // The equivalent current source from DC linearization (Ieq) must NOT be injected.
        }

        return true;
    }

    static_assert(::phy_engine::model::defines::can_iterate_ac<PN_junction>);

    inline constexpr bool step_changed_tr_define(::phy_engine::model::model_reserve_type_t<PN_junction>,
                                                 PN_junction& pn,
                                                 [[maybe_unused]] double nlaststep,
                                                 [[maybe_unused]] double nstep) noexcept
    {
        auto const node_0{pn.pins[0].nodes};
        auto const node_1{pn.pins[1].nodes};

        if(node_0 && node_1) [[likely]] { pn.Ud_last = node_0->node_information.an.voltage.real() - node_1->node_information.an.voltage.real(); }

        return true;
    }

    inline constexpr ::phy_engine::model::pin_view generate_pin_view_define(::phy_engine::model::model_reserve_type_t<PN_junction>, PN_junction& pn) noexcept
    {
        return {pn.pins, 2};
    }

    static_assert(::phy_engine::model::defines::can_generate_pin_view<PN_junction>);

}  // namespace phy_engine::model
