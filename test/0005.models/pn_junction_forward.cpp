#include <cmath>
#include <cstddef>
#include <limits>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/non-linear/PN_junction.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    constexpr double kKelvin{-273.15};
    constexpr double qElement{1.6021765314e-19};
    constexpr double kBoltzmann{1.380650524e-23};

    double thermal_voltage(double temp_c) noexcept { return kBoltzmann * (temp_c - kKelvin) / qElement; }

    // Solve (Vs - Vd)/R = Is*(exp(Vd/(N*Ut)) - 1) for Vd in [0, Vs].
    double solve_diode_drop(double Vs, double R, double Is, double N, double temp_c)
    {
        double const Ut = thermal_voltage(temp_c);
        double const Ute = N * Ut;

        auto f = [&](double Vd) -> double
        {
            double const Id = Is * (::std::exp(Vd / Ute) - 1.0);
            double const Ir = (Vs - Vd) / R;
            return Ir - Id;
        };

        double lo = 0.0;
        double hi = Vs;
        double flo = f(lo);
        double fhi = f(hi);
        if(!(flo > 0.0 && fhi < 0.0))
        {
            ::fast_io::io::perr("pn_junction_forward: invalid bracket\n");
            return ::std::numeric_limits<double>::quiet_NaN();
        }

        for(int iter = 0; iter < 200; ++iter)
        {
            double const mid = 0.5 * (lo + hi);
            double const fmid = f(mid);
            if(::std::abs(fmid) < 1e-15 || (hi - lo) < 1e-12) { return mid; }
            if(fmid > 0.0)
            {
                lo = mid;
                flo = fmid;
            }
            else
            {
                hi = mid;
                fhi = fmid;
            }
        }
        return 0.5 * (lo + hi);
    }
}  // namespace

int main()
{
    constexpr double Vs = 5.0;
    constexpr double R = 1000.0;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};

    auto [vsrc, vsrc_pos]{add_model(nl, ::phy_engine::model::VDC{.V = Vs})};
    auto [r, r_pos]{add_model(nl, ::phy_engine::model::resistance{.r = R})};

    ::phy_engine::model::PN_junction d{};
    d.Is = 1e-14;
    d.N = 1.0;
    d.Isr = 0.0;
    d.Temp = 27.0;
    auto [pn, pn_pos]{add_model(nl, ::std::move(d))};

    auto& node_vs{create_node(nl)};
    auto& node_d{create_node(nl)};
    auto& gnd{nl.ground_node};

    add_to_node(nl, *vsrc, 0, node_vs);
    add_to_node(nl, *vsrc, 1, gnd);

    add_to_node(nl, *r, 0, node_vs);
    add_to_node(nl, *r, 1, node_d);

    // Diode: A=node_d, B=gnd
    add_to_node(nl, *pn, 0, node_d);
    add_to_node(nl, *pn, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("pn_junction_forward: analyze failed\n");
        return 1;
    }

    double const Vd_sim{node_d.node_information.an.voltage.real()};
    double const Vd_exp{solve_diode_drop(Vs, R, 1e-14, 1.0, 27.0)};
    if(!::std::isfinite(Vd_exp)) { return 1; }

    // The expected solution is ~0.57V for these parameters; allow some solver tolerance.
    if(!(::std::abs(Vd_sim - Vd_exp) < 5e-3))
    {
        ::fast_io::io::perr("pn_junction_forward: diode drop mismatch, Vd_sim=", Vd_sim, " Vd_exp=", Vd_exp, "\n");
        return 1;
    }

    // Current consistency: I = (Vs - Vd)/R
    double const I_sim{(Vs - Vd_sim) / R};
    double const Ut = thermal_voltage(27.0);
    double const I_exp{1e-14 * (::std::exp(Vd_sim / Ut) - 1.0)};
    if(!(::std::abs(I_sim - I_exp) < 5e-6))
    {
        ::fast_io::io::perr("pn_junction_forward: current mismatch, I_sim=", I_sim, " I_exp=", I_exp, "\n");
        return 1;
    }

    return 0;
}
