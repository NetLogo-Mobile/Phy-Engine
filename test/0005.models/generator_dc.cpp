#include <cmath>
#include <numbers>

#include <fast_io/fast_io.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/generator/pulse.h>
#include <phy_engine/model/models/generator/sawtooth.h>
#include <phy_engine/model/models/generator/square.h>
#include <phy_engine/model/models/generator/triangle.h>
#include <phy_engine/netlist/impl.h>

namespace
{
    inline bool near(double a, double b, double eps = 1e-9) noexcept { return ::std::fabs(a - b) <= eps; }
}  // namespace

int main()
{
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl{c.get_netlist()};
    auto& gnd{nl.ground_node};

    constexpr double Vh = 5.0;
    constexpr double Vl = 1.0;
    constexpr double freq = 1000.0;
    constexpr double T = 1.0 / freq;

    auto [sq0, _sq0_pos]{add_model(nl, ::phy_engine::model::square_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .duty = 0.25, .phase = 0.0})};
    auto [sqpi, _sqpi_pos]{
        add_model(nl, ::phy_engine::model::square_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .duty = 0.25, .phase = ::std::numbers::pi})};

    auto [saw0, _saw0_pos]{add_model(nl, ::phy_engine::model::sawtooth_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .phase = 0.0})};
    auto [sawpi, _sawpi_pos]{
        add_model(nl, ::phy_engine::model::sawtooth_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .phase = ::std::numbers::pi})};

    auto [tri0, _tri0_pos]{add_model(nl, ::phy_engine::model::triangle_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .phase = 0.0})};
    auto [trip, _trip_pos]{
        add_model(nl, ::phy_engine::model::triangle_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .phase = ::std::numbers::pi})};

    auto [pulse_hi, _pulse_hi_pos]{
        add_model(nl, ::phy_engine::model::pulse_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .duty = 0.1, .phase = 0.0, .tr = 0.0, .tf = 0.0})};
    auto [pulse_lo, _pulse_lo_pos]{
        add_model(nl,
                  ::phy_engine::model::pulse_gen{.Vh = Vh, .Vl = Vl, .freq = freq, .duty = 0.1, .phase = 1.5 * ::std::numbers::pi, .tr = 0.0, .tf = 0.0})};

    auto& n_sq0{create_node(nl)};
    auto& n_sqpi{create_node(nl)};
    auto& n_saw0{create_node(nl)};
    auto& n_sawpi{create_node(nl)};
    auto& n_tri0{create_node(nl)};
    auto& n_trip{create_node(nl)};
    auto& n_pulse_hi{create_node(nl)};
    auto& n_pulse_lo{create_node(nl)};

    add_to_node(nl, *sq0, 0, n_sq0);
    add_to_node(nl, *sq0, 1, gnd);
    add_to_node(nl, *sqpi, 0, n_sqpi);
    add_to_node(nl, *sqpi, 1, gnd);

    add_to_node(nl, *saw0, 0, n_saw0);
    add_to_node(nl, *saw0, 1, gnd);
    add_to_node(nl, *sawpi, 0, n_sawpi);
    add_to_node(nl, *sawpi, 1, gnd);

    add_to_node(nl, *tri0, 0, n_tri0);
    add_to_node(nl, *tri0, 1, gnd);
    add_to_node(nl, *trip, 0, n_trip);
    add_to_node(nl, *trip, 1, gnd);

    add_to_node(nl, *pulse_hi, 0, n_pulse_hi);
    add_to_node(nl, *pulse_hi, 1, gnd);
    add_to_node(nl, *pulse_lo, 0, n_pulse_lo);
    add_to_node(nl, *pulse_lo, 1, gnd);

    if(!c.analyze())
    {
        ::fast_io::io::perr("generator_dc: analyze failed\n");
        return 1;
    }

    double const v_sq0{n_sq0.node_information.an.voltage.real()};
    double const v_sqpi{n_sqpi.node_information.an.voltage.real()};
    double const v_saw0{n_saw0.node_information.an.voltage.real()};
    double const v_sawpi{n_sawpi.node_information.an.voltage.real()};
    double const v_tri0{n_tri0.node_information.an.voltage.real()};
    double const v_trip{n_trip.node_information.an.voltage.real()};
    double const v_pulse_hi{n_pulse_hi.node_information.an.voltage.real()};
    double const v_pulse_lo{n_pulse_lo.node_information.an.voltage.real()};

    double const v_sawpi_exp{Vl + (Vh - Vl) * 0.5};

    if(!near(v_sq0, Vh) || !near(v_sqpi, Vl) || !near(v_saw0, Vl) || !near(v_sawpi, v_sawpi_exp) || !near(v_tri0, Vl) || !near(v_trip, Vh) ||
       !near(v_pulse_hi, Vh) || !near(v_pulse_lo, Vl))
    {
        ::fast_io::io::perr("generator_dc: unexpected DC values\n",
                            " sq0=", v_sq0, " exp=", Vh, "\n",
                            " sqpi=", v_sqpi, " exp=", Vl, "\n",
                            " saw0=", v_saw0, " exp=", Vl, "\n",
                            " sawpi=", v_sawpi, " exp=", v_sawpi_exp, " (T=", T, ")\n",
                            " tri0=", v_tri0, " exp=", Vl, "\n",
                            " trip=", v_trip, " exp=", Vh, "\n",
                            " pulse_hi=", v_pulse_hi, " exp=", Vh, "\n",
                            " pulse_lo=", v_pulse_lo, " exp=", Vl, "\n");
        return 2;
    }

    return 0;
}

