#include <cmath>
#include <iostream>
#include <iomanip>

namespace numerical
{
    struct RCParams
    {
        double R;
        double C;
        double Vin;
    };

    struct RLParams
    {
        double R;
        double L;
        double Vin;
    };

    // Analytic step responses
    inline double rc_v_analytic(double t, RCParams p)
    {
        double const tau{p.R * p.C};
        return p.Vin * (1.0 - std::exp(-t / tau));
    }

    inline double rl_i_analytic(double t, RLParams p)
    {
        double const tau{p.L / p.R};
        return (p.Vin / p.R) * (1.0 - std::exp(-t / tau));
    }

    // Backward Euler updates
    inline double rc_step_backward_euler(double v_prev, RCParams p, double dt)
    {
        // (v_n - v_{n-1})/dt = (Vin - v_n)/(R*C) => v_n = (v_{n-1} + dt*Vin/(R*C)) / (1 + dt/(R*C))
        double const alpha{dt / (p.R * p.C)};
        return (v_prev + alpha * p.Vin) / (1.0 + alpha);
    }

    inline double rl_step_backward_euler(double i_prev, RLParams p, double dt)
    {
        // (i_n - i_{n-1})/dt = (Vin - R*i_n)/L => i_n = (i_{n-1} + dt*Vin/L) / (1 + dt*R/L)
        double const alpha{dt * p.R / p.L};
        return (i_prev + dt * p.Vin / p.L) / (1.0 + alpha);
    }

    // Trapezoidal updates
    inline double rc_step_trapezoidal(double v_prev, RCParams p, double dt)
    {
        // dv/dt = (Vin - v)/(R*C)
        // v_n = v_{n-1} + dt/2 * [(Vin - v_{n-1})/(R*C) + (Vin - v_n)/(R*C)]
        // => (1 + dt/(2*R*C)) v_n = v_{n-1} + dt/(2*R*C) * (2*Vin - v_{n-1})
        double const a{dt / (2.0 * p.R * p.C)};
        return (v_prev + a * (2.0 * p.Vin - v_prev)) / (1.0 + a);
    }

    inline double rl_step_trapezoidal(double i_prev, RLParams p, double dt)
    {
        // di/dt = (Vin - R*i)/L
        // i_n = i_{n-1} + dt/2 * [ (Vin - R*i_{n-1})/L + (Vin - R*i_n)/L ]
        // => (1 + dt*R/(2L)) i_n = i_{n-1} + dt/(2L) * (2*Vin - R*i_{n-1})
        double const a{dt * p.R / (2.0 * p.L)};
        return (i_prev + (dt / (2.0 * p.L)) * (2.0 * p.Vin - p.R * i_prev)) / (1.0 + a);
    }
}

int main()
{
    using std::cout;
    using std::setw;
    using std::setprecision;
    using std::fixed;

    // Parameters
    numerical::RCParams rc{1000.0, 1e-6, 5.0};   // R=1k, C=1uF, step 5V
    numerical::RLParams rl{1000.0, 1e-3, 5.0};   // R=1k, L=1mH, step 5V

    double const T{0.01};  // simulate 10ms
    double const dt{1e-4}; // 0.1ms step

    // State variables
    double v_rc_be{};
    double v_rc_tr{};
    double i_rl_be{};
    double i_rl_tr{};

    cout << fixed << setprecision(8);
    cout << "t\tRC_exact_V\tRC_BE_V\tRC_TR_V\tRC_BE_err\tRC_TR_err\tRL_exact_I\tRL_BE_I\tRL_TR_I\tRL_BE_err\tRL_TR_err\n";

    for(double t{}; t <= T + 1e-12; t += dt)
    {
        // Analytic
        double const v_rc_exact{numerical::rc_v_analytic(t, rc)};
        double const i_rl_exact{numerical::rl_i_analytic(t, rl)};

        // Print current state (before advancing) so t pairs with current estimates
        double const rc_be_err{std::abs(v_rc_be - v_rc_exact)};
        double const rc_tr_err{std::abs(v_rc_tr - v_rc_exact)};
        double const rl_be_err{std::abs(i_rl_be - i_rl_exact)};
        double const rl_tr_err{std::abs(i_rl_tr - i_rl_exact)};

        cout << t << '\t'
             << v_rc_exact << '\t' << v_rc_be << '\t' << v_rc_tr << '\t' << rc_be_err << '\t' << rc_tr_err << '\t'
             << i_rl_exact << '\t' << i_rl_be << '\t' << i_rl_tr << '\t' << rl_be_err << '\t' << rl_tr_err << '\n';

        // Advance one step
        v_rc_be = numerical::rc_step_backward_euler(v_rc_be, rc, dt);
        v_rc_tr = numerical::rc_step_trapezoidal(v_rc_tr, rc, dt);
        i_rl_be = numerical::rl_step_backward_euler(i_rl_be, rl, dt);
        i_rl_tr = numerical::rl_step_trapezoidal(i_rl_tr, rl, dt);
    }

    return 0;
}


