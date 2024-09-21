#pragma once
#include <cstdint>
#include <cstddef>
#include <complex>
#include <algorithm>
#include <fast_io/fast_io_dsal/vector.h>
#include "integral_history.h"

namespace phy_engine::solver
{
    struct integral_corrector_gear
    {
        inline static constexpr ::std::size_t max_order{::phy_engine::solver::integral_history::MaxNumHistoryNum};
        inline static constexpr ::std::size_t matrix_rows{max_order + 1};
        double m_coeffs[matrix_rows + 1]{};
        ::std::complex<double> m_A[matrix_rows * matrix_rows]{};
        ::std::complex<double> m_x[matrix_rows]{};
        ::std::complex<double> m_b[matrix_rows]{};
        double m_trucnErrorCoeff{};

        ::std::size_t m_order{};

        ::fast_io::vector<::std::size_t> m_integralU{};
        ::fast_io::vector<::std::size_t> m_integralJ{};


#if __cpp_lib_constexpr_cmath >= 202202L
        constexpr
#endif
            bool
            setOrder(::std::size_t order, ::phy_engine::solver::integral_history const& hsteps) noexcept
        {
            m_order = order;
            setStep(hsteps);

            /*
             * Generate constant ai and bi of GEAR algorithm.
             *
             * The principle is shown as follows:
             * Linear multistep interation:
             * \f[
             *      x_{n+1} = \sum_{i=0}^{p}a_i x_{n-i} + h_n\sum_{i=-1}^{p}b_i f(x_{n-i},t_{n-i})
             * \f]
             *
             * Whereas \f$ p = order + 1 \f$; ai and bi meet:
             * \f[
             *      \begin{cases}
             *      \sum\limits_{i=0}^{p}a_i=1 \\
             *      \sum\limits_{i=1}^{p}{(-i)}^ja_i+ j\sum\limits_{i=-1}^{p} {(-1)}^{j-1}b_i = 1, & j=1,2, \cdots , k
             *      \end{cases}
             * \f]
             *
             * For Implicit GEAR:
             * \f[
             *      p=k-1, b_0=b_1=\cdots=b_{k-1}=0
             * \f]
             *
             * So the coefficient ai and bi can be obtained by solving the above equation.
             */
            ::std::size_t rows{order + 1};

            ::std::fill(m_A, m_A + rows * rows, 0.0);
            ::std::fill(m_b, m_b + rows, 0.0);

            /* Generate A */
            for(::std::size_t i{1}; i <= order; i++)
            {
                m_A[i * rows] = static_cast<double>(i);
                m_A[i] = 1.0;
            }
            for(::std::size_t i{1}; i <= order - 1; i++)
            {
                auto t{static_cast<double>(-static_cast<::std::ptrdiff_t>(i))};
                for(::std::size_t j{1}; j <= order; j++)
                {
                    m_A[j * rows + (i + 1)] = t;
                    t *= static_cast<double>(-static_cast<::std::ptrdiff_t>(i));
                }
            }

            /* Generate b */
            for(::std::size_t i{}; i <= order; i++) { m_b[i] = 1.0; }

/* Solve x */
#if 0
            if(m_linearSolver->solve(m_A, rows, m_x, m_b)) { return false; }
#endif

            /*
             * Generate truncation error coefficient
             */
            m_trucnErrorCoeff = 0.0;
            double f{1.0};
            for(auto i{order + 1}; i > 1u; --i) { f *= static_cast<double>(i); }
            for(::std::ptrdiff_t i{-1}; i < static_cast<::std::ptrdiff_t>(order) - 1; ++i)
            {
                double a{(i == -1 ? -1.0 : m_x[i + 1].real())};
                m_trucnErrorCoeff -= a * ::std::pow(static_cast<::std::ptrdiff_t>(order) - 1 - i, order + 1);
            }
            m_trucnErrorCoeff /= f;
            f /= order + 1;
            m_trucnErrorCoeff -= m_x[0].real() * ::std::pow(order, order) / f;

            return true;
        }

        constexpr int setStep(::phy_engine::solver::integral_history const& hsteps) noexcept
        {
            auto order{m_order};
            auto rows{order + 1};

            ::std::fill(m_A, m_A + rows * rows, 0.0);
            ::std::fill(m_b, m_b + rows, 0.0);

            /* Generate A */
            for(auto c{m_A}; c != m_A + order + 1; ++c) { *c = 1.0; }
            double f{};
            for(::std::size_t c{}; c < order; c++)
            {
                f += hsteps.get(c);
                double a{1.0};
                for(::std::size_t r = 0; r < order; r++)
                {
                    a *= f / hsteps.get(0);
                    m_A[(r + 1) * rows + c + 1] = a;
                }
            }
            /* Generate B */
            m_b[1] = -1.0 / hsteps.get(0);

/* Solve x */
#if 0
            if(m_linearSolver->solve(m_A, rows, m_x, m_b)) { return CERR_SET_INTEGRAL_STEP; }
#endif

            for(::std::size_t r{}; r <= order; r++) { m_coeffs[r] = m_x[r].real(); }

            return true;
        }

        constexpr void integrate(::phy_engine::solver::integral_history const& x,
                                 ::phy_engine::solver::integral_history& y,
                                 double k,
                                 double& c0,
                                 double& c1) const noexcept
        {
            c0 = k * m_coeffs[0];
            c1 = 0.0;
            for(::std::size_t i{1}; i <= m_order; i++) { c1 += k * m_coeffs[i] * x.get(i); }
            y.set(0, x.get(0) * (c0) + c1);
        }

        constexpr double getTruncErrorCoeff() const noexcept { return m_trucnErrorCoeff; }

        constexpr double getIntegralCoeff(::std::size_t index) const noexcept { return m_coeffs[index]; }
    };

}  // namespace phy_engine::solver
