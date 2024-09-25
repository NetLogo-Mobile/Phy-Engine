#pragma once
#include <complex>
#include <map>
#include <fast_io/fast_io.h>
#include <Eigen/Dense>

namespace phy_engine::MNA
{

    struct MNA
    {
        MNA() noexcept = default;

        MNA(::std::size_t ns, ::std::size_t bs) noexcept : node_size{ns}, branch_size{bs}
        {
#if 0
            X.resize(node_size + branch_size);
#endif
        }

        void resize(::std::size_t ns, ::std::size_t bs) noexcept
        {
            node_size = ns;
            branch_size = bs;
#if 0
            X.resize(node_size + branch_size);
#endif
        }

        void clear() noexcept
        {
            A.clear();
            X.setZero();
            Z.clear();
        }

        void clear_destroy() noexcept { clear(); }

        auto& A_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox == SIZE_MAX || col == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size + branch_size || col >= node_size + branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return A[rox][col];
        }

        auto& G_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox == SIZE_MAX || col == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size || col >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return A[rox][col];
        }

        auto& B_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox == SIZE_MAX || col == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size || col >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return A[rox][col + node_size];
        }

        auto& C_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox == SIZE_MAX || col == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= branch_size || col >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return A[rox + node_size][col];
        }

        auto& D_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox == SIZE_MAX || col == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= branch_size || col >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return A[rox + node_size][col + node_size];
        }

        auto& X_ref(::std::size_t rox) noexcept
        {
            if(rox == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size + branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return X.coeffRef(rox);
        }

        auto& V_ref(::std::size_t rox) noexcept
        {
            if(rox == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return X.coeffRef(rox);
        }

        auto& J_ref(::std::size_t rox) noexcept
        {
            if(rox == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return X.coeffRef(rox + node_size);
        }

        auto& Z_ref(::std::size_t rox) noexcept
        {
            if(rox == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size + branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return Z[rox];
        }

        auto& I_ref(::std::size_t rox) noexcept
        {
            if(rox == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return Z[rox];
        }

        auto& E_ref(::std::size_t rox) noexcept
        {
            if(rox == SIZE_MAX) [[unlikely]] { return d_temp; }
#ifdef _DEBUG
            if(rox >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
#endif
            return Z[rox + node_size];
        }

        ::std::map<::std::size_t, ::std::map<::std::size_t, ::std::complex<double>>> A{};
        ::Eigen::VectorXcd X{};
        ::std::map<::std::size_t, ::std::complex<double>> Z{};

        ::std::size_t node_size{};
        ::std::size_t branch_size{};

    private:
        ::std::complex<double> d_temp;
    };
}  // namespace phy_engine::MNA
