#pragma once
#include <complex>
// #include <map>
#include <absl/container/btree_map.h>
#include <fast_io/fast_io.h>

// #include <Eigen/Dense>

namespace phy_engine::MNA
{

    struct MNA
    {
        MNA() noexcept = default;

        MNA(::std::size_t ns, ::std::size_t bs) noexcept : node_size{ns}, branch_size{bs}
        {
#if 1
            A.resize(node_size + branch_size);
#endif
#if 0
            X.resize(node_size + branch_size);
#endif
        }

        void resize(::std::size_t ns, ::std::size_t bs) noexcept
        {
            node_size = ns;
            branch_size = bs;
#if 1
            A.resize(node_size + branch_size);
#endif
#if 0
            X.resize(node_size + branch_size);
#endif
        }

        void clear() noexcept
        {
            for(auto& row: A) { row.clear(); }
#if 0
            X.setZero();
#endif
            Z.clear();
        }

        // Keep sparsity pattern, reset numeric values to zero.
        // This avoids re-allocating map nodes across repeated solves of the same netlist.
        void clear_values_keep_pattern() noexcept
        {
            for(auto& row: A)
            {
                for(auto& [_, v]: row) { v = {}; }
            }
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
#if 0
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
#endif
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

        // Row indices are dense [0, node_size + branch_size), so use a row-indexed container.
        ::fast_io::vector<::absl::btree_map<::std::size_t, ::std::complex<double>>> A{};
        ::absl::btree_map<::std::size_t, ::std::complex<double>> Z{};

        ::std::size_t node_size{};
        ::std::size_t branch_size{};
        double r_open{1e12};

    private:
        ::std::complex<double> d_temp;
    };
}  // namespace phy_engine::MNA
