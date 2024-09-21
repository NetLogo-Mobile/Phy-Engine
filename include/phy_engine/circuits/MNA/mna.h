#pragma once
#include <complex>
#include <fast_io/fast_io.h>
#include <Eigen/Sparse>


namespace phy_engine::MNA
{
    using sparse_vector = ::Eigen::SparseVector<double>;
    using sparse_matrix = ::Eigen::SparseMatrix<double>;
    using sparse_complex_vector = ::Eigen::SparseVector<::std::complex<double>>;
    using sparse_complex_matrix = ::Eigen::SparseMatrix<::std::complex<double>>;

    struct MNA
    {
        MNA() noexcept = default;

        MNA(::std::size_t ns, ::std::size_t bs) noexcept : node_size{ns}, branch_size{bs}
        {
            auto const npm_size{node_size + branch_size};
            A.resize(npm_size, npm_size);
            X.resize(npm_size, 1);
            Z.resize(npm_size, 1);
        }

        void resize(::std::size_t ns, ::std::size_t bs) noexcept
        {
            node_size = ns;
            branch_size = bs;
            auto const npm_size{node_size + branch_size};
            A.resize(npm_size, npm_size);
            X.resize(npm_size, 1);
            Z.resize(npm_size, 1);
        }

        void clear() noexcept
        {
            A.setZero();
            X.setZero();
            Z.setZero();
        }

        void clear_destroy() noexcept
        {
            A.~SparseMatrix();
            X.~SparseVector();
            Z.~SparseVector();
        }

        auto& a_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox >= node_size + branch_size || col >= node_size + branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return A.coeffRef(rox, col);
        }

        auto& G_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox >= node_size || col >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return A.coeffRef(rox, col);
        }

        auto& B_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox >= node_size || col >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return A.coeffRef(rox, col + node_size);
        }

        auto& C_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox >= branch_size || col >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return A.coeffRef(rox + node_size, col);
        }

        auto& D_ref(::std::size_t rox, ::std::size_t col) noexcept
        {
            if(rox >= branch_size || col >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return A.coeffRef(rox + node_size, col + node_size);
        }

        auto& X_ref(::std::size_t rox) noexcept
        {
            if(rox >= node_size + branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return X.coeffRef(rox);
        }

        auto& V_ref(::std::size_t rox) noexcept
        {
            if(rox >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return X.coeffRef(rox);
        }

        auto& J_ref(::std::size_t rox) noexcept
        {
            if(rox >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return X.coeffRef(rox + node_size);
        }

        auto& Z_ref(::std::size_t rox) noexcept
        {
            if(rox >= node_size + branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return Z.coeffRef(rox);
        }

        auto& I_ref(::std::size_t rox) noexcept
        {
            if(rox >= node_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return Z.coeffRef(rox);
        }

        auto& E_ref(::std::size_t rox) noexcept
        {
            if(rox >= branch_size) [[unlikely]] { ::fast_io::fast_terminate(); }
            return Z.coeffRef(rox + node_size);
        }

        sparse_complex_matrix A{};
        sparse_complex_vector X{};
        sparse_complex_vector Z{};

        ::std::size_t node_size{};
        ::std::size_t branch_size{};
    };
}  // namespace phy_engine::MNA
