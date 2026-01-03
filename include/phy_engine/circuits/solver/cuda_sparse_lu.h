#pragma once

#if defined(__CUDA__) && !defined(__CUDA_ARCH__)

    #include <complex>
    #include <cstddef>
    #include <memory>

    #include <cuda_runtime.h>
    #include <cuComplex.h>
    #include <cusolverSp.h>
    #include <cusparse.h>

namespace phy_engine::solver
{
    class cuda_sparse_lu
    {
    public:
        cuda_sparse_lu() noexcept
        {
            int device_count{};
            if(cudaGetDeviceCount(&device_count) != cudaSuccess || device_count <= 0)
            {
                available = false;
                return;
            }

            if(cudaSetDevice(0) != cudaSuccess)
            {
                available = false;
                return;
            }

            if(cusolverSpCreate(&cusolver_handle) != CUSOLVER_STATUS_SUCCESS)
            {
                available = false;
                return;
            }

            if(cusparseCreateMatDescr(&descrA) != CUSPARSE_STATUS_SUCCESS)
            {
                available = false;
                return;
            }

            cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL);
            cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ZERO);

            available = true;
        }

        cuda_sparse_lu(cuda_sparse_lu const&) = delete;
        cuda_sparse_lu& operator= (cuda_sparse_lu const&) = delete;

        cuda_sparse_lu(cuda_sparse_lu&& other) noexcept { *this = static_cast<cuda_sparse_lu&&>(other); }

        cuda_sparse_lu& operator= (cuda_sparse_lu&& other) noexcept
        {
            if(this == ::std::addressof(other)) { return *this; }

            cleanup();

            available = other.available;
            cusolver_handle = other.cusolver_handle;
            descrA = other.descrA;

            n_capacity = other.n_capacity;
            nnz_capacity = other.nnz_capacity;

            d_row_ptr = other.d_row_ptr;
            d_col_ind = other.d_col_ind;
            d_values = other.d_values;
            d_b = other.d_b;
            d_x = other.d_x;

            other.available = false;
            other.cusolver_handle = nullptr;
            other.descrA = nullptr;
            other.n_capacity = 0;
            other.nnz_capacity = 0;
            other.d_row_ptr = nullptr;
            other.d_col_ind = nullptr;
            other.d_values = nullptr;
            other.d_b = nullptr;
            other.d_x = nullptr;

            return *this;
        }

        ~cuda_sparse_lu() noexcept { cleanup(); }

        [[nodiscard]] bool is_available() const noexcept { return available; }

        [[nodiscard]] bool solve_csr(int n,
                                     int nnz,
                                     int const* row_ptr_host,
                                     int const* col_ind_host,
                                     ::std::complex<double> const* values_host,
                                     ::std::complex<double> const* b_host,
                                     ::std::complex<double>* x_host) noexcept
        {
            static_assert(sizeof(::std::complex<double>) == sizeof(cuDoubleComplex));
            if(!available || n <= 0 || nnz < 0) { return false; }
            if(row_ptr_host == nullptr || col_ind_host == nullptr || values_host == nullptr || b_host == nullptr || x_host == nullptr) { return false; }

            if(!ensure_capacity(n, nnz)) { return false; }

            if(cudaMemcpy(d_row_ptr, row_ptr_host, static_cast<::std::size_t>(n + 1) * sizeof(int), cudaMemcpyHostToDevice) != cudaSuccess) { return false; }
            if(cudaMemcpy(d_col_ind, col_ind_host, static_cast<::std::size_t>(nnz) * sizeof(int), cudaMemcpyHostToDevice) != cudaSuccess) { return false; }
            if(cudaMemcpy(d_values, values_host, static_cast<::std::size_t>(nnz) * sizeof(cuDoubleComplex), cudaMemcpyHostToDevice) != cudaSuccess)
            {
                return false;
            }
            if(cudaMemcpy(d_b, b_host, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaMemcpyHostToDevice) != cudaSuccess) { return false; }

            int singularity{-1};
            constexpr double tol{0.0};
            constexpr int reorder{1};

            auto const st{cusolverSpZcsrlsvqr(cusolver_handle, n, nnz, descrA, d_values, d_row_ptr, d_col_ind, d_b, tol, reorder, d_x, &singularity)};

            if(st != CUSOLVER_STATUS_SUCCESS || singularity >= 0) { return false; }

            if(cudaMemcpy(x_host, d_x, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaMemcpyDeviceToHost) != cudaSuccess) { return false; }
            return true;
        }

    private:
        bool available{};
        cusolverSpHandle_t cusolver_handle{};
        cusparseMatDescr_t descrA{};

        int n_capacity{};
        int nnz_capacity{};

        int* d_row_ptr{};
        int* d_col_ind{};
        cuDoubleComplex* d_values{};
        cuDoubleComplex* d_b{};
        cuDoubleComplex* d_x{};

        void cleanup() noexcept
        {
            if(d_row_ptr) { cudaFree(d_row_ptr); }
            if(d_col_ind) { cudaFree(d_col_ind); }
            if(d_values) { cudaFree(d_values); }
            if(d_b) { cudaFree(d_b); }
            if(d_x) { cudaFree(d_x); }
            d_row_ptr = nullptr;
            d_col_ind = nullptr;
            d_values = nullptr;
            d_b = nullptr;
            d_x = nullptr;
            n_capacity = 0;
            nnz_capacity = 0;

            if(descrA) { cusparseDestroyMatDescr(descrA); }
            descrA = nullptr;

            if(cusolver_handle) { cusolverSpDestroy(cusolver_handle); }
            cusolver_handle = nullptr;
        }

        [[nodiscard]] bool ensure_capacity(int n, int nnz) noexcept
        {
            if(n <= n_capacity && nnz <= nnz_capacity) { return true; }

            if(d_row_ptr) { cudaFree(d_row_ptr); }
            if(d_col_ind) { cudaFree(d_col_ind); }
            if(d_values) { cudaFree(d_values); }
            if(d_b) { cudaFree(d_b); }
            if(d_x) { cudaFree(d_x); }

            d_row_ptr = nullptr;
            d_col_ind = nullptr;
            d_values = nullptr;
            d_b = nullptr;
            d_x = nullptr;

            if(cudaMalloc(reinterpret_cast<void**>(&d_row_ptr), static_cast<::std::size_t>(n + 1) * sizeof(int)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_col_ind), static_cast<::std::size_t>(nnz) * sizeof(int)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_values), static_cast<::std::size_t>(nnz) * sizeof(cuDoubleComplex)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_b), static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_x), static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex)) != cudaSuccess) { return false; }

            n_capacity = n;
            nnz_capacity = nnz;
            return true;
        }
    };

}  // namespace phy_engine::solver

#endif
