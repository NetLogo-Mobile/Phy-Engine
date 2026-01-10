#pragma once

#if (defined(__CUDA__) || defined(__CUDACC__) || defined(__NVCC__)) && !defined(__CUDA_ARCH__)

    #include <complex>
    #include <cstddef>
    #include <cstring>
    #include <cstdio>
    #include <chrono>
    #include <cstdlib>
    #include <memory>
    #include <utility>

    #include <cuda_runtime.h>
    #include <cuComplex.h>
    #include <cublas_v2.h>
    #include <cusolverSp.h>
    #include <cusparse.h>

namespace phy_engine::solver
{
    class cuda_sparse_lu
    {
    public:
        struct timings
        {
            double h2d_ms{};
            double solve_ms{};
            double d2h_ms{};
            double solve_host_ms{};
            double solve_total_host_ms{};
        };

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

            if(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking) != cudaSuccess)
            {
                available = false;
                return;
            }

            if(cublasCreate(&cublas_handle) != CUBLAS_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }
            if(cublasSetStream(cublas_handle, stream) != CUBLAS_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }

            if(cusparseCreate(&cusparse_handle) != CUSPARSE_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }
            if(cusparseSetStream(cusparse_handle, stream) != CUSPARSE_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }

            if(cusolverSpCreate(&cusolver_handle) != CUSOLVER_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }

            if(cusolverSpSetStream(cusolver_handle, stream) != CUSOLVER_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }

            if(cusparseCreateMatDescr(&descrA) != CUSPARSE_STATUS_SUCCESS)
            {
                available = false;
                cleanup();
                return;
            }

            cusparseSetMatType(descrA, CUSPARSE_MATRIX_TYPE_GENERAL);
            cusparseSetMatIndexBase(descrA, CUSPARSE_INDEX_BASE_ZERO);

            pinned = []() noexcept {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_PINNED");
                return v != nullptr && (*v == '1' || *v == 'y' || *v == 'Y' || *v == 't' || *v == 'T');
            }();

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
            cublas_handle = other.cublas_handle;
            cusparse_handle = other.cusparse_handle;
            cusolver_handle = other.cusolver_handle;
            descrA = other.descrA;
            stream = other.stream;

            pattern_uploaded = other.pattern_uploaded;
            pattern_n = other.pattern_n;
            pattern_nnz = other.pattern_nnz;

            pinned = other.pinned;
            pinned_n_capacity_z = other.pinned_n_capacity_z;
            pinned_nnz_capacity_z = other.pinned_nnz_capacity_z;
            pinned_n_capacity_d = other.pinned_n_capacity_d;
            pinned_nnz_capacity_d = other.pinned_nnz_capacity_d;
            h_values_z = other.h_values_z;
            h_b_z = other.h_b_z;
            h_x_z = other.h_x_z;
            h_values_d = other.h_values_d;
            h_b_d = other.h_b_d;
            h_x_d = other.h_x_d;

            n_capacity = other.n_capacity;
            nnz_capacity = other.nnz_capacity;

            d_row_ptr = other.d_row_ptr;
            d_col_ind = other.d_col_ind;
            n_capacity_z = other.n_capacity_z;
            nnz_capacity_z = other.nnz_capacity_z;
            d_values_z = other.d_values_z;
            d_b_z = other.d_b_z;
            d_x_z = other.d_x_z;

            n_capacity_d = other.n_capacity_d;
            nnz_capacity_d = other.nnz_capacity_d;
            d_values_d = other.d_values_d;
            d_b_d = other.d_b_d;
            d_x_d = other.d_x_d;

            ilu_n_d = other.ilu_n_d;
            ilu_nnz_d = other.ilu_nnz_d;
            ilu_info_d = other.ilu_info_d;
            ilu_buffer_d = other.ilu_buffer_d;
            ilu_buffer_bytes_d = other.ilu_buffer_bytes_d;
            spmatA_d = other.spmatA_d;
            spmatL_d = other.spmatL_d;
            spmatU_d = other.spmatU_d;
            spsvL_d = other.spsvL_d;
            spsvU_d = other.spsvU_d;
            vec_in_d = other.vec_in_d;
            vec_out_d = other.vec_out_d;
            spmv_buffer_d = other.spmv_buffer_d;
            spmv_buffer_bytes_d = other.spmv_buffer_bytes_d;
            spsvL_buffer_d = other.spsvL_buffer_d;
            spsvL_buffer_bytes_d = other.spsvL_buffer_bytes_d;
            spsvU_buffer_d = other.spsvU_buffer_d;
            spsvU_buffer_bytes_d = other.spsvU_buffer_bytes_d;
            d_ilu_vals_d = other.d_ilu_vals_d;
            d_r_d = other.d_r_d;
            d_rhat_d = other.d_rhat_d;
            d_p_d = other.d_p_d;
            d_v_d = other.d_v_d;
            d_s_d = other.d_s_d;
            d_t_d = other.d_t_d;
            d_y_d = other.d_y_d;
            d_z_d = other.d_z_d;

            other.available = false;
            other.cublas_handle = nullptr;
            other.cusparse_handle = nullptr;
            other.cusolver_handle = nullptr;
            other.descrA = nullptr;
            other.stream = nullptr;
            other.pattern_uploaded = false;
            other.pattern_n = 0;
            other.pattern_nnz = 0;
            other.pinned = false;
            other.pinned_n_capacity_z = 0;
            other.pinned_nnz_capacity_z = 0;
            other.pinned_n_capacity_d = 0;
            other.pinned_nnz_capacity_d = 0;
            other.h_values_z = nullptr;
            other.h_b_z = nullptr;
            other.h_x_z = nullptr;
            other.h_values_d = nullptr;
            other.h_b_d = nullptr;
            other.h_x_d = nullptr;
            other.n_capacity = 0;
            other.nnz_capacity = 0;
            other.d_row_ptr = nullptr;
            other.d_col_ind = nullptr;
            other.n_capacity_z = 0;
            other.nnz_capacity_z = 0;
            other.d_values_z = nullptr;
            other.d_b_z = nullptr;
            other.d_x_z = nullptr;
            other.n_capacity_d = 0;
            other.nnz_capacity_d = 0;
            other.d_values_d = nullptr;
            other.d_b_d = nullptr;
            other.d_x_d = nullptr;
            other.ilu_n_d = 0;
            other.ilu_nnz_d = 0;
            other.ilu_info_d = nullptr;
            other.ilu_buffer_d = nullptr;
            other.ilu_buffer_bytes_d = 0;
            other.spmatA_d = nullptr;
            other.spmatL_d = nullptr;
            other.spmatU_d = nullptr;
            other.spsvL_d = nullptr;
            other.spsvU_d = nullptr;
            other.vec_in_d = nullptr;
            other.vec_out_d = nullptr;
            other.spmv_buffer_d = nullptr;
            other.spmv_buffer_bytes_d = 0;
            other.spsvL_buffer_d = nullptr;
            other.spsvL_buffer_bytes_d = 0;
            other.spsvU_buffer_d = nullptr;
            other.spsvU_buffer_bytes_d = 0;
            other.d_ilu_vals_d = nullptr;
            other.d_r_d = nullptr;
            other.d_rhat_d = nullptr;
            other.d_p_d = nullptr;
            other.d_v_d = nullptr;
            other.d_s_d = nullptr;
            other.d_t_d = nullptr;
            other.d_y_d = nullptr;
            other.d_z_d = nullptr;

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
                                     ::std::complex<double>* x_host,
                                     bool copy_pattern = true) noexcept
        {
            timings ignored{};
            return solve_csr_timed(n, nnz, row_ptr_host, col_ind_host, values_host, b_host, x_host, ignored, copy_pattern);
        }

        [[nodiscard]] bool solve_csr_timed(int n,
                                           int nnz,
                                           int const* row_ptr_host,
                                           int const* col_ind_host,
                                           ::std::complex<double> const* values_host,
                                           ::std::complex<double> const* b_host,
                                           ::std::complex<double>* x_host,
                                           timings& out,
                                           bool copy_pattern = true) noexcept
        {
            static_assert(sizeof(::std::complex<double>) == sizeof(cuDoubleComplex));
            if(!available || n <= 0 || nnz < 0) { return false; }
            if(row_ptr_host == nullptr || col_ind_host == nullptr || values_host == nullptr || b_host == nullptr || x_host == nullptr) { return false; }

            if(!ensure_capacity_common(n, nnz)) { return false; }
            if(!ensure_capacity_complex(n, nnz)) { return false; }
            if(!ensure_pinned_complex(n, nnz)) { return false; }

            auto t_h2d_start = cudaEvent_t{};
            auto t_h2d_end = cudaEvent_t{};
            auto t_solve_start = cudaEvent_t{};
            auto t_solve_end = cudaEvent_t{};
            auto t_d2h_end = cudaEvent_t{};
            if(cudaEventCreate(&t_h2d_start) != cudaSuccess) { return false; }
            if(cudaEventCreate(&t_h2d_end) != cudaSuccess) { cudaEventDestroy(t_h2d_start); return false; }
            if(cudaEventCreate(&t_solve_start) != cudaSuccess)
            {
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }
            if(cudaEventCreate(&t_solve_end) != cudaSuccess)
            {
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }
            if(cudaEventCreate(&t_d2h_end) != cudaSuccess)
            {
                cudaEventDestroy(t_solve_end);
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }

            auto const destroy_events = [&]() noexcept {
                cudaEventDestroy(t_d2h_end);
                cudaEventDestroy(t_solve_end);
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
            };

            if(cudaEventRecord(t_h2d_start, stream) != cudaSuccess) { destroy_events(); return false; }

            bool const must_copy_pattern{copy_pattern || !pattern_uploaded || pattern_n != n || pattern_nnz != nnz};
            if(must_copy_pattern)
            {
                if(cudaMemcpyAsync(d_row_ptr, row_ptr_host, static_cast<::std::size_t>(n + 1) * sizeof(int), cudaMemcpyHostToDevice, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                if(cudaMemcpyAsync(d_col_ind, col_ind_host, static_cast<::std::size_t>(nnz) * sizeof(int), cudaMemcpyHostToDevice, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                pattern_uploaded = true;
                pattern_n = n;
                pattern_nnz = nnz;
            }
            void const* values_src{values_host};
            void const* b_src{b_host};
            if(pinned && h_values_z && h_b_z)
            {
                std::memcpy(h_values_z, values_host, static_cast<::std::size_t>(nnz) * sizeof(cuDoubleComplex));
                std::memcpy(h_b_z, b_host, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex));
                values_src = h_values_z;
                b_src = h_b_z;
            }

            if(cudaMemcpyAsync(d_values_z, values_src, static_cast<::std::size_t>(nnz) * sizeof(cuDoubleComplex), cudaMemcpyHostToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }
            if(cudaMemcpyAsync(d_b_z, b_src, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaMemcpyHostToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }

            if(cudaEventRecord(t_h2d_end, stream) != cudaSuccess) { destroy_events(); return false; }

            int singularity{-1};
            constexpr double tol{0.0};
            int const reorder = []() noexcept {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_QR_REORDER");
                if(v == nullptr) { return 1; }
                return (*v == '0') ? 0 : 1;
            }();

            if(cudaEventRecord(t_solve_start, stream) != cudaSuccess) { destroy_events(); return false; }
            auto const host_solve0 = ::std::chrono::steady_clock::now();
            // Prefer LU for general sparse systems; fallback to QR when LU is unavailable/unsupported.
            auto st = CUSOLVER_STATUS_INTERNAL_ERROR;
#if defined(CUSOLVER_VER_MAJOR)
#if (CUSOLVER_VER_MAJOR >= 12)
            st = cusolverSpZcsrlsvlu(cusolver_handle, n, nnz, descrA, d_values_z, d_row_ptr, d_col_ind, d_b_z, tol, reorder, d_x_z, &singularity);
#endif
#endif
            if(st != CUSOLVER_STATUS_SUCCESS) { st = cusolverSpZcsrlsvqr(cusolver_handle, n, nnz, descrA, d_values_z, d_row_ptr, d_col_ind, d_b_z, tol, reorder, d_x_z, &singularity); }
            auto const host_solve1 = ::std::chrono::steady_clock::now();

            if(cudaEventRecord(t_solve_end, stream) != cudaSuccess) { destroy_events(); return false; }

            if(st != CUSOLVER_STATUS_SUCCESS || singularity >= 0) { destroy_events(); return false; }

            if(pinned && h_x_z)
            {
                if(cudaMemcpyAsync(h_x_z, d_x_z, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaMemcpyDeviceToHost, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
            }
            else
            {
                if(cudaMemcpyAsync(x_host, d_x_z, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaMemcpyDeviceToHost, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
            }

            if(cudaEventRecord(t_d2h_end, stream) != cudaSuccess) { destroy_events(); return false; }
            if(cudaEventSynchronize(t_d2h_end) != cudaSuccess) { destroy_events(); return false; }

            if(pinned && h_x_z)
            {
                std::memcpy(x_host, h_x_z, static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex));
            }

            float ms_h2d{};
            float ms_solve{};
            float ms_d2h{};
            cudaEventElapsedTime(&ms_h2d, t_h2d_start, t_h2d_end);
            cudaEventElapsedTime(&ms_solve, t_solve_start, t_solve_end);
            cudaEventElapsedTime(&ms_d2h, t_solve_end, t_d2h_end);
            out.h2d_ms = static_cast<double>(ms_h2d);
            out.solve_ms = static_cast<double>(ms_solve);
            out.d2h_ms = static_cast<double>(ms_d2h);
            out.solve_host_ms = ::std::chrono::duration_cast<::std::chrono::duration<double, ::std::milli>>(host_solve1 - host_solve0).count();
            out.solve_total_host_ms = out.h2d_ms + out.solve_host_ms + out.d2h_ms;
            destroy_events();
            return true;
        }

        [[nodiscard]] bool solve_csr_real(int n,
                                          int nnz,
                                          int const* row_ptr_host,
                                          int const* col_ind_host,
                                          double const* values_host,
                                          double const* b_host,
                                          double* x_host,
                                          timings& out,
                                          bool copy_pattern = true) noexcept
        {
            if(!available || n <= 0 || nnz < 0) { return false; }
            if(row_ptr_host == nullptr || col_ind_host == nullptr || values_host == nullptr || b_host == nullptr || x_host == nullptr) { return false; }

            if(!ensure_capacity_common(n, nnz)) { return false; }
            if(!ensure_capacity_real(n, nnz)) { return false; }
            if(!ensure_pinned_real(n, nnz)) { return false; }

            static int const solver_mode = []() noexcept
            {
                // 0: QR (default), 1: ILU0+BiCGSTAB
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_SOLVER");
                if(v == nullptr) { return 0; }
                if(*v == 'i' || *v == 'I') { return 1; }
                return 0;
            }();

            if(solver_mode == 1)
            {
                return solve_csr_real_ilu0_bicgstab(n, nnz, row_ptr_host, col_ind_host, values_host, b_host, x_host, out, copy_pattern);
            }

            auto t_h2d_start = cudaEvent_t{};
            auto t_h2d_end = cudaEvent_t{};
            auto t_solve_start = cudaEvent_t{};
            auto t_solve_end = cudaEvent_t{};
            auto t_d2h_end = cudaEvent_t{};
            if(cudaEventCreate(&t_h2d_start) != cudaSuccess) { return false; }
            if(cudaEventCreate(&t_h2d_end) != cudaSuccess) { cudaEventDestroy(t_h2d_start); return false; }
            if(cudaEventCreate(&t_solve_start) != cudaSuccess)
            {
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }
            if(cudaEventCreate(&t_solve_end) != cudaSuccess)
            {
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }
            if(cudaEventCreate(&t_d2h_end) != cudaSuccess)
            {
                cudaEventDestroy(t_solve_end);
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }

            auto const destroy_events = [&]() noexcept {
                cudaEventDestroy(t_d2h_end);
                cudaEventDestroy(t_solve_end);
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
            };

            if(cudaEventRecord(t_h2d_start, stream) != cudaSuccess) { destroy_events(); return false; }

            bool const must_copy_pattern{copy_pattern || !pattern_uploaded || pattern_n != n || pattern_nnz != nnz};
            if(must_copy_pattern)
            {
                if(cudaMemcpyAsync(d_row_ptr, row_ptr_host, static_cast<::std::size_t>(n + 1) * sizeof(int), cudaMemcpyHostToDevice, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                if(cudaMemcpyAsync(d_col_ind, col_ind_host, static_cast<::std::size_t>(nnz) * sizeof(int), cudaMemcpyHostToDevice, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                pattern_uploaded = true;
                pattern_n = n;
                pattern_nnz = nnz;
            }
            double const* values_src{values_host};
            double const* b_src{b_host};
            if(pinned && h_values_d && h_b_d)
            {
                std::memcpy(h_values_d, values_host, static_cast<::std::size_t>(nnz) * sizeof(double));
                std::memcpy(h_b_d, b_host, static_cast<::std::size_t>(n) * sizeof(double));
                values_src = h_values_d;
                b_src = h_b_d;
            }

            if(cudaMemcpyAsync(d_values_d, values_src, static_cast<::std::size_t>(nnz) * sizeof(double), cudaMemcpyHostToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }
            if(cudaMemcpyAsync(d_b_d, b_src, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyHostToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }

            if(cudaEventRecord(t_h2d_end, stream) != cudaSuccess) { destroy_events(); return false; }

            bool const use_batched_qr = []() noexcept {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_QR_BATCHED");
                // Default ON (batched QR is typically more stable and faster for repeated solves).
                return v == nullptr || (*v == '1' || *v == 'y' || *v == 'Y' || *v == 't' || *v == 'T');
            }();
            bool const debug = []() noexcept {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_DEBUG");
                return v != nullptr && (*v == '1' || *v == 'y' || *v == 'Y' || *v == 't' || *v == 'T');
            }();

            int singularity{-1};
            constexpr double tol{0.0};
            int const reorder = []() noexcept {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_QR_REORDER");
                if(v == nullptr) { return 1; }
                return (*v == '0') ? 0 : 1;
            }();

            cusolverStatus_t st{};
            auto const host_solve0 = ::std::chrono::steady_clock::now();
            if(cudaEventRecord(t_solve_start, stream) != cudaSuccess) { destroy_events(); return false; }
            if(use_batched_qr)
            {
                st = prepare_csrqr_real(n, nnz);
                if(st == CUSOLVER_STATUS_SUCCESS)
                {
                    st = cusolverSpDcsrqrsvBatched(cusolver_handle,
                                                   n,
                                                   n,
                                                   nnz,
                                                   descrA,
                                                   d_values_d,
                                                   d_row_ptr,
                                                   d_col_ind,
                                                   d_b_d,
                                                   d_x_d,
                                                   1,
                                                   csrqr_info_d,
                                                   csrqr_buffer_d);
                }
                if(st != CUSOLVER_STATUS_SUCCESS && debug)
                {
                    std::fprintf(stderr,
                                 "[phy_engine][cuda] csrqrsvBatched failed: st=%d n=%d nnz=%d internal=%zu workspace=%zu buf=%zu (fallback=csrlsvqr)\n",
                                 static_cast<int>(st),
                                 n,
                                 nnz,
                                 last_csrqr_internal_bytes_d,
                                 last_csrqr_workspace_bytes_d,
                                 csrqr_buffer_bytes_d);
                }
            }
            if(st != CUSOLVER_STATUS_SUCCESS)
            {
                // Prefer LU for general sparse systems when available; fallback to QR.
#if defined(CUSOLVER_VER_MAJOR)
#if (CUSOLVER_VER_MAJOR >= 12)
                st = cusolverSpDcsrlsvlu(cusolver_handle, n, nnz, descrA, d_values_d, d_row_ptr, d_col_ind, d_b_d, tol, reorder, d_x_d, &singularity);
#endif
#endif
                if(st != CUSOLVER_STATUS_SUCCESS)
                {
                    st = cusolverSpDcsrlsvqr(cusolver_handle, n, nnz, descrA, d_values_d, d_row_ptr, d_col_ind, d_b_d, tol, reorder, d_x_d, &singularity);
                }
            }
            auto const host_solve1 = ::std::chrono::steady_clock::now();

            if(cudaEventRecord(t_solve_end, stream) != cudaSuccess) { destroy_events(); return false; }

            if(st != CUSOLVER_STATUS_SUCCESS || (singularity >= 0)) { destroy_events(); return false; }

            if(pinned && h_x_d)
            {
                if(cudaMemcpyAsync(h_x_d, d_x_d, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyDeviceToHost, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
            }
            else
            {
                if(cudaMemcpyAsync(x_host, d_x_d, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyDeviceToHost, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
            }

            if(cudaEventRecord(t_d2h_end, stream) != cudaSuccess) { destroy_events(); return false; }
            if(cudaEventSynchronize(t_d2h_end) != cudaSuccess) { destroy_events(); return false; }

            if(pinned && h_x_d)
            {
                std::memcpy(x_host, h_x_d, static_cast<::std::size_t>(n) * sizeof(double));
            }

            float ms_h2d{};
            float ms_solve{};
            float ms_d2h{};
            cudaEventElapsedTime(&ms_h2d, t_h2d_start, t_h2d_end);
            cudaEventElapsedTime(&ms_solve, t_solve_start, t_solve_end);
            cudaEventElapsedTime(&ms_d2h, t_solve_end, t_d2h_end);
            out.h2d_ms = static_cast<double>(ms_h2d);
            out.solve_ms = static_cast<double>(ms_solve);
            out.d2h_ms = static_cast<double>(ms_d2h);
            out.solve_host_ms = ::std::chrono::duration_cast<::std::chrono::duration<double, ::std::milli>>(host_solve1 - host_solve0).count();
            out.solve_total_host_ms = out.h2d_ms + out.solve_host_ms + out.d2h_ms;
            destroy_events();
            return true;
        }

    private:
        bool available{};
        cublasHandle_t cublas_handle{};
        cusparseHandle_t cusparse_handle{};
        cusolverSpHandle_t cusolver_handle{};
        cusparseMatDescr_t descrA{};
        cudaStream_t stream{};

        bool pattern_uploaded{};
        int pattern_n{};
        int pattern_nnz{};

        bool pinned{};
        int pinned_n_capacity_z{};
        int pinned_nnz_capacity_z{};
        int pinned_n_capacity_d{};
        int pinned_nnz_capacity_d{};
        cuDoubleComplex* h_values_z{};
        cuDoubleComplex* h_b_z{};
        cuDoubleComplex* h_x_z{};
        double* h_values_d{};
        double* h_b_d{};
        double* h_x_d{};

        int n_capacity{};
        int nnz_capacity{};

        int* d_row_ptr{};
        int* d_col_ind{};

        int n_capacity_z{};
        int nnz_capacity_z{};
        cuDoubleComplex* d_values_z{};
        cuDoubleComplex* d_b_z{};
        cuDoubleComplex* d_x_z{};

        int n_capacity_d{};
        int nnz_capacity_d{};
        double* d_values_d{};
        double* d_b_d{};
        double* d_x_d{};

        // ILU0 + BiCGSTAB (real-only)
        int ilu_n_d{};
        int ilu_nnz_d{};
        csrilu02Info_t ilu_info_d{};
        void* ilu_buffer_d{};
        int ilu_buffer_bytes_d{};

        // cuSPARSE generic API descriptors for SpMV/SpSV (preferred on CUDA 12+).
        cusparseSpMatDescr_t spmatA_d{};    // A (for SpMV), values = d_values_d
        cusparseSpMatDescr_t spmatL_d{};    // L (for SpSV), values = d_ilu_vals_d
        cusparseSpMatDescr_t spmatU_d{};    // U (for SpSV), values = d_ilu_vals_d
        cusparseSpSVDescr_t spsvL_d{};
        cusparseSpSVDescr_t spsvU_d{};
        cusparseDnVecDescr_t vec_in_d{};
        cusparseDnVecDescr_t vec_out_d{};
        void* spmv_buffer_d{};
        size_t spmv_buffer_bytes_d{};
        void* spsvL_buffer_d{};
        size_t spsvL_buffer_bytes_d{};
        void* spsvU_buffer_d{};
        size_t spsvU_buffer_bytes_d{};

        double* d_ilu_vals_d{};
        double* d_r_d{};
        double* d_rhat_d{};
        double* d_p_d{};
        double* d_v_d{};
        double* d_s_d{};
        double* d_t_d{};
        double* d_y_d{};
        double* d_z_d{};

        csrqrInfo_t csrqr_info_d{};
        void* csrqr_buffer_d{};
        size_t csrqr_buffer_bytes_d{};
        int csrqr_n_d{};
        int csrqr_nnz_d{};
        size_t last_csrqr_internal_bytes_d{};
        size_t last_csrqr_workspace_bytes_d{};

        void cleanup() noexcept
        {
            if(ilu_buffer_d) { cudaFree(ilu_buffer_d); }
            ilu_buffer_d = nullptr;
            ilu_buffer_bytes_d = 0;
            if(d_ilu_vals_d) { cudaFree(d_ilu_vals_d); }
            if(d_r_d) { cudaFree(d_r_d); }
            if(d_rhat_d) { cudaFree(d_rhat_d); }
            if(d_p_d) { cudaFree(d_p_d); }
            if(d_v_d) { cudaFree(d_v_d); }
            if(d_s_d) { cudaFree(d_s_d); }
            if(d_t_d) { cudaFree(d_t_d); }
            if(d_y_d) { cudaFree(d_y_d); }
            if(d_z_d) { cudaFree(d_z_d); }
            d_ilu_vals_d = nullptr;
            d_r_d = nullptr;
            d_rhat_d = nullptr;
            d_p_d = nullptr;
            d_v_d = nullptr;
            d_s_d = nullptr;
            d_t_d = nullptr;
            d_y_d = nullptr;
            d_z_d = nullptr;
            ilu_n_d = 0;
            ilu_nnz_d = 0;

            if(ilu_info_d) { cusparseDestroyCsrilu02Info(ilu_info_d); }
            ilu_info_d = nullptr;
            if(spsvL_d) { cusparseSpSV_destroyDescr(spsvL_d); }
            if(spsvU_d) { cusparseSpSV_destroyDescr(spsvU_d); }
            spsvL_d = nullptr;
            spsvU_d = nullptr;
            if(spmatA_d) { cusparseDestroySpMat(spmatA_d); }
            if(spmatL_d) { cusparseDestroySpMat(spmatL_d); }
            if(spmatU_d) { cusparseDestroySpMat(spmatU_d); }
            spmatA_d = nullptr;
            spmatL_d = nullptr;
            spmatU_d = nullptr;
            if(vec_in_d) { cusparseDestroyDnVec(vec_in_d); }
            if(vec_out_d) { cusparseDestroyDnVec(vec_out_d); }
            vec_in_d = nullptr;
            vec_out_d = nullptr;
            if(spmv_buffer_d) { cudaFree(spmv_buffer_d); }
            if(spsvL_buffer_d) { cudaFree(spsvL_buffer_d); }
            if(spsvU_buffer_d) { cudaFree(spsvU_buffer_d); }
            spmv_buffer_d = nullptr;
            spsvL_buffer_d = nullptr;
            spsvU_buffer_d = nullptr;
            spmv_buffer_bytes_d = 0;
            spsvL_buffer_bytes_d = 0;
            spsvU_buffer_bytes_d = 0;

            if(d_row_ptr) { cudaFree(d_row_ptr); }
            if(d_col_ind) { cudaFree(d_col_ind); }
            if(d_values_z) { cudaFree(d_values_z); }
            if(d_b_z) { cudaFree(d_b_z); }
            if(d_x_z) { cudaFree(d_x_z); }
            if(d_values_d) { cudaFree(d_values_d); }
            if(d_b_d) { cudaFree(d_b_d); }
            if(d_x_d) { cudaFree(d_x_d); }
            if(h_values_z) { cudaFreeHost(h_values_z); }
            if(h_b_z) { cudaFreeHost(h_b_z); }
            if(h_x_z) { cudaFreeHost(h_x_z); }
            if(h_values_d) { cudaFreeHost(h_values_d); }
            if(h_b_d) { cudaFreeHost(h_b_d); }
            if(h_x_d) { cudaFreeHost(h_x_d); }
            d_row_ptr = nullptr;
            d_col_ind = nullptr;
            d_values_z = nullptr;
            d_b_z = nullptr;
            d_x_z = nullptr;
            d_values_d = nullptr;
            d_b_d = nullptr;
            d_x_d = nullptr;
            h_values_z = nullptr;
            h_b_z = nullptr;
            h_x_z = nullptr;
            h_values_d = nullptr;
            h_b_d = nullptr;
            h_x_d = nullptr;
            n_capacity = 0;
            nnz_capacity = 0;
            n_capacity_z = 0;
            nnz_capacity_z = 0;
            n_capacity_d = 0;
            nnz_capacity_d = 0;
            pinned_n_capacity_z = 0;
            pinned_nnz_capacity_z = 0;
            pinned_n_capacity_d = 0;
            pinned_nnz_capacity_d = 0;

            if(csrqr_buffer_d) { cudaFree(csrqr_buffer_d); }
            csrqr_buffer_d = nullptr;
            csrqr_buffer_bytes_d = 0;
            csrqr_n_d = 0;
            csrqr_nnz_d = 0;
            if(csrqr_info_d) { cusolverSpDestroyCsrqrInfo(csrqr_info_d); }
            csrqr_info_d = nullptr;

            if(descrA) { cusparseDestroyMatDescr(descrA); }
            descrA = nullptr;

            if(cusolver_handle) { cusolverSpDestroy(cusolver_handle); }
            cusolver_handle = nullptr;

            if(cusparse_handle) { cusparseDestroy(cusparse_handle); }
            cusparse_handle = nullptr;

            if(cublas_handle) { cublasDestroy(cublas_handle); }
            cublas_handle = nullptr;

            if(stream) { cudaStreamDestroy(stream); }
            stream = nullptr;

            pattern_uploaded = false;
            pattern_n = 0;
            pattern_nnz = 0;
        }

        [[nodiscard]] bool ensure_capacity_common(int n, int nnz) noexcept
        {
            if(n <= n_capacity && nnz <= nnz_capacity) { return true; }

            if(d_row_ptr) { cudaFree(d_row_ptr); }
            if(d_col_ind) { cudaFree(d_col_ind); }

            d_row_ptr = nullptr;
            d_col_ind = nullptr;
            pattern_uploaded = false;
            pattern_n = 0;
            pattern_nnz = 0;

            if(cudaMalloc(reinterpret_cast<void**>(&d_row_ptr), static_cast<::std::size_t>(n + 1) * sizeof(int)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_col_ind), static_cast<::std::size_t>(nnz) * sizeof(int)) != cudaSuccess) { return false; }

            n_capacity = n;
            nnz_capacity = nnz;
            return true;
        }

        [[nodiscard]] bool ensure_capacity_complex(int n, int nnz) noexcept
        {
            if(n <= n_capacity_z && nnz <= nnz_capacity_z) { return true; }
            if(d_values_z) { cudaFree(d_values_z); }
            if(d_b_z) { cudaFree(d_b_z); }
            if(d_x_z) { cudaFree(d_x_z); }
            d_values_z = nullptr;
            d_b_z = nullptr;
            d_x_z = nullptr;
            if(cudaMalloc(reinterpret_cast<void**>(&d_values_z), static_cast<::std::size_t>(nnz) * sizeof(cuDoubleComplex)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_b_z), static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_x_z), static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex)) != cudaSuccess) { return false; }
            n_capacity_z = n;
            nnz_capacity_z = nnz;
            return true;
        }

        [[nodiscard]] bool ensure_capacity_real(int n, int nnz) noexcept
        {
            if(n <= n_capacity_d && nnz <= nnz_capacity_d) { return true; }
            if(d_values_d) { cudaFree(d_values_d); }
            if(d_b_d) { cudaFree(d_b_d); }
            if(d_x_d) { cudaFree(d_x_d); }
            d_values_d = nullptr;
            d_b_d = nullptr;
            d_x_d = nullptr;
            if(cudaMalloc(reinterpret_cast<void**>(&d_values_d), static_cast<::std::size_t>(nnz) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_b_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_x_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            n_capacity_d = n;
            nnz_capacity_d = nnz;
            return true;
        }

        [[nodiscard]] bool ensure_pinned_real(int n, int nnz) noexcept
        {
            if(!pinned) { return true; }
            if(n <= 0 || nnz < 0) { return false; }
            if(n <= pinned_n_capacity_d && nnz <= pinned_nnz_capacity_d) { return true; }

            if(h_values_d) { cudaFreeHost(h_values_d); }
            if(h_b_d) { cudaFreeHost(h_b_d); }
            if(h_x_d) { cudaFreeHost(h_x_d); }
            h_values_d = nullptr;
            h_b_d = nullptr;
            h_x_d = nullptr;
            pinned_n_capacity_d = 0;
            pinned_nnz_capacity_d = 0;

            if(cudaHostAlloc(reinterpret_cast<void**>(&h_values_d), static_cast<::std::size_t>(nnz) * sizeof(double), cudaHostAllocDefault) != cudaSuccess) { pinned = false; return true; }
            if(cudaHostAlloc(reinterpret_cast<void**>(&h_b_d), static_cast<::std::size_t>(n) * sizeof(double), cudaHostAllocDefault) != cudaSuccess) { pinned = false; return true; }
            if(cudaHostAlloc(reinterpret_cast<void**>(&h_x_d), static_cast<::std::size_t>(n) * sizeof(double), cudaHostAllocDefault) != cudaSuccess) { pinned = false; return true; }

            pinned_n_capacity_d = n;
            pinned_nnz_capacity_d = nnz;
            return true;
        }

        [[nodiscard]] bool ensure_ilu_workspace_real(int n, int nnz) noexcept
        {
            if(n <= 0 || nnz < 0) { return false; }
            if(ilu_n_d == n && ilu_nnz_d == nnz && d_ilu_vals_d != nullptr && d_r_d != nullptr) { return true; }

            if(ilu_info_d) { cusparseDestroyCsrilu02Info(ilu_info_d); ilu_info_d = nullptr; }
            if(spsvL_d) { cusparseSpSV_destroyDescr(spsvL_d); spsvL_d = nullptr; }
            if(spsvU_d) { cusparseSpSV_destroyDescr(spsvU_d); spsvU_d = nullptr; }
            if(spmatA_d) { cusparseDestroySpMat(spmatA_d); spmatA_d = nullptr; }
            if(spmatL_d) { cusparseDestroySpMat(spmatL_d); spmatL_d = nullptr; }
            if(spmatU_d) { cusparseDestroySpMat(spmatU_d); spmatU_d = nullptr; }
            if(vec_in_d) { cusparseDestroyDnVec(vec_in_d); vec_in_d = nullptr; }
            if(vec_out_d) { cusparseDestroyDnVec(vec_out_d); vec_out_d = nullptr; }
            if(spmv_buffer_d) { cudaFree(spmv_buffer_d); spmv_buffer_d = nullptr; spmv_buffer_bytes_d = 0; }
            if(spsvL_buffer_d) { cudaFree(spsvL_buffer_d); spsvL_buffer_d = nullptr; spsvL_buffer_bytes_d = 0; }
            if(spsvU_buffer_d) { cudaFree(spsvU_buffer_d); spsvU_buffer_d = nullptr; spsvU_buffer_bytes_d = 0; }
            if(ilu_buffer_d) { cudaFree(ilu_buffer_d); ilu_buffer_d = nullptr; }
            ilu_buffer_bytes_d = 0;

            if(d_ilu_vals_d) { cudaFree(d_ilu_vals_d); d_ilu_vals_d = nullptr; }
            if(d_r_d) { cudaFree(d_r_d); d_r_d = nullptr; }
            if(d_rhat_d) { cudaFree(d_rhat_d); d_rhat_d = nullptr; }
            if(d_p_d) { cudaFree(d_p_d); d_p_d = nullptr; }
            if(d_v_d) { cudaFree(d_v_d); d_v_d = nullptr; }
            if(d_s_d) { cudaFree(d_s_d); d_s_d = nullptr; }
            if(d_t_d) { cudaFree(d_t_d); d_t_d = nullptr; }
            if(d_y_d) { cudaFree(d_y_d); d_y_d = nullptr; }
            if(d_z_d) { cudaFree(d_z_d); d_z_d = nullptr; }

            if(cusparseCreateCsrilu02Info(&ilu_info_d) != CUSPARSE_STATUS_SUCCESS) { return false; }

            if(cudaMalloc(reinterpret_cast<void**>(&d_ilu_vals_d), static_cast<::std::size_t>(nnz) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_r_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_rhat_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_p_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_v_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_s_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_t_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_y_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }
            if(cudaMalloc(reinterpret_cast<void**>(&d_z_d), static_cast<::std::size_t>(n) * sizeof(double)) != cudaSuccess) { return false; }

            // Build generic descriptors (SpMV/SpSV). These are recreated on size change.
            if(cusparseCreateCsr(&spmatA_d,
                                static_cast<int64_t>(n),
                                static_cast<int64_t>(n),
                                static_cast<int64_t>(nnz),
                                d_row_ptr,
                                d_col_ind,
                                d_values_d,
                                CUSPARSE_INDEX_32I,
                                CUSPARSE_INDEX_32I,
                                CUSPARSE_INDEX_BASE_ZERO,
                                CUDA_R_64F) != CUSPARSE_STATUS_SUCCESS)
            {
                return false;
            }
            if(cusparseCreateCsr(&spmatL_d,
                                static_cast<int64_t>(n),
                                static_cast<int64_t>(n),
                                static_cast<int64_t>(nnz),
                                d_row_ptr,
                                d_col_ind,
                                d_ilu_vals_d,
                                CUSPARSE_INDEX_32I,
                                CUSPARSE_INDEX_32I,
                                CUSPARSE_INDEX_BASE_ZERO,
                                CUDA_R_64F) != CUSPARSE_STATUS_SUCCESS)
            {
                return false;
            }
            if(cusparseCreateCsr(&spmatU_d,
                                static_cast<int64_t>(n),
                                static_cast<int64_t>(n),
                                static_cast<int64_t>(nnz),
                                d_row_ptr,
                                d_col_ind,
                                d_ilu_vals_d,
                                CUSPARSE_INDEX_32I,
                                CUSPARSE_INDEX_32I,
                                CUSPARSE_INDEX_BASE_ZERO,
                                CUDA_R_64F) != CUSPARSE_STATUS_SUCCESS)
            {
                return false;
            }
            {
                cusparseFillMode_t fmL{CUSPARSE_FILL_MODE_LOWER};
                cusparseFillMode_t fmU{CUSPARSE_FILL_MODE_UPPER};
                cusparseDiagType_t dtL{CUSPARSE_DIAG_TYPE_UNIT};
                cusparseDiagType_t dtU{CUSPARSE_DIAG_TYPE_NON_UNIT};
                (void)cusparseSpMatSetAttribute(spmatL_d, CUSPARSE_SPMAT_FILL_MODE, &fmL, sizeof(fmL));
                (void)cusparseSpMatSetAttribute(spmatL_d, CUSPARSE_SPMAT_DIAG_TYPE, &dtL, sizeof(dtL));
                (void)cusparseSpMatSetAttribute(spmatU_d, CUSPARSE_SPMAT_FILL_MODE, &fmU, sizeof(fmU));
                (void)cusparseSpMatSetAttribute(spmatU_d, CUSPARSE_SPMAT_DIAG_TYPE, &dtU, sizeof(dtU));
            }
            if(cusparseCreateDnVec(&vec_in_d, static_cast<int64_t>(n), d_y_d, CUDA_R_64F) != CUSPARSE_STATUS_SUCCESS) { return false; }
            if(cusparseCreateDnVec(&vec_out_d, static_cast<int64_t>(n), d_z_d, CUDA_R_64F) != CUSPARSE_STATUS_SUCCESS) { return false; }
            if(cusparseSpSV_createDescr(&spsvL_d) != CUSPARSE_STATUS_SUCCESS) { return false; }
            if(cusparseSpSV_createDescr(&spsvU_d) != CUSPARSE_STATUS_SUCCESS) { return false; }

            ilu_n_d = n;
            ilu_nnz_d = nnz;
            return true;
        }

        [[nodiscard]] bool solve_csr_real_ilu0_bicgstab(int n,
                                                        int nnz,
                                                        int const* row_ptr_host,
                                                        int const* col_ind_host,
                                                        double const* values_host,
                                                        double const* b_host,
                                                        double* x_host,
                                                        timings& out,
                                                        bool copy_pattern) noexcept
        {
            if(!ensure_ilu_workspace_real(n, nnz)) { return false; }
            bool const debug = []() noexcept {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_DEBUG");
                return v != nullptr && (*v == '1' || *v == 'y' || *v == 'Y' || *v == 't' || *v == 'T');
            }();

            auto t_h2d_start = cudaEvent_t{};
            auto t_h2d_end = cudaEvent_t{};
            auto t_solve_start = cudaEvent_t{};
            auto t_solve_end = cudaEvent_t{};
            auto t_d2h_end = cudaEvent_t{};
            if(cudaEventCreate(&t_h2d_start) != cudaSuccess) { return false; }
            if(cudaEventCreate(&t_h2d_end) != cudaSuccess) { cudaEventDestroy(t_h2d_start); return false; }
            if(cudaEventCreate(&t_solve_start) != cudaSuccess)
            {
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }
            if(cudaEventCreate(&t_solve_end) != cudaSuccess)
            {
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }
            if(cudaEventCreate(&t_d2h_end) != cudaSuccess)
            {
                cudaEventDestroy(t_solve_end);
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
                return false;
            }

            auto const destroy_events = [&]() noexcept {
                cudaEventDestroy(t_d2h_end);
                cudaEventDestroy(t_solve_end);
                cudaEventDestroy(t_solve_start);
                cudaEventDestroy(t_h2d_end);
                cudaEventDestroy(t_h2d_start);
            };

            if(cudaEventRecord(t_h2d_start, stream) != cudaSuccess) { destroy_events(); return false; }

            bool const must_copy_pattern{copy_pattern || !pattern_uploaded || pattern_n != n || pattern_nnz != nnz};
            if(must_copy_pattern)
            {
                if(cudaMemcpyAsync(d_row_ptr, row_ptr_host, static_cast<::std::size_t>(n + 1) * sizeof(int), cudaMemcpyHostToDevice, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                if(cudaMemcpyAsync(d_col_ind, col_ind_host, static_cast<::std::size_t>(nnz) * sizeof(int), cudaMemcpyHostToDevice, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                pattern_uploaded = true;
                pattern_n = n;
                pattern_nnz = nnz;
            }

            double const* values_src{values_host};
            double const* b_src{b_host};
            if(pinned && h_values_d && h_b_d)
            {
                std::memcpy(h_values_d, values_host, static_cast<::std::size_t>(nnz) * sizeof(double));
                std::memcpy(h_b_d, b_host, static_cast<::std::size_t>(n) * sizeof(double));
                values_src = h_values_d;
                b_src = h_b_d;
            }

            if(cudaMemcpyAsync(d_values_d, values_src, static_cast<::std::size_t>(nnz) * sizeof(double), cudaMemcpyHostToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }
            if(cudaMemcpyAsync(d_b_d, b_src, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyHostToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }

            // Make a working copy for ILU0 factorization.
            if(cudaMemcpyAsync(d_ilu_vals_d, d_values_d, static_cast<::std::size_t>(nnz) * sizeof(double), cudaMemcpyDeviceToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }

            if(cudaEventRecord(t_h2d_end, stream) != cudaSuccess) { destroy_events(); return false; }

            auto const it_max = []() noexcept -> int
            {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_ITER_MAX");
                if(v == nullptr) { return 200; }
                return static_cast<int>(std::strtol(v, nullptr, 10));
            }();
            auto const tol = []() noexcept -> double
            {
                auto const* v = ::std::getenv("PHY_ENGINE_CUDA_ITER_TOL");
                if(v == nullptr) { return 1e-10; }
                return std::strtod(v, nullptr);
            }();

            auto const host_solve0 = ::std::chrono::steady_clock::now();
            if(cudaEventRecord(t_solve_start, stream) != cudaSuccess) { destroy_events(); return false; }

            // ILU0 factorization + triangular solve analysis.
            int ilu_buf_bytes{};
            if(cusparseDcsrilu02_bufferSize(cusparse_handle, n, nnz, descrA, d_ilu_vals_d, d_row_ptr, d_col_ind, ilu_info_d, &ilu_buf_bytes) !=
               CUSPARSE_STATUS_SUCCESS)
            {
                destroy_events();
                return false;
            }
            int const need_bytes = ilu_buf_bytes;
            if(need_bytes > ilu_buffer_bytes_d)
            {
                if(ilu_buffer_d) { cudaFree(ilu_buffer_d); ilu_buffer_d = nullptr; }
                if(cudaMalloc(&ilu_buffer_d, static_cast<::std::size_t>(need_bytes)) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
                ilu_buffer_bytes_d = need_bytes;
            }

            if(cusparseDcsrilu02_analysis(cusparse_handle, n, nnz, descrA, d_ilu_vals_d, d_row_ptr, d_col_ind, ilu_info_d, CUSPARSE_SOLVE_POLICY_NO_LEVEL, ilu_buffer_d) !=
               CUSPARSE_STATUS_SUCCESS)
            {
                destroy_events();
                return false;
            }
            if(cusparseDcsrilu02(cusparse_handle, n, nnz, descrA, d_ilu_vals_d, d_row_ptr, d_col_ind, ilu_info_d, CUSPARSE_SOLVE_POLICY_NO_LEVEL, ilu_buffer_d) !=
               CUSPARSE_STATUS_SUCCESS)
            {
                destroy_events();
                return false;
            }

            // Generic SpSV/SpMV analysis and buffers (reused across iterations).
            {
                size_t spmv_buf{};
                double const one{1.0};
                double const zero{0.0};
                // Prepare SpMV(A * x) buffers (use v buffer).
                (void)cusparseDnVecSetValues(vec_in_d, d_v_d);
                (void)cusparseDnVecSetValues(vec_out_d, d_t_d);
                if(cusparseSpMV_bufferSize(cusparse_handle,
                                           CUSPARSE_OPERATION_NON_TRANSPOSE,
                                           &one,
                                           spmatA_d,
                                           vec_in_d,
                                           &zero,
                                           vec_out_d,
                                           CUDA_R_64F,
                                           CUSPARSE_SPMV_ALG_DEFAULT,
                                           &spmv_buf) != CUSPARSE_STATUS_SUCCESS)
                {
                    destroy_events();
                    return false;
                }
                if(spmv_buf > spmv_buffer_bytes_d)
                {
                    if(spmv_buffer_d) { cudaFree(spmv_buffer_d); }
                    spmv_buffer_d = nullptr;
                    if(cudaMalloc(&spmv_buffer_d, spmv_buf) != cudaSuccess)
                    {
                        destroy_events();
                        return false;
                    }
                    spmv_buffer_bytes_d = spmv_buf;
                }

                // SpSV buffers (L and U). Use y/z vectors for sizing.
                size_t Lbuf{};
                (void)cusparseDnVecSetValues(vec_in_d, d_p_d);
                (void)cusparseDnVecSetValues(vec_out_d, d_y_d);
                if(cusparseSpSV_bufferSize(cusparse_handle,
                                           CUSPARSE_OPERATION_NON_TRANSPOSE,
                                           &one,
                                           spmatL_d,
                                           vec_in_d,
                                           vec_out_d,
                                           CUDA_R_64F,
                                           CUSPARSE_SPSV_ALG_DEFAULT,
                                           spsvL_d,
                                           &Lbuf) != CUSPARSE_STATUS_SUCCESS)
                {
                    destroy_events();
                    return false;
                }
                if(Lbuf > spsvL_buffer_bytes_d)
                {
                    if(spsvL_buffer_d) { cudaFree(spsvL_buffer_d); }
                    spsvL_buffer_d = nullptr;
                    if(cudaMalloc(&spsvL_buffer_d, Lbuf) != cudaSuccess)
                    {
                        destroy_events();
                        return false;
                    }
                    spsvL_buffer_bytes_d = Lbuf;
                }
                if(cusparseSpSV_analysis(cusparse_handle,
                                         CUSPARSE_OPERATION_NON_TRANSPOSE,
                                         &one,
                                         spmatL_d,
                                         vec_in_d,
                                         vec_out_d,
                                         CUDA_R_64F,
                                         CUSPARSE_SPSV_ALG_DEFAULT,
                                         spsvL_d,
                                         spsvL_buffer_d) != CUSPARSE_STATUS_SUCCESS)
                {
                    destroy_events();
                    return false;
                }

                size_t Ubuf{};
                (void)cusparseDnVecSetValues(vec_in_d, d_y_d);
                (void)cusparseDnVecSetValues(vec_out_d, d_z_d);
                if(cusparseSpSV_bufferSize(cusparse_handle,
                                           CUSPARSE_OPERATION_NON_TRANSPOSE,
                                           &one,
                                           spmatU_d,
                                           vec_in_d,
                                           vec_out_d,
                                           CUDA_R_64F,
                                           CUSPARSE_SPSV_ALG_DEFAULT,
                                           spsvU_d,
                                           &Ubuf) != CUSPARSE_STATUS_SUCCESS)
                {
                    destroy_events();
                    return false;
                }
                if(Ubuf > spsvU_buffer_bytes_d)
                {
                    if(spsvU_buffer_d) { cudaFree(spsvU_buffer_d); }
                    spsvU_buffer_d = nullptr;
                    if(cudaMalloc(&spsvU_buffer_d, Ubuf) != cudaSuccess)
                    {
                        destroy_events();
                        return false;
                    }
                    spsvU_buffer_bytes_d = Ubuf;
                }
                if(cusparseSpSV_analysis(cusparse_handle,
                                         CUSPARSE_OPERATION_NON_TRANSPOSE,
                                         &one,
                                         spmatU_d,
                                         vec_in_d,
                                         vec_out_d,
                                         CUDA_R_64F,
                                         CUSPARSE_SPSV_ALG_DEFAULT,
                                         spsvU_d,
                                         spsvU_buffer_d) != CUSPARSE_STATUS_SUCCESS)
                {
                    destroy_events();
                    return false;
                }
            }

            double b_norm{};
            if(cublasDnrm2(cublas_handle, n, d_b_d, 1, &b_norm) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
            if(b_norm == 0.0) { b_norm = 1.0; }

            // x = 0, r = b, rhat = r
            if(cudaMemsetAsync(d_x_d, 0, static_cast<::std::size_t>(n) * sizeof(double), stream) != cudaSuccess) { destroy_events(); return false; }
            if(cudaMemcpyAsync(d_r_d, d_b_d, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyDeviceToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }
            if(cudaMemcpyAsync(d_rhat_d, d_r_d, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyDeviceToDevice, stream) != cudaSuccess)
            {
                destroy_events();
                return false;
            }
            if(cudaMemsetAsync(d_p_d, 0, static_cast<::std::size_t>(n) * sizeof(double), stream) != cudaSuccess) { destroy_events(); return false; }
            if(cudaMemsetAsync(d_v_d, 0, static_cast<::std::size_t>(n) * sizeof(double), stream) != cudaSuccess) { destroy_events(); return false; }

            auto const apply_precond = [&](double const* rhs, double* outv) noexcept -> bool
            {
                double const one{1.0};
                (void)cusparseDnVecSetValues(vec_in_d, const_cast<double*>(rhs));
                (void)cusparseDnVecSetValues(vec_out_d, d_y_d);
                if(cusparseSpSV_solve(cusparse_handle,
                                      CUSPARSE_OPERATION_NON_TRANSPOSE,
                                      &one,
                                      spmatL_d,
                                      vec_in_d,
                                      vec_out_d,
                                      CUDA_R_64F,
                                      CUSPARSE_SPSV_ALG_DEFAULT,
                                      spsvL_d) != CUSPARSE_STATUS_SUCCESS)
                {
                    return false;
                }
                (void)cusparseDnVecSetValues(vec_in_d, d_y_d);
                (void)cusparseDnVecSetValues(vec_out_d, outv);
                if(cusparseSpSV_solve(cusparse_handle,
                                      CUSPARSE_OPERATION_NON_TRANSPOSE,
                                      &one,
                                      spmatU_d,
                                      vec_in_d,
                                      vec_out_d,
                                      CUDA_R_64F,
                                      CUSPARSE_SPSV_ALG_DEFAULT,
                                      spsvU_d) != CUSPARSE_STATUS_SUCCESS)
                {
                    return false;
                }
                return true;
            };

            auto const spmv = [&](double const* xin, double* yout) noexcept -> bool
            {
                double const one{1.0};
                double const zero{0.0};
                (void)cusparseDnVecSetValues(vec_in_d, const_cast<double*>(xin));
                (void)cusparseDnVecSetValues(vec_out_d, yout);
                if(cusparseSpMV(cusparse_handle,
                                CUSPARSE_OPERATION_NON_TRANSPOSE,
                                &one,
                                spmatA_d,
                                vec_in_d,
                                &zero,
                                vec_out_d,
                                CUDA_R_64F,
                                CUSPARSE_SPMV_ALG_DEFAULT,
                                spmv_buffer_d) != CUSPARSE_STATUS_SUCCESS)
                {
                    return false;
                }
                return true;
            };

            double rho_old{1.0};
            double alpha{1.0};
            double omega{1.0};

            int it{};
            double r_norm{};
            for(; it < it_max; ++it)
            {
                double rho_new{};
                if(cublasDdot(cublas_handle, n, d_rhat_d, 1, d_r_d, 1, &rho_new) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if(rho_new == 0.0) { break; }

                if(it == 0)
                {
                    if(cublasDcopy(cublas_handle, n, d_r_d, 1, d_p_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                }
                else
                {
                    double const beta{(rho_new / rho_old) * (alpha / omega)};
                    // p = r + beta * (p - omega*v)
                    double const neg_omega{-omega};
                    if(cublasDaxpy(cublas_handle, n, &neg_omega, d_v_d, 1, d_p_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                    if(cublasDscal(cublas_handle, n, &beta, d_p_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                    double const one{1.0};
                    if(cublasDaxpy(cublas_handle, n, &one, d_r_d, 1, d_p_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                }

                // y = M^{-1} p
                if(!apply_precond(d_p_d, d_y_d)) { destroy_events(); return false; }
                // v = A*y
                if(!spmv(d_y_d, d_v_d)) { destroy_events(); return false; }

                double denom{};
                if(cublasDdot(cublas_handle, n, d_rhat_d, 1, d_v_d, 1, &denom) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if(denom == 0.0) { break; }
                alpha = rho_new / denom;

                // s = r - alpha*v
                if(cublasDcopy(cublas_handle, n, d_r_d, 1, d_s_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                double const neg_alpha{-alpha};
                if(cublasDaxpy(cublas_handle, n, &neg_alpha, d_v_d, 1, d_s_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }

                double s_norm{};
                if(cublasDnrm2(cublas_handle, n, d_s_d, 1, &s_norm) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if((s_norm / b_norm) < tol)
                {
                    if(cublasDaxpy(cublas_handle, n, &alpha, d_y_d, 1, d_x_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                    r_norm = s_norm;
                    break;
                }

                // z = M^{-1} s
                if(!apply_precond(d_s_d, d_z_d)) { destroy_events(); return false; }
                // t = A*z
                if(!spmv(d_z_d, d_t_d)) { destroy_events(); return false; }

                double tt{};
                double ts{};
                if(cublasDdot(cublas_handle, n, d_t_d, 1, d_t_d, 1, &tt) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if(cublasDdot(cublas_handle, n, d_t_d, 1, d_s_d, 1, &ts) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if(tt == 0.0) { break; }
                omega = ts / tt;
                if(omega == 0.0) { break; }

                // x = x + alpha*y + omega*z
                if(cublasDaxpy(cublas_handle, n, &alpha, d_y_d, 1, d_x_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if(cublasDaxpy(cublas_handle, n, &omega, d_z_d, 1, d_x_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }

                // r = s - omega*t
                if(cublasDcopy(cublas_handle, n, d_s_d, 1, d_r_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                double const neg_omega{-omega};
                if(cublasDaxpy(cublas_handle, n, &neg_omega, d_t_d, 1, d_r_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }

                if(cublasDnrm2(cublas_handle, n, d_r_d, 1, &r_norm) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if((r_norm / b_norm) < tol) { break; }

                rho_old = rho_new;
            }

            // Compute true residual r = b - A*x and its norm for correctness reporting.
            {
                // r := b
                if(cublasDcopy(cublas_handle, n, d_b_d, 1, d_r_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                // v := A*x
                if(!spmv(d_x_d, d_v_d)) { destroy_events(); return false; }
                // r := r - v
                double const minus_one{-1.0};
                if(cublasDaxpy(cublas_handle, n, &minus_one, d_v_d, 1, d_r_d, 1) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
                if(cublasDnrm2(cublas_handle, n, d_r_d, 1, &r_norm) != CUBLAS_STATUS_SUCCESS) { destroy_events(); return false; }
            }

            if(pinned && h_x_d)
            {
                if(cudaMemcpyAsync(h_x_d, d_x_d, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyDeviceToHost, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
            }
            else
            {
                if(cudaMemcpyAsync(x_host, d_x_d, static_cast<::std::size_t>(n) * sizeof(double), cudaMemcpyDeviceToHost, stream) != cudaSuccess)
                {
                    destroy_events();
                    return false;
                }
            }

            if(cudaEventRecord(t_solve_end, stream) != cudaSuccess) { destroy_events(); return false; }
            if(cudaEventRecord(t_d2h_end, stream) != cudaSuccess) { destroy_events(); return false; }
            if(cudaEventSynchronize(t_d2h_end) != cudaSuccess) { destroy_events(); return false; }

            if(pinned && h_x_d)
            {
                std::memcpy(x_host, h_x_d, static_cast<::std::size_t>(n) * sizeof(double));
            }

            auto const host_solve1 = ::std::chrono::steady_clock::now();

            float ms_h2d{};
            float ms_solve{};
            float ms_d2h{};
            cudaEventElapsedTime(&ms_h2d, t_h2d_start, t_h2d_end);
            cudaEventElapsedTime(&ms_solve, t_solve_start, t_solve_end);
            cudaEventElapsedTime(&ms_d2h, t_solve_end, t_d2h_end);
            out.h2d_ms = static_cast<double>(ms_h2d);
            out.solve_ms = static_cast<double>(ms_solve);
            out.d2h_ms = static_cast<double>(ms_d2h);
            out.solve_host_ms = ::std::chrono::duration_cast<::std::chrono::duration<double, ::std::milli>>(host_solve1 - host_solve0).count();
            out.solve_total_host_ms = out.h2d_ms + out.solve_host_ms + out.d2h_ms;

            destroy_events();
            if(debug)
            {
                std::fprintf(stderr,
                             "[phy_engine][cuda][ilu0_bicgstab] n=%d nnz=%d it=%d it_max=%d r_rel=%.3e tol=%.3e (true_resid)\n",
                             n,
                             nnz,
                             it,
                             it_max,
                             (b_norm != 0.0) ? (r_norm / b_norm) : 0.0,
                             tol);
            }
            return (r_norm / b_norm) < tol * 10;
        }

        [[nodiscard]] bool ensure_pinned_complex(int n, int nnz) noexcept
        {
            if(!pinned) { return true; }
            if(n <= 0 || nnz < 0) { return false; }
            if(n <= pinned_n_capacity_z && nnz <= pinned_nnz_capacity_z) { return true; }

            if(h_values_z) { cudaFreeHost(h_values_z); }
            if(h_b_z) { cudaFreeHost(h_b_z); }
            if(h_x_z) { cudaFreeHost(h_x_z); }
            h_values_z = nullptr;
            h_b_z = nullptr;
            h_x_z = nullptr;
            pinned_n_capacity_z = 0;
            pinned_nnz_capacity_z = 0;

            if(cudaHostAlloc(reinterpret_cast<void**>(&h_values_z), static_cast<::std::size_t>(nnz) * sizeof(cuDoubleComplex), cudaHostAllocDefault) != cudaSuccess) { pinned = false; return true; }
            if(cudaHostAlloc(reinterpret_cast<void**>(&h_b_z), static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaHostAllocDefault) != cudaSuccess) { pinned = false; return true; }
            if(cudaHostAlloc(reinterpret_cast<void**>(&h_x_z), static_cast<::std::size_t>(n) * sizeof(cuDoubleComplex), cudaHostAllocDefault) != cudaSuccess) { pinned = false; return true; }

            pinned_n_capacity_z = n;
            pinned_nnz_capacity_z = nnz;
            return true;
        }

        [[nodiscard]] cusolverStatus_t prepare_csrqr_real(int n, int nnz) noexcept
        {
            if(csrqr_info_d == nullptr)
            {
                auto const st{cusolverSpCreateCsrqrInfo(&csrqr_info_d)};
                if(st != CUSOLVER_STATUS_SUCCESS) { csrqr_info_d = nullptr; return st; }
            }

            if(csrqr_n_d != n || csrqr_nnz_d != nnz)
            {
                auto const st{cusolverSpXcsrqrAnalysisBatched(cusolver_handle, n, n, nnz, descrA, d_row_ptr, d_col_ind, csrqr_info_d)};
                if(st != CUSOLVER_STATUS_SUCCESS) { return st; }
                csrqr_n_d = n;
                csrqr_nnz_d = nnz;
            }

            size_t internal_bytes{};
            size_t workspace_bytes{};
            auto const st2{cusolverSpDcsrqrBufferInfoBatched(
                cusolver_handle, n, n, nnz, descrA, d_values_d, d_row_ptr, d_col_ind, 1, csrqr_info_d, &internal_bytes, &workspace_bytes)};
            if(st2 != CUSOLVER_STATUS_SUCCESS) { return st2; }

            last_csrqr_internal_bytes_d = internal_bytes;
            last_csrqr_workspace_bytes_d = workspace_bytes;

            size_t const needed{internal_bytes + workspace_bytes};
            if(needed > csrqr_buffer_bytes_d)
            {
                if(csrqr_buffer_d) { cudaFree(csrqr_buffer_d); }
                csrqr_buffer_d = nullptr;
                csrqr_buffer_bytes_d = 0;
                if(needed != 0)
                {
                    if(cudaMalloc(&csrqr_buffer_d, needed) != cudaSuccess) { return CUSOLVER_STATUS_ALLOC_FAILED; }
                    csrqr_buffer_bytes_d = needed;
                }
            }
            return CUSOLVER_STATUS_SUCCESS;
        }
    };

}  // namespace phy_engine::solver

#endif
