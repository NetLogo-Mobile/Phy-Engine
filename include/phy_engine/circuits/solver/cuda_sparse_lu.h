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

            other.available = false;
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

        csrqrInfo_t csrqr_info_d{};
        void* csrqr_buffer_d{};
        size_t csrqr_buffer_bytes_d{};
        int csrqr_n_d{};
        int csrqr_nnz_d{};
        size_t last_csrqr_internal_bytes_d{};
        size_t last_csrqr_workspace_bytes_d{};

        void cleanup() noexcept
        {
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
