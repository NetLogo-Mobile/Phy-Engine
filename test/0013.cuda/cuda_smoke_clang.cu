#include <cmath>
#include <complex>
#include <cstdio>
#include <cstdlib>
#
#include <cuda_runtime.h>
#
#include <phy_engine/circuits/solver/cuda_sparse_lu.h>
#
#if !defined(__CUDA__)
static_assert(false, "This file must be compiled in CUDA mode (clang++ -x cuda).");
#endif
#
namespace
{
#if !defined(__CUDA_ARCH__)
    [[nodiscard]] bool check_cuda(cudaError_t st, char const* what)
    {
        if(st == cudaSuccess) { return true; }
        std::fprintf(stderr, "%s failed: %s\n", what, cudaGetErrorString(st));
        return false;
    }
#
#endif
    __global__ void increment_kernel(int* value) { *value += 1; }
}  // namespace
#
#if !defined(__CUDA_ARCH__)
int main()
{
    int device_count{};
    auto const dc_st{cudaGetDeviceCount(&device_count)};
    if(dc_st != cudaSuccess)
    {
        std::fprintf(stderr, "cudaGetDeviceCount failed: %s\n", cudaGetErrorString(dc_st));
        return 77;
    }
    if(device_count <= 0)
    {
        std::fprintf(stderr, "No CUDA devices found.\n");
        return 77;
    }
#
    cudaDeviceProp prop{};
    if(!check_cuda(cudaGetDeviceProperties(&prop, 0), "cudaGetDeviceProperties")) { return 77; }
    std::fprintf(stdout, "GPU[0]=%s, cc=%d.%d\n", prop.name, prop.major, prop.minor);
    if(!(prop.major == 7 && prop.minor == 0))
    {
        std::fprintf(stdout, "Note: expected cc=7.0 (sm_70), but got cc=%d.%d\n", prop.major, prop.minor);
    }
#
    int host_value{41};
    int* device_value{};
    if(!check_cuda(cudaMalloc(reinterpret_cast<void**>(&device_value), sizeof(int)), "cudaMalloc")) { return 77; }
    if(!check_cuda(cudaMemcpy(device_value, &host_value, sizeof(int), cudaMemcpyHostToDevice), "cudaMemcpy(H2D)")) { return 77; }
#
    increment_kernel<<<1, 1>>>(device_value);
    if(!check_cuda(cudaGetLastError(), "kernel launch")) { return 77; }
    if(!check_cuda(cudaDeviceSynchronize(), "cudaDeviceSynchronize")) { return 77; }
#
    if(!check_cuda(cudaMemcpy(&host_value, device_value, sizeof(int), cudaMemcpyDeviceToHost), "cudaMemcpy(D2H)")) { return 77; }
    if(!check_cuda(cudaFree(device_value), "cudaFree")) { return 77; }
#
    if(host_value != 42)
    {
        std::fprintf(stderr, "Kernel smoke test failed: expected 42, got %d\n", host_value);
        return 2;
    }
#
    phy_engine::solver::cuda_sparse_lu solver{};
    if(!solver.is_available())
    {
        std::fprintf(stderr, "phy_engine::solver::cuda_sparse_lu is not available (CUDA runtime / cuSolver init failed).\n");
        return 3;
    }
#
    constexpr int n{2};
    constexpr int nnz{2};
    int const row_ptr[n + 1]{0, 1, 2};
    int const col_ind[nnz]{0, 1};
    std::complex<double> const values[nnz]{{1.0, 0.0}, {1.0, 0.0}};
    std::complex<double> const b[n]{{3.0, 0.0}, {4.0, 0.0}};
    std::complex<double> x[n]{};
#
    if(!solver.solve_csr(n, nnz, row_ptr, col_ind, values, b, x))
    {
        std::fprintf(stderr, "cuda_sparse_lu.solve_csr failed.\n");
        return 4;
    }
#
    auto const err0{std::abs(x[0] - b[0])};
    auto const err1{std::abs(x[1] - b[1])};
    if(!(err0 < 1e-9 && err1 < 1e-9))
    {
        std::fprintf(stderr, "cuda_sparse_lu result mismatch: x=[%.17g%+.17gi, %.17g%+.17gi]\n",
                     x[0].real(),
                     x[0].imag(),
                     x[1].real(),
                     x[1].imag());
        return 5;
    }
#
    std::fprintf(stdout, "CUDA clang smoke test OK.\n");
    return 0;
}
#endif
