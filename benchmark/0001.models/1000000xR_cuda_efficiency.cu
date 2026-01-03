#if !defined(__CUDA__)
#error "This benchmark must be compiled with Clang CUDA mode (clang++ -x cuda)."
#endif
#
#if !defined(__CUDA_ARCH__)
#include <chrono>
#include <cstring>
#include <cstddef>
#
#include <cuda_runtime.h>
#
#include <fast_io/fast_io.h>
#include <phy_engine/phy_engine.h>
#
namespace
{
    [[nodiscard]] bool starts_with(char const* s, char const* prefix) noexcept
    {
        for(; *prefix; ++s, ++prefix)
        {
            if(*s != *prefix) { return false; }
        }
        return true;
    }
#
    [[nodiscard]] bool parse_u64(char const* s, std::size_t& out) noexcept
    {
        if(s == nullptr || *s == '\0') { return false; }
        std::size_t v{};
        for(; *s; ++s)
        {
            auto const ch{static_cast<unsigned char>(*s)};
            if(ch < static_cast<unsigned char>('0') || ch > static_cast<unsigned char>('9')) { return false; }
            v = v * 10 + static_cast<std::size_t>(ch - static_cast<unsigned char>('0'));
        }
        out = v;
        return true;
    }
#
    struct bench_config
    {
        std::size_t warmup{1};
        std::size_t iters{5};
    };
#
    [[nodiscard]] bench_config parse_args(int argc, char** argv)
    {
        bench_config cfg{};
        for(int i{1}; i < argc; ++i)
        {
            char const* a{argv[i]};
            if(a == nullptr) { continue; }
            if(starts_with(a, "--warmup="))
            {
                std::size_t v{};
                if(parse_u64(a + 9, v)) { cfg.warmup = v; }
            }
            else if(starts_with(a, "--iters="))
            {
                std::size_t v{};
                if(parse_u64(a + 8, v)) { cfg.iters = v; }
            }
            else if(!std::strcmp(a, "--help"))
            {
                ::fast_io::io::perr("Usage: 1000000xR_cuda_efficiency [--warmup=N] [--iters=N]\n");
                ::fast_io::fast_terminate();
            }
        }
        return cfg;
    }
}  // namespace
#
int main(int argc, char** argv)
{
    auto const cfg{parse_args(argc, argv)};
#
    int device_count{};
    auto const dc_st{cudaGetDeviceCount(&device_count)};
    if(dc_st != cudaSuccess || device_count <= 0)
    {
        ::fast_io::io::perr("No CUDA device visible (", ::fast_io::mnp::os_c_str(cudaGetErrorString(dc_st)), ").\n");
        return 77;
    }
#
    cudaDeviceProp prop{};
    if(cudaGetDeviceProperties(&prop, 0) == cudaSuccess)
    {
        ::fast_io::io::perr("GPU[0]=", prop.name, ", cc=", prop.major, ".", prop.minor, "\n");
    }
#
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::DC);
#
    auto& nl{c.get_netlist()};
#
    auto [R1, R1_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 10.0})};
    auto [R2, R2_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
    auto [R4, R4_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 20.0})};
#
    auto [VDC, VDC_pos]{add_model(nl, ::phy_engine::model::VDC{.V = 3.0})};
    auto& node1{nl.ground_node};
    add_to_node(nl, *R1, 1, node1);
    add_to_node(nl, *R2, 0, node1);
    ::phy_engine::model::model_base* model{R2};
#
    for(::std::size_t i = 0; i < 1000000; i++)
    {
        auto [R3, R3_pos]{add_model(nl, ::phy_engine::model::resistance{.r = 30.0})};
        auto& n{create_node(nl)};
#
        add_to_node(nl, *R3, 0, n);
        add_to_node(nl, *model, 1, n);
        model = R3;
    }
#
    auto& node2{create_node(nl)};
    add_to_node(nl, *VDC, 0, node2);
    add_to_node(nl, *R1, 0, node2);
#
    auto& node4{create_node(nl)};
    add_to_node(nl, *VDC, 1, node4);
    add_to_node(nl, *model, 1, node4);
#
    ::fast_io::io::perr("prepare...\n");
    c.prepare();
#
    if(!c.cuda_solver.is_available())
    {
        ::fast_io::io::perr("CUDA solver unavailable (cudart/cusolver init failed).\n");
        return 77;
    }
    if(c.node_counter < c.cuda_node_threshold)
    {
        ::fast_io::io::perr("node_counter=", c.node_counter, " < cuda_node_threshold=", c.cuda_node_threshold, "\n");
        return 2;
    }
#
    ::fast_io::io::perr("node_counter=", c.node_counter, ", branch_counter=", c.branch_counter, "\n");
#
    ::fast_io::io::perr("warmup=", cfg.warmup, ", iters=", cfg.iters, "\n");
#
    for(::std::size_t i{}; i < cfg.warmup; ++i)
    {
        if(!c.solve_once())
        {
            ::fast_io::io::perr("warmup solve failed at iter=", i, "\n");
            return 3;
        }
    }
#
    auto const nnz_size = [&]() -> std::size_t
    {
        std::size_t nnz{};
        for(auto const& row: c.mna.A) { nnz += row.second.size(); }
        return nnz;
    }();
    ::fast_io::io::perr("nnz=", nnz_size, "\n");
#
    auto const t0{std::chrono::steady_clock::now()};
    for(::std::size_t i{}; i < cfg.iters; ++i)
    {
        if(!c.solve_once())
        {
            ::fast_io::io::perr("timed solve failed at iter=", i, "\n");
            return 4;
        }
    }
    auto const t1{std::chrono::steady_clock::now()};
#
    auto const elapsed_ns{std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()};
    double const elapsed_s{static_cast<double>(elapsed_ns) * 1e-9};
    double const avg_ms{(elapsed_s / static_cast<double>(cfg.iters)) * 1e3};
    double const solves_per_s{static_cast<double>(cfg.iters) / elapsed_s};
#
    ::fast_io::io::perr("total_s=", elapsed_s, ", avg_ms=", avg_ms, ", solves_per_s=", solves_per_s, "\n");
    return 0;
}
#endif
