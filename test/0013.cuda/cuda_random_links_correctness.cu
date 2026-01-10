#if !defined(__CUDACC__) && !defined(__NVCC__) && !defined(__CUDA__)
#error "This test must be compiled with a CUDA compiler (nvcc or clang++ -x cuda)."
#endif

#if !defined(__CUDA_ARCH__)
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <random>
#include <vector>

#include <cuda_runtime.h>

#include <fast_io/fast_io.h>
#include <phy_engine/phy_engine.h>

namespace
{
    inline double absd(double x) noexcept { return x < 0.0 ? -x : x; }

    void build_random_links_case(::phy_engine::circult& c,
                                 std::size_t nodes,
                                 std::size_t links,
                                 std::uint64_t seed,
                                 double r_chain,
                                 double r_link,
                                 double vdc) noexcept
    {
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl{c.get_netlist()};

        std::vector<::phy_engine::model::node_t*> ns{};
        ns.reserve(nodes);
        for(std::size_t i{}; i < nodes; ++i)
        {
            auto& n{create_node(nl)};
            ns.push_back(std::addressof(n));
        }

        {
            auto [Rg, _]{add_model(nl, ::phy_engine::model::resistance{.r = r_chain})};
            add_to_node(nl, *Rg, 0, nl.ground_node);
            add_to_node(nl, *Rg, 1, *ns[0]);
        }
        for(std::size_t i{}; i + 1 < nodes; ++i)
        {
            auto [R, _]{add_model(nl, ::phy_engine::model::resistance{.r = r_chain})};
            add_to_node(nl, *R, 0, *ns[i]);
            add_to_node(nl, *R, 1, *ns[i + 1]);
        }

        {
            auto [V, _]{add_model(nl, ::phy_engine::model::VDC{.V = vdc})};
            add_to_node(nl, *V, 0, *ns[nodes - 1]);
            add_to_node(nl, *V, 1, nl.ground_node);
        }

        std::mt19937_64 rng{seed};
        std::uniform_int_distribution<std::size_t> dist(0, nodes - 1);
        for(std::size_t i{}; i < links; ++i)
        {
            auto a{dist(rng)};
            auto b{dist(rng)};
            while(b == a) { b = dist(rng); }
            auto [R, _]{add_model(nl, ::phy_engine::model::resistance{.r = r_link})};
            add_to_node(nl, *R, 0, *ns[a]);
            add_to_node(nl, *R, 1, *ns[b]);
        }
    }
}  // namespace

int main()
{
    int device_count{};
    auto const st = cudaGetDeviceCount(&device_count);
    if(st != cudaSuccess || device_count <= 0)
    {
        ::fast_io::io::perr("cuda_random_links_correctness: no CUDA device (", ::fast_io::mnp::os_c_str(cudaGetErrorString(st)), ")\n");
        return 77;
    }

    constexpr std::size_t nodes = 2048;
    constexpr std::size_t links = 8192;
    constexpr std::uint64_t seed = 1;

    ::phy_engine::circult c{};
    build_random_links_case(c, nodes, links, seed, /*r_chain=*/1000.0, /*r_link=*/1000.0, /*vdc=*/1.0);
    c.prepare();

    if(!c.cuda_solver.is_available())
    {
        ::fast_io::io::perr("cuda_random_links_correctness: CUDA solver unavailable\n");
        return 77;
    }

    // CPU solve (Eigen)
    c.set_cuda_policy(::phy_engine::circult::cuda_solve_policy::force_cpu);
    if(!c.solve_once())
    {
        ::fast_io::io::perr("cuda_random_links_correctness: CPU solve_once failed\n");
        return 2;
    }

    std::vector<double> cpu_v(static_cast<std::size_t>(c.node_counter));
    for(auto* const n: c.size_t_to_node_p)
    {
        if(n->node_index < cpu_v.size()) { cpu_v[n->node_index] = n->node_information.an.voltage.real(); }
    }

    // CUDA solve
    c.set_cuda_policy(::phy_engine::circult::cuda_solve_policy::force_cuda);
    c.set_cuda_node_threshold(0);
    if(!c.solve_once())
    {
        ::fast_io::io::perr("cuda_random_links_correctness: CUDA solve_once failed\n");
        return 3;
    }

    double max_abs_diff{};
    for(auto* const n: c.size_t_to_node_p)
    {
        if(n->node_index >= cpu_v.size()) { continue; }
        double const dv = n->node_information.an.voltage.real() - cpu_v[n->node_index];
        double const a = absd(dv);
        if(a > max_abs_diff) { max_abs_diff = a; }
    }

    if(!(max_abs_diff < 1e-6))
    {
        ::fast_io::io::perr("cuda_random_links_correctness: max_abs_diff=", max_abs_diff, " (expected < 1e-6)\n");
        return 4;
    }

    return 0;
}
#endif

