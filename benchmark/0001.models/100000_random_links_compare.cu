#if !defined(__CUDACC__) && !defined(__NVCC__) && !defined(__CUDA__)
#error "This benchmark must be compiled with a CUDA compiler (nvcc or clang++ -x cuda)."
#endif

#if !defined(__CUDA_ARCH__)
#include <chrono>
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
    [[nodiscard]] bool starts_with(char const* s, char const* prefix) noexcept
    {
        for(; *prefix; ++s, ++prefix)
        {
            if(*s != *prefix) { return false; }
        }
        return true;
    }

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

    [[nodiscard]] bool parse_u64_list(char const* s, std::vector<std::size_t>& out) noexcept
    {
        out.clear();
        if(s == nullptr || *s == '\0') { return false; }
        std::size_t v{};
        bool has_digit{};
        for(; *s; ++s)
        {
            auto const ch{static_cast<unsigned char>(*s)};
            if(ch >= static_cast<unsigned char>('0') && ch <= static_cast<unsigned char>('9'))
            {
                has_digit = true;
                v = v * 10 + static_cast<std::size_t>(ch - static_cast<unsigned char>('0'));
            }
            else if(ch == static_cast<unsigned char>(','))
            {
                if(!has_digit) { return false; }
                out.push_back(v);
                v = 0;
                has_digit = false;
            }
            else
            {
                return false;
            }
        }
        if(!has_digit) { return false; }
        out.push_back(v);
        return !out.empty();
    }

    struct bench_config
    {
        std::size_t nodes{100000};
        std::vector<std::size_t> links{10, 100, 1000, 10000};
        std::uint64_t seed{1};
        std::size_t warmup{1};
        std::size_t iters{5};
        double r_chain{1000.0};
        double r_link{1000.0};
        double vdc{1.0};
        double eps{1e-6};
    };

    [[nodiscard]] bench_config parse_args(int argc, char** argv)
    {
        bench_config cfg{};
        for(int i{1}; i < argc; ++i)
        {
            char const* a{argv[i]};
            if(a == nullptr) { continue; }
            if(starts_with(a, "--nodes="))
            {
                std::size_t v{};
                if(parse_u64(a + 8, v)) { cfg.nodes = v; }
            }
            else if(starts_with(a, "--links="))
            {
                (void)parse_u64_list(a + 8, cfg.links);
            }
            else if(starts_with(a, "--seed="))
            {
                std::size_t v{};
                if(parse_u64(a + 7, v)) { cfg.seed = static_cast<std::uint64_t>(v); }
            }
            else if(starts_with(a, "--warmup="))
            {
                std::size_t v{};
                if(parse_u64(a + 9, v)) { cfg.warmup = v; }
            }
            else if(starts_with(a, "--iters="))
            {
                std::size_t v{};
                if(parse_u64(a + 8, v)) { cfg.iters = v; }
            }
            else if(starts_with(a, "--eps="))
            {
                cfg.eps = std::strtod(a + 6, nullptr);
            }
            else if(!std::strcmp(a, "--help"))
            {
                ::fast_io::io::perr(
                    "Usage: 100000_random_links_compare [--nodes=N] [--links=10,100,1000,10000] [--seed=S] [--warmup=N] [--iters=N] [--eps=1e-6]\n");
                ::fast_io::fast_terminate();
            }
        }
        return cfg;
    }

    inline double absd(double x) noexcept { return x < 0.0 ? -x : x; }

    void build_random_links_case(::phy_engine::circult& c, std::size_t nodes, std::size_t links, std::uint64_t seed, double r_chain, double r_link, double vdc)
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

        // Ensure connectivity to ground: a chain from ground -> n0 -> ... -> n_{N-1}.
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

        // Excitation: set last node voltage w.r.t. ground.
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

    [[nodiscard]] std::size_t nnz_from_mna(::phy_engine::circult const& c) noexcept
    {
        std::size_t nnz{};
        for(auto const& row: c.mna.A) { nnz += row.size(); }
        return nnz;
    }

    struct run_stats
    {
        double avg_ms{};
        double solves_per_s{};
    };

    [[nodiscard]] run_stats run_solve_loops(::phy_engine::circult& c, std::size_t warmup, std::size_t iters)
    {
        for(std::size_t i{}; i < warmup; ++i)
        {
            if(!c.solve_once()) { return {}; }
        }

        auto const t0{std::chrono::steady_clock::now()};
        for(std::size_t i{}; i < iters; ++i)
        {
            if(!c.solve_once()) { return {}; }
        }
        auto const t1{std::chrono::steady_clock::now()};
        auto const elapsed_ns{std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count()};
        double const elapsed_s{static_cast<double>(elapsed_ns) * 1e-9};
        run_stats st{};
        st.avg_ms = (elapsed_s / static_cast<double>(iters)) * 1e3;
        st.solves_per_s = static_cast<double>(iters) / elapsed_s;
        return st;
    }

    void snapshot_solution(::phy_engine::circult const& c, std::vector<double>& node_v, std::vector<double>& branch_i)
    {
        node_v.assign(c.node_counter, 0.0);
        for(auto* const n: c.size_t_to_node_p)
        {
            if(n->node_index < node_v.size()) { node_v[n->node_index] = n->node_information.an.voltage.real(); }
        }

        branch_i.assign(c.branch_counter, 0.0);
        for(auto* const b: c.size_t_to_branch_p)
        {
            if(b->index < branch_i.size()) { branch_i[b->index] = b->current.real(); }
        }
    }

    [[nodiscard]] double max_abs_diff(std::vector<double> const& a, std::vector<double> const& b) noexcept
    {
        auto const n = (a.size() < b.size()) ? a.size() : b.size();
        double m{};
        for(std::size_t i{}; i < n; ++i)
        {
            double const d = absd(a[i] - b[i]);
            if(d > m) { m = d; }
        }
        return m;
    }
}  // namespace

int main(int argc, char** argv)
{
    auto const cfg{parse_args(argc, argv)};
    if(cfg.nodes < 2)
    {
        ::fast_io::io::perr("--nodes must be >= 2\n");
        return 2;
    }
    if(cfg.iters == 0)
    {
        ::fast_io::io::perr("--iters must be >= 1\n");
        return 2;
    }

    int device_count{};
    auto const dc_st{cudaGetDeviceCount(&device_count)};
    if(dc_st != cudaSuccess || device_count <= 0)
    {
        ::fast_io::io::perr("No CUDA device visible (", ::fast_io::mnp::os_c_str(cudaGetErrorString(dc_st)), ").\n");
        return 77;
    }
    cudaDeviceProp prop{};
    if(cudaGetDeviceProperties(&prop, 0) == cudaSuccess)
    {
        ::fast_io::io::perr("GPU[0]=", prop.name, ", cc=", prop.major, ".", prop.minor, "\n");
    }

    ::fast_io::io::perr("CPU vs CUDA random links benchmark: nodes=", cfg.nodes, ", warmup=", cfg.warmup, ", iters=", cfg.iters, ", seed=", cfg.seed, "\n");
    ::fast_io::io::perr("links cases: ");
    for(std::size_t i{}; i < cfg.links.size(); ++i)
    {
        if(i) { ::fast_io::io::perr(","); }
        ::fast_io::io::perr(cfg.links[i]);
    }
    ::fast_io::io::perr("\n");

    for(auto const links: cfg.links)
    {
        ::phy_engine::circult c{};
        build_random_links_case(c, cfg.nodes, links, cfg.seed, cfg.r_chain, cfg.r_link, cfg.vdc);
        c.prepare();

        if(!c.cuda_solver.is_available())
        {
            ::fast_io::io::perr("CUDA solver unavailable (cudart/cusolver init failed).\n");
            return 77;
        }

        auto const nnz{nnz_from_mna(c)};

        // CPU
        c.set_cuda_policy(::phy_engine::circult::cuda_solve_policy::force_cpu);
        auto const cpu_stats{run_solve_loops(c, cfg.warmup, cfg.iters)};
        if(cpu_stats.avg_ms == 0.0)
        {
            ::fast_io::io::perr("CPU solve failed\n");
            return 3;
        }

        std::vector<double> cpu_node_v{};
        std::vector<double> cpu_branch_i{};
        snapshot_solution(c, cpu_node_v, cpu_branch_i);

        // CUDA
        c.set_cuda_policy(::phy_engine::circult::cuda_solve_policy::force_cuda);
        c.set_cuda_node_threshold(0);
        auto const cuda_stats{run_solve_loops(c, cfg.warmup, cfg.iters)};
        if(cuda_stats.avg_ms == 0.0)
        {
            ::fast_io::io::perr("CUDA solve failed\n");
            return 4;
        }

        std::vector<double> cuda_node_v{};
        std::vector<double> cuda_branch_i{};
        snapshot_solution(c, cuda_node_v, cuda_branch_i);

        double const dv = max_abs_diff(cpu_node_v, cuda_node_v);
        double const di = max_abs_diff(cpu_branch_i, cuda_branch_i);

        double const speedup = cpu_stats.avg_ms / cuda_stats.avg_ms;
        ::fast_io::io::perr("links=", links,
                            " node_counter=", c.node_counter,
                            " branch_counter=", c.branch_counter,
                            " nnz=", nnz,
                            " cpu_avg_ms=", cpu_stats.avg_ms,
                            " cuda_avg_ms=", cuda_stats.avg_ms,
                            " speedup=", speedup,
                            " max_abs_diff_v=", dv,
                            " max_abs_diff_i=", di,
                            "\n");

        if(!(dv < cfg.eps && di < cfg.eps))
        {
            ::fast_io::io::perr("ERROR: solution mismatch (eps=", cfg.eps, ")\n");
            return 5;
        }
    }

    return 0;
}
#endif

