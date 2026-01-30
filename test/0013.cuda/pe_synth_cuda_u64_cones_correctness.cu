#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <random>
#include <vector>

#define PHY_ENGINE_ENABLE_CUDA_PE_SYNTH 1
#include <phy_engine/verilog/digital/pe_synth.h>

int main()
{
    using namespace ::phy_engine::verilog::digital::details;

    int const devs = phy_engine_pe_synth_cuda_get_device_count();
    if(devs <= 0) { return 77; }  // skipped (no CUDA device/runtime)

    std::mt19937_64 rng{12345u};

    constexpr std::size_t N = 4096;
    std::vector<cuda_u64_cone_desc> cones{};
    cones.resize(N);
    std::vector<std::uint64_t> cpu{};
    cpu.resize(N);
    std::vector<std::uint64_t> gpu{};
    gpu.resize(N);

    std::uniform_int_distribution<unsigned> var_dist(0u, 6u);
    std::uniform_int_distribution<unsigned> gate_dist(1u, 64u);
    std::uniform_int_distribution<unsigned> kind_dist(0u, 8u);
    std::uniform_int_distribution<unsigned> const_dist(0u, 7u);  // bias to leaves/gates; occasionally consts

    for(std::size_t i{}; i < N; ++i)
    {
        cuda_u64_cone_desc c{};
        c.var_count = static_cast<std::uint8_t>(var_dist(rng));
        c.gate_count = static_cast<std::uint8_t>(gate_dist(rng));

        auto const vc = static_cast<unsigned>(c.var_count);
        auto const gc = static_cast<unsigned>(c.gate_count);

        for(unsigned gi = 0u; gi < gc; ++gi)
        {
            c.kind[gi] = static_cast<std::uint8_t>(kind_dist(rng));

            auto pick_idx = [&](bool allow_gate) -> std::uint8_t
            {
                unsigned const pick = const_dist(rng);
                if(pick == 0u) { return 254u; }
                if(pick == 1u) { return 255u; }

                if(vc != 0u)
                {
                    std::uniform_int_distribution<unsigned> leaf_pick(0u, vc - 1u);
                    if(!allow_gate || pick < 4u) { return static_cast<std::uint8_t>(leaf_pick(rng)); }
                }

                if(allow_gate && gi != 0u)
                {
                    std::uniform_int_distribution<unsigned> gate_pick(0u, gi - 1u);
                    return static_cast<std::uint8_t>(6u + gate_pick(rng));
                }

                return 254u;
            };

            c.in0[gi] = pick_idx(true);
            c.in1[gi] = pick_idx(c.kind[gi] != 0u);  // NOT ignores in1
            if(c.kind[gi] == 0u) { c.in1[gi] = 254u; }
        }

        cones[i] = c;
        cpu[i] = eval_u64_cone_cpu(cones[i]);
    }

    if(!phy_engine_pe_synth_cuda_eval_u64_cones(0u, cones.data(), cones.size(), gpu.data()))
    {
        std::fprintf(stderr, "cuda_eval_u64_cones failed\n");
        return 2;
    }

    for(std::size_t i{}; i < N; ++i)
    {
        if(cpu[i] != gpu[i])
        {
            std::fprintf(stderr, "mismatch at %zu: cpu=%016llx gpu=%016llx\n", i, (unsigned long long)cpu[i], (unsigned long long)gpu[i]);
            return 1;
        }
    }

    return 0;
}

