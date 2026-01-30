#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
using namespace phy_engine::verilog::digital;
using namespace phy_engine::verilog::digital::details;
}  // namespace

int main()
{
    // Case 1: 3-literal single cube (all positive) => gate_count=2 (AND tree), literal_count=3.
    {
        std::vector<qm_implicant> cover{};
        cover.push_back(qm_implicant{static_cast<std::uint16_t>(0b111u), static_cast<std::uint16_t>(0u), false});

        pe_synth_options opt_gate{};
        opt_gate.two_level_cost = pe_synth_options::two_level_cost_model::gate_count;
        auto const gate_cost = two_level_cover_cost(cover, 3, opt_gate);
        if(gate_cost != 2u)
        {
            std::fprintf(stderr, "gate_count expected 2, got %zu\n", gate_cost);
            return 2;
        }

        pe_synth_options opt_lit{};
        opt_lit.two_level_cost = pe_synth_options::two_level_cost_model::literal_count;
        auto const lit_cost = two_level_cover_cost(cover, 3, opt_lit);
        if(lit_cost != 3u)
        {
            std::fprintf(stderr, "literal_count expected 3, got %zu\n", lit_cost);
            return 3;
        }
    }

    // Case 2: one negated literal; test weighting.
    {
        std::vector<qm_implicant> cover{};
        // ~v0 & v1 & v2 (mask=0, value=0b110)
        cover.push_back(qm_implicant{static_cast<std::uint16_t>(0b110u), static_cast<std::uint16_t>(0u), false});

        pe_synth_options opt{};
        opt.two_level_cost = pe_synth_options::two_level_cost_model::gate_count;
        opt.two_level_weights.not_w = 5;
        opt.two_level_weights.and_w = 1;
        opt.two_level_weights.or_w = 1;

        auto const cost = two_level_cover_cost(cover, 3, opt);
        // not_w=5 (one neg), and_cost=2, or_cost=0 => 7
        if(cost != 7u)
        {
            std::fprintf(stderr, "weighted gate_count expected 7, got %zu\n", cost);
            return 4;
        }
    }

    return 0;
}

