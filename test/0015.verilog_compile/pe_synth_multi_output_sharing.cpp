#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
using namespace phy_engine::verilog::digital;
using namespace phy_engine::verilog::digital::details;

bool cover_eval(std::vector<qm_implicant> const& cover, std::uint16_t m) noexcept
{
    for(auto const& c : cover)
    {
        if(implicant_covers(c, m)) { return true; }
    }
    return false;
}

std::vector<qm_implicant> qm_exact_cover(std::vector<std::uint16_t> const& on, std::vector<std::uint16_t> const& dc, std::size_t var_count, pe_synth_options const& opt)
{
    if(on.empty()) { return {}; }
    if(on.size() == (1u << var_count))
    {
        return {qm_implicant{0u, static_cast<std::uint16_t>((var_count >= 16) ? 0xFFFFu : ((1u << var_count) - 1u)), false}};
    }

    auto const primes = qm_prime_implicants(on, dc, var_count);
    auto const sol = qm_exact_minimum_cover(primes, on, var_count, opt);
    if(sol.cost == static_cast<std::size_t>(-1)) { return {}; }

    std::vector<qm_implicant> cover{};
    cover.reserve(sol.pick.size());
    for(auto const idx : sol.pick)
    {
        if(idx < primes.size()) { cover.push_back(primes[idx]); }
    }
    return cover;
}
}  // namespace

int main()
{
    pe_synth_options opt{};
    opt.assume_binary_inputs = true;
    opt.two_level_cost = pe_synth_options::two_level_cost_model::gate_count;
    opt.two_level_weights = {};

    constexpr std::size_t var_count = 4;

    // Found by random search with per-output DC-sets: joint cover selection reduces total cost vs independent minimization.
    // Care set is OFF = !(ON ∪ DC).
    std::vector<std::uint16_t> on1{6, 8, 9, 11};
    std::vector<std::uint16_t> dc1{12, 13, 14, 15};
    std::vector<std::uint16_t> on2{1, 7, 11, 13};
    std::vector<std::uint16_t> dc2{5, 8, 9};

    auto c1 = qm_exact_cover(on1, dc1, var_count, opt);
    auto c2 = qm_exact_cover(on2, dc2, var_count, opt);
    if(c1.empty() || c2.empty())
    {
        std::fprintf(stderr, "expected non-empty independent covers\n");
        return 2;
    }

    std::vector<std::vector<qm_implicant>> base{c1, c2};
    auto const base_cost = multi_output_gate_cost(base, var_count, opt);

    std::vector<std::vector<std::uint16_t>> on_list{on1, on2};
    std::vector<std::vector<std::uint16_t>> dc_list{dc1, dc2};
    auto mo = multi_output_two_level_minimize(on_list, dc_list, var_count, opt);
    if(mo.cost == static_cast<std::size_t>(-1) || mo.covers.size() != 2u)
    {
        std::fprintf(stderr, "expected valid multi-output solution\n");
        return 3;
    }

    if(!(mo.cost < base_cost))
    {
        std::fprintf(stderr, "expected multi-output sharing to reduce cost (base=%zu mo=%zu)\n", base_cost, mo.cost);
        return 4;
    }

    // Exhaustive correctness check on the care set (OFF).
    for(std::uint16_t m{}; m < 16u; ++m)
    {
        auto in_vec = [&](std::vector<std::uint16_t> const& v) noexcept -> bool
        {
            for(auto const x : v)
            {
                if(x == m) { return true; }
            }
            return false;
        };

        bool const is_dc1 = in_vec(dc1);
        bool const is_dc2 = in_vec(dc2);
        bool const exp1 = in_vec(on1);
        bool const exp2 = in_vec(on2);

        bool const got1 = cover_eval(mo.covers[0], m);
        bool const got2 = cover_eval(mo.covers[1], m);

        if(!is_dc1 && got1 != exp1)
        {
            std::fprintf(stderr, "y1 mismatch at m=%u (got=%u exp=%u)\n", m, static_cast<unsigned>(got1), static_cast<unsigned>(exp1));
            return 5;
        }
        if(!is_dc2 && got2 != exp2)
        {
            std::fprintf(stderr,
                         "y2 mismatch at m=%u (got=%u exp=%u)\n",
                         m,
                         static_cast<unsigned>(got2),
                         static_cast<unsigned>(exp2));
            return 5;
        }
    }

    return 0;
}
