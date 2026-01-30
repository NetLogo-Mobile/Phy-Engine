#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
using ::phy_engine::verilog::digital::details::espresso_two_level_minimize;
using ::phy_engine::verilog::digital::details::implicant_covers;

bool cover_covers_minterm(::std::vector<::phy_engine::verilog::digital::details::qm_implicant> const& cover, std::uint16_t m) noexcept
{
    for(auto const& c : cover)
    {
        if(implicant_covers(c, m)) { return true; }
    }
    return false;
}
}  // namespace

int main()
{
    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = true;

    // Case 1: f = ~x2 on 4 vars. ON-set = all minterms with bit2=0.
    {
        std::vector<std::uint16_t> on{0, 1, 2, 3, 8, 9, 10, 11};
        std::vector<std::uint16_t> dc{};

        auto sol = espresso_two_level_minimize(on, dc, 4, opt);
        if(sol.cost != 1u || sol.cover.size() != 1u)
        {
            std::fprintf(stderr, "espresso unexpected result for ~x2: cost=%zu cubes=%zu\n", sol.cost, sol.cover.size());
            return 2;
        }

        for(std::uint16_t m{}; m < 16u; ++m)
        {
            bool const exp = ((m & 0x4u) == 0u);
            bool const got = cover_covers_minterm(sol.cover, m);
            if(got != exp)
            {
                std::fprintf(stderr, "espresso mismatch for ~x2 at m=%u (exp=%u got=%u)\n", m, static_cast<unsigned>(exp), static_cast<unsigned>(got));
                return 3;
            }
        }
    }

    // Case 2 (DC exploitation): var_count=2, ON={0}, DC={1,2,3}. OFF is empty, so f can be treated as constant 1.
    {
        std::vector<std::uint16_t> on{0};
        std::vector<std::uint16_t> dc{1, 2, 3};

        auto sol = espresso_two_level_minimize(on, dc, 2, opt);
        if(sol.cost != 0u || sol.cover.empty())
        {
            std::fprintf(stderr, "espresso unexpected result for DC-const1: cost=%zu cubes=%zu\n", sol.cost, sol.cover.size());
            return 4;
        }

        for(std::uint16_t m{}; m < 4u; ++m)
        {
            if(!cover_covers_minterm(sol.cover, m))
            {
                std::fprintf(stderr, "espresso expected const1 cover to hit m=%u\n", m);
                return 5;
            }
        }
    }

    return 0;
}
