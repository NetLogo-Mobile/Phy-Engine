#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
std::size_t count_logic_gates(::phy_engine::netlist::netlist const& nl) noexcept
{
    std::size_t gates{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            auto const n = m->ptr->get_model_name();
            if(n == u8"AND" || n == u8"OR" || n == u8"XOR" || n == u8"XNOR" || n == u8"NOT" || n == u8"NAND" || n == u8"NOR" || n == u8"IMP" ||
               n == u8"NIMP" || n == u8"YES")
            {
                ++gates;
            }
        }
    }
    return gates;
}
}  // namespace

int main()
{
    using namespace phy_engine;

    // Regression: the AND/OR flatten pass used a recursive DFS that could stack-overflow on very deep chains.
    // Build a deep (10k) AND chain that should simplify from (a & b & a & a & ...) to (a & b).
    constexpr std::size_t G = 10'000;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& nl = c.get_netlist();

    auto& a_ref = ::phy_engine::netlist::create_node(nl);
    auto& b_ref = ::phy_engine::netlist::create_node(nl);
    auto* a = __builtin_addressof(a_ref);
    auto* b = __builtin_addressof(b_ref);

    std::vector<::phy_engine::model::node_t*> w{};
    w.reserve(G);
    for(std::size_t i{}; i < G; ++i)
    {
        auto& nref = ::phy_engine::netlist::create_node(nl);
        w.push_back(__builtin_addressof(nref));
    }

    // Add an OUTPUT model to ensure the final node has a consumer.
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)pos;
        if(m == nullptr || !::phy_engine::netlist::add_to_node(nl, *m, 0, *w.back())) { return 1; }
    }

    // Create the chain "backwards" so the deepest root appears early in the root scan.
    for(std::size_t i = G; i-- > 0;)
    {
        auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
        (void)pos;
        if(m == nullptr) { return 2; }

        auto* in0 = (i == 0) ? a : w[i - 1];
        auto* in1 = (i == 0) ? b : a;
        auto* out = w[i];
        if(in0 == nullptr || in1 == nullptr || out == nullptr) { return 3; }
        if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in0) || !::phy_engine::netlist::add_to_node(nl, *m, 1, *in1) ||
           !::phy_engine::netlist::add_to_node(nl, *m, 2, *out))
        {
            return 4;
        }
    }

    auto const before = count_logic_gates(nl);
    if(before < G) { return 5; }

    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = false;  // keep the transformation in the 4-valued safe mode

    std::vector<::phy_engine::model::node_t*> protected_nodes{};
    bool const changed =
        ::phy_engine::verilog::digital::details::optimize_flatten_associative_and_or_in_pe_netlist(nl, protected_nodes, opt);

    auto const after = count_logic_gates(nl);
    if(!changed || after >= before)
    {
        std::fprintf(stderr, "expected flatten to reduce gates (before=%zu after=%zu changed=%d)\n", before, after, static_cast<int>(changed));
        return 6;
    }

    return 0;
}
