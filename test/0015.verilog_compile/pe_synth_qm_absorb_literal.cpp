#include <cstddef>
#include <cstdio>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
std::size_t count_models_by_name(::phy_engine::netlist::netlist const& nl, ::fast_io::u8string_view name) noexcept
{
    std::size_t n{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal || m->ptr == nullptr) { continue; }
            if(m->ptr->get_model_name() == name) { ++n; }
        }
    }
    return n;
}
}  // namespace

int main()
{
    using namespace ::phy_engine;

    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& nl = c.get_netlist();

    // Named binary inputs.
    ::phy_engine::model::node_t* a{};
    ::phy_engine::model::node_t* b{};
    {
        auto [ma, posa] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)posa;
        auto [mb, posb] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)posb;
        if(ma == nullptr || mb == nullptr || ma->ptr == nullptr || mb->ptr == nullptr) { return 1; }
        {
            ::fast_io::u8string nm{};
            nm.push_back(u8'a');
            ma->name = ::std::move(nm);
        }
        {
            ::fast_io::u8string nm{};
            nm.push_back(u8'b');
            mb->name = ::std::move(nm);
        }
        a = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        b = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        if(!::phy_engine::netlist::add_to_node(nl, *ma, 0, *a)) { return 2; }
        if(!::phy_engine::netlist::add_to_node(nl, *mb, 0, *b)) { return 3; }
    }

    // Build y = (a & b) | (a & ~b) = a.
    ::phy_engine::model::node_t* not_b{};
    ::phy_engine::model::node_t* and0_out{};
    ::phy_engine::model::node_t* and1_out{};
    ::phy_engine::model::node_t* y{};
    {
        auto [m_not, pos_not] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
        (void)pos_not;
        auto [m_and0, pos_and0] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
        (void)pos_and0;
        auto [m_and1, pos_and1] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
        (void)pos_and1;
        auto [m_or, pos_or] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OR{});
        (void)pos_or;
        if(m_not == nullptr || m_and0 == nullptr || m_and1 == nullptr || m_or == nullptr) { return 4; }
        if(m_not->ptr == nullptr || m_and0->ptr == nullptr || m_and1->ptr == nullptr || m_or->ptr == nullptr) { return 5; }

        not_b = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        and0_out = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        and1_out = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        y = __builtin_addressof(::phy_engine::netlist::create_node(nl));

        if(!::phy_engine::netlist::add_to_node(nl, *m_not, 0, *b) || !::phy_engine::netlist::add_to_node(nl, *m_not, 1, *not_b)) { return 6; }

        if(!::phy_engine::netlist::add_to_node(nl, *m_and0, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m_and0, 1, *b) ||
           !::phy_engine::netlist::add_to_node(nl, *m_and0, 2, *and0_out))
        {
            return 7;
        }
        if(!::phy_engine::netlist::add_to_node(nl, *m_and1, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m_and1, 1, *not_b) ||
           !::phy_engine::netlist::add_to_node(nl, *m_and1, 2, *and1_out))
        {
            return 8;
        }
        if(!::phy_engine::netlist::add_to_node(nl, *m_or, 0, *and0_out) || !::phy_engine::netlist::add_to_node(nl, *m_or, 1, *and1_out) ||
           !::phy_engine::netlist::add_to_node(nl, *m_or, 2, *y))
        {
            return 9;
        }
    }

    auto [m_out, pos_out] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)pos_out;
    if(m_out == nullptr || m_out->ptr == nullptr) { return 10; }
    if(!::phy_engine::netlist::add_to_node(nl, *m_out, 0, *y)) { return 11; }

    auto const before_and = count_models_by_name(nl, u8"AND");
    auto const before_or = count_models_by_name(nl, u8"OR");
    auto const before_not = count_models_by_name(nl, u8"NOT");
    if(before_and != 2u || before_or != 1u || before_not != 1u) { return 12; }

    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = true;
    opt.qm_max_vars = 8;
    opt.qm_max_gates = 64;
    opt.qm_max_primes = 4096;
    opt.infer_dc_from_xz = true;
    opt.cuda_enable = true;  // exercise CUDA TT path (best-effort; falls back to CPU TT when CUDA is not built)

    bool const changed = ::phy_engine::verilog::digital::details::optimize_qm_two_level_minimize_in_pe_netlist(nl, {}, opt, nullptr);
    if(!changed) { return 13; }

    auto const after_and = count_models_by_name(nl, u8"AND");
    auto const after_or = count_models_by_name(nl, u8"OR");
    auto const after_not = count_models_by_name(nl, u8"NOT");

    // The optimized implementation should absorb b and simplify to y = a (no logic gates needed).
    if(after_and != 0u) { return 14; }
    if(after_or != 0u) { return 15; }
    if(after_not != 0u) { return 16; }

    return 0;
}
