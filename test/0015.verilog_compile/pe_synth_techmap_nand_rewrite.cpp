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

    // y = ~(a & b) built as AND + NOT (2 gates).
    ::phy_engine::model::node_t* and_out{};
    ::phy_engine::model::node_t* y{};
    {
        auto [m_and, pos_and] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::AND{});
        (void)pos_and;
        auto [m_not, pos_not] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
        (void)pos_not;
        if(m_and == nullptr || m_not == nullptr || m_and->ptr == nullptr || m_not->ptr == nullptr) { return 4; }
        and_out = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        y = __builtin_addressof(::phy_engine::netlist::create_node(nl));
        if(!::phy_engine::netlist::add_to_node(nl, *m_and, 0, *a) || !::phy_engine::netlist::add_to_node(nl, *m_and, 1, *b) ||
           !::phy_engine::netlist::add_to_node(nl, *m_and, 2, *and_out))
        {
            return 5;
        }
        if(!::phy_engine::netlist::add_to_node(nl, *m_not, 0, *and_out) || !::phy_engine::netlist::add_to_node(nl, *m_not, 1, *y)) { return 6; }
    }

    auto [m_out, pos_out] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)pos_out;
    if(m_out == nullptr || m_out->ptr == nullptr) { return 7; }
    if(!::phy_engine::netlist::add_to_node(nl, *m_out, 0, *y)) { return 8; }

    auto const before_and = count_models_by_name(nl, u8"AND");
    auto const before_not = count_models_by_name(nl, u8"NOT");
    auto const before_nand = count_models_by_name(nl, u8"NAND");
    if(before_and != 1u || before_not != 1u || before_nand != 0u) { return 10; }

    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = true;
    opt.techmap_enable = true;
    opt.techmap_max_cut = 2;
    opt.techmap_max_gates = 32;
    opt.techmap_max_cuts = 64;
    opt.techmap_richer_library = true;
    opt.cuda_enable = true;  // exercise CUDA batch path (best-effort; falls back to CPU when CUDA is not built)

    bool const changed = ::phy_engine::verilog::digital::details::optimize_cut_based_techmap_in_pe_netlist(nl, {}, opt);
    if(!changed) { return 11; }

    auto const after_and = count_models_by_name(nl, u8"AND");
    auto const after_not = count_models_by_name(nl, u8"NOT");
    auto const after_nand = count_models_by_name(nl, u8"NAND");

    // Expect a single NAND2 implementation, no need for AND+NOT.
    if(after_nand != 1u) { return 12; }
    if(after_and != 0u) { return 13; }
    if(after_not != 0u) { return 14; }

    return 0;
}
