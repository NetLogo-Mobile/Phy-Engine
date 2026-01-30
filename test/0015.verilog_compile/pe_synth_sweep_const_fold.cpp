#include <cstddef>
#include <cstdio>

#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
inline bool is_unnamed_input_const(::phy_engine::model::model_base const& mb, ::phy_engine::model::digital_node_statement_t v) noexcept
{
    if(mb.type != ::phy_engine::model::model_type::normal || mb.ptr == nullptr) { return false; }
    if(mb.name.size() != 0) { return false; }
    if(mb.ptr->get_model_name() != u8"INPUT") { return false; }
    auto vi = mb.ptr->get_attribute(0);
    return (vi.type == ::phy_engine::model::variant_type::digital) && (vi.digital == v);
}

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

    auto [out_m, out_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)out_pos;
    if(out_m == nullptr || out_m->ptr == nullptr) { return 1; }

    auto [not_m, not_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::NOT{});
    (void)not_pos;
    if(not_m == nullptr || not_m->ptr == nullptr) { return 2; }

    // Unnamed INPUT constants are represented as INPUT with empty model name.
    auto [c0_m, c0_pos] =
        ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
    (void)c0_pos;
    if(c0_m == nullptr || c0_m->ptr == nullptr) { return 3; }

    auto& n_c0 = ::phy_engine::netlist::create_node(nl);
    auto& n_y = ::phy_engine::netlist::create_node(nl);

    // NOT(c0) -> y
    if(!::phy_engine::netlist::add_to_node(nl, *c0_m, 0, n_c0)) { return 4; }
    if(!::phy_engine::netlist::add_to_node(nl, *not_m, 0, n_c0)) { return 5; }
    if(!::phy_engine::netlist::add_to_node(nl, *not_m, 1, n_y)) { return 6; }
    if(!::phy_engine::netlist::add_to_node(nl, *out_m, 0, n_y)) { return 7; }

    if(count_models_by_name(nl, u8"NOT") != 1u) { return 10; }

    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = true;
    opt.sweep_max_vars = 6;
    opt.sweep_max_gates = 64;

    if(!::phy_engine::verilog::digital::details::optimize_bounded_sweep_in_pe_netlist(nl, {}, opt)) { return 11; }

    // The NOT should be folded away, and OUTPUT should read CONST1 (unnamed INPUT=1).
    if(count_models_by_name(nl, u8"NOT") != 0u) { return 12; }

    ::phy_engine::model::node_t* const1_node{};
    for(auto const& blk : nl.models)
    {
        for(auto const* m = blk.begin; m != blk.curr; ++m)
        {
            if(!is_unnamed_input_const(*m, ::phy_engine::model::digital_node_statement_t::true_state)) { continue; }
            auto pv = m->ptr->generate_pin_view();
            if(pv.size == 1 && pv.pins[0].nodes != nullptr) { const1_node = pv.pins[0].nodes; }
        }
    }
    if(const1_node == nullptr) { return 13; }

    ::phy_engine::model::node_t* out_node{};
    {
        auto pv = out_m->ptr->generate_pin_view();
        if(pv.size != 1 || pv.pins[0].nodes == nullptr) { return 14; }
        out_node = pv.pins[0].nodes;
    }
    if(out_node != const1_node) { return 15; }

    return 0;
}

