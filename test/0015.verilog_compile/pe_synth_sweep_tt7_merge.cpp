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

    // 7 named inputs (not constants).
    ::phy_engine::model::node_t* in_nodes[7]{};
    for(std::size_t i{}; i < 7u; ++i)
    {
        auto [m, pos] =
            ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
        (void)pos;
        if(m == nullptr || m->ptr == nullptr) { return 1; }
        ::fast_io::u8string nm{};
        nm.push_back(u8'x');
        nm.push_back(static_cast<char8_t>(u8'0' + static_cast<char8_t>(i)));
        m->name = ::std::move(nm);
        auto& n = ::phy_engine::netlist::create_node(nl);
        in_nodes[i] = __builtin_addressof(n);
        if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *in_nodes[i])) { return 2; }
    }

    auto build_xor_chain = [&](::phy_engine::model::node_t* const* ins, std::size_t nins) noexcept -> ::phy_engine::model::node_t*
    {
        if(nins == 0u) { return nullptr; }
        ::phy_engine::model::node_t* cur = ins[0];
        for(std::size_t i = 1; i < nins; ++i)
        {
            auto [xm, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::XOR{});
            (void)pos;
            if(xm == nullptr || xm->ptr == nullptr) { return nullptr; }
            auto& out_ref = ::phy_engine::netlist::create_node(nl);
            auto* out = __builtin_addressof(out_ref);
            if(!::phy_engine::netlist::add_to_node(nl, *xm, 0, *cur) || !::phy_engine::netlist::add_to_node(nl, *xm, 1, *ins[i]) ||
               !::phy_engine::netlist::add_to_node(nl, *xm, 2, *out))
            {
                return nullptr;
            }
            cur = out;
        }
        return cur;
    };

    // Two identical 7-input XOR functions built as two separate cones.
    auto* y1 = build_xor_chain(in_nodes, 7u);
    auto* y2 = build_xor_chain(in_nodes, 7u);
    if(y1 == nullptr || y2 == nullptr) { return 3; }

    auto [o1, o1_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    auto [o2, o2_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
    (void)o1_pos;
    (void)o2_pos;
    if(o1 == nullptr || o2 == nullptr) { return 4; }
    if(!::phy_engine::netlist::add_to_node(nl, *o1, 0, *y1)) { return 5; }
    if(!::phy_engine::netlist::add_to_node(nl, *o2, 0, *y2)) { return 6; }

    auto const before_xor = count_models_by_name(nl, u8"XOR");
    if(before_xor != 12u) { return 10; }  // 2 cones * (7 inputs -> 6 XOR gates)

    ::phy_engine::verilog::digital::pe_synth_options opt{};
    opt.assume_binary_inputs = true;
    opt.cuda_enable = true;      // force TT path even without CUDA build (it will fall back to CPU TT)
    opt.sweep_max_vars = 8;      // >6 -> TT path
    opt.sweep_max_gates = 256;

    bool const changed = ::phy_engine::verilog::digital::details::optimize_bounded_sweep_in_pe_netlist(nl, {}, opt);
    if(!changed) { return 11; }

    auto const after_xor = count_models_by_name(nl, u8"XOR");
    if(after_xor != 11u) { return 12; }  // at least the two 7-var roots should merge -> delete one XOR

    return 0;
}
