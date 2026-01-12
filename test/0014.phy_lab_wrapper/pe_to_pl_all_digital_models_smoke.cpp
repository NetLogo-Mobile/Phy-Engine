#include <phy_engine/model/models/digital/combinational/counter4.h>
#include <phy_engine/model/models/digital/combinational/d_ff.h>
#include <phy_engine/model/models/digital/combinational/d_ff_arstn.h>
#include <phy_engine/model/models/digital/combinational/d_latch.h>
#include <phy_engine/model/models/digital/combinational/full_adder.h>
#include <phy_engine/model/models/digital/combinational/full_subtractor.h>
#include <phy_engine/model/models/digital/combinational/half_adder.h>
#include <phy_engine/model/models/digital/combinational/half_subtractor.h>
#include <phy_engine/model/models/digital/combinational/jk_ff.h>
#include <phy_engine/model/models/digital/combinational/mul2.h>
#include <phy_engine/model/models/digital/combinational/random_generator4.h>
#include <phy_engine/model/models/digital/combinational/t_bar_ff.h>
#include <phy_engine/model/models/digital/combinational/t_ff.h>
#include <phy_engine/model/models/digital/logical/and.h>
#include <phy_engine/model/models/digital/logical/case_eq.h>
#include <phy_engine/model/models/digital/logical/eight_bit_display.h>
#include <phy_engine/model/models/digital/logical/eight_bit_input.h>
#include <phy_engine/model/models/digital/logical/implication.h>
#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/is_unknown.h>
#include <phy_engine/model/models/digital/logical/nand.h>
#include <phy_engine/model/models/digital/logical/non_implication.h>
#include <phy_engine/model/models/digital/logical/nor.h>
#include <phy_engine/model/models/digital/logical/not.h>
#include <phy_engine/model/models/digital/logical/or.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/model/models/digital/logical/resolve2.h>
#include <phy_engine/model/models/digital/logical/schmitt_trigger.h>
#include <phy_engine/model/models/digital/logical/tick_delay.h>
#include <phy_engine/model/models/digital/logical/tri_state.h>
#include <phy_engine/model/models/digital/logical/xnor.h>
#include <phy_engine/model/models/digital/logical/xor.h>
#include <phy_engine/model/models/digital/logical/yes.h>
#include <phy_engine/netlist/operation.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <cassert>

int main()
{
    using namespace phy_engine;

    netlist::netlist nl{};

    auto connect_all_pins = [&](model::model_base* m) {
        assert(m != nullptr);
        assert(m->ptr != nullptr);
        auto pv = m->ptr->generate_pin_view();
        for(std::size_t i{}; i < pv.size; ++i)
        {
            auto& n = netlist::create_node(nl);
            bool ok = netlist::add_to_node(nl, *m, i, n);
            assert(ok);
        }
    };

    // Add one instance of every PE digital model (excluding VERILOG_*).
    {
        auto [m, pos] = netlist::add_model(nl, model::INPUT{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::OUTPUT{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::YES{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::NOT{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::AND{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::OR{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::XOR{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::XNOR{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::NAND{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::NOR{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::IMP{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::NIMP{});
        (void)pos;
        connect_all_pins(m);
    }

    // Verilog/digital helper primitives used by the synthesizer.
    {
        auto [m, pos] = netlist::add_model(nl, model::CASE_EQ{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::IS_UNKNOWN{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::TICK_DELAY{});
        (void)pos;
        connect_all_pins(m);
    }

    // Multi-driver / inout helpers.
    {
        auto [m, pos] = netlist::add_model(nl, model::TRI{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::RESOLVE2{});
        (void)pos;
        connect_all_pins(m);
    }

    // Utility/bus blocks.
    {
        auto [m, pos] = netlist::add_model(nl, model::EIGHT_BIT_INPUT{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::EIGHT_BIT_DISPLAY{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::SCHMITT_TRIGGER{});
        (void)pos;
        connect_all_pins(m);
    }

    // Arithmetic and sequential blocks.
    {
        auto [m, pos] = netlist::add_model(nl, model::HALF_ADDER{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::FULL_ADDER{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::HALF_SUB{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::FULL_SUB{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::MUL2{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::DFF{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::DFF_ARSTN{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::DLATCH{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::TFF{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::T_BAR_FF{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::JKFF{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::COUNTER4{});
        (void)pos;
        connect_all_pins(m);
    }
    {
        auto [m, pos] = netlist::add_model(nl, model::RANDOM_GENERATOR4{});
        (void)pos;
        connect_all_pins(m);
    }

    auto r = phy_lab_wrapper::pe_to_pl::convert(nl);

    assert(r.ex.type() == phy_lab_wrapper::experiment_type::circuit);
    assert(r.ex.wires().empty());

    // This is the number of non-VERILOG_* models added above.
    assert(r.ex.elements().size() == 33);

    for(auto const& e : r.ex.elements())
    {
        auto const p = e.element_position();
        assert(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
    }

    return 0;
}

