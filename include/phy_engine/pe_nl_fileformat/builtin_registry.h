#pragma once

// Registers all models under include/phy_engine/model/models.
// (Used for loading/saving complete circuits without requiring per-project registration.)

#include <phy_engine/model/models/controller/comparator.h>
#include <phy_engine/model/models/controller/relay.h>
#include <phy_engine/model/models/controller/switch.h>

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

#include <phy_engine/model/models/digital/verilog_module.h>
#include <phy_engine/model/models/digital/verilog_ports.h>

#include <phy_engine/model/models/generator/pulse.h>
#include <phy_engine/model/models/generator/sawtooth.h>
#include <phy_engine/model/models/generator/square.h>
#include <phy_engine/model/models/generator/triangle.h>

#include <phy_engine/model/models/linear/CCCS.h>
#include <phy_engine/model/models/linear/CCVS.h>
#include <phy_engine/model/models/linear/IAC.h>
#include <phy_engine/model/models/linear/IDC.h>
#include <phy_engine/model/models/linear/VAC.h>
#include <phy_engine/model/models/linear/VCCS.h>
#include <phy_engine/model/models/linear/VCVS.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/capacitor.h>
#include <phy_engine/model/models/linear/coupled_inductors.h>
#include <phy_engine/model/models/linear/inductor.h>
#include <phy_engine/model/models/linear/op_amp.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/model/models/linear/transformer.h>
#include <phy_engine/model/models/linear/transformer_center_tap.h>

#include <phy_engine/model/models/non-linear/BJT_NPN.h>
#include <phy_engine/model/models/non-linear/BJT_PNP.h>
#include <phy_engine/model/models/non-linear/PN_junction.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/model/models/non-linear/full_bridge_rectifier.h>
#include <phy_engine/model/models/non-linear/nmosfet.h>
#include <phy_engine/model/models/non-linear/pmosfet.h>

#include "model_registry.h"

namespace phy_engine::pe_nl_fileformat
{
    inline model_registry const& default_registry()
    {
        static model_registry reg = []()
        {
            model_registry r{};

            // controller
            r.add(details::make_entry<::phy_engine::model::comparator>());
            r.add(details::make_entry<::phy_engine::model::relay>());
            r.add(details::make_entry<::phy_engine::model::single_pole_switch>());

            // digital (combinational)
            r.add(details::make_entry<::phy_engine::model::COUNTER4>());
            r.add(details::make_entry<::phy_engine::model::DFF>());
            r.add(details::make_entry<::phy_engine::model::DFF_ARSTN>());
            r.add(details::make_entry<::phy_engine::model::DLATCH>());
            r.add(details::make_entry<::phy_engine::model::FULL_ADDER>());
            r.add(details::make_entry<::phy_engine::model::FULL_SUB>());
            r.add(details::make_entry<::phy_engine::model::HALF_ADDER>());
            r.add(details::make_entry<::phy_engine::model::HALF_SUB>());
            r.add(details::make_entry<::phy_engine::model::JKFF>());
            r.add(details::make_entry<::phy_engine::model::MUL2>());
            r.add(details::make_entry<::phy_engine::model::RANDOM_GENERATOR4>());
            r.add(details::make_entry<::phy_engine::model::T_BAR_FF>());
            r.add(details::make_entry<::phy_engine::model::TFF>());

            // digital (logical)
            r.add(details::make_entry<::phy_engine::model::AND>());
            r.add(details::make_entry<::phy_engine::model::CASE_EQ>());
            r.add(details::make_entry<::phy_engine::model::EIGHT_BIT_DISPLAY>());
            r.add(details::make_entry<::phy_engine::model::EIGHT_BIT_INPUT>());
            r.add(details::make_entry<::phy_engine::model::IMP>());
            r.add(details::make_entry<::phy_engine::model::INPUT>());
            r.add(details::make_entry<::phy_engine::model::IS_UNKNOWN>());
            r.add(details::make_entry<::phy_engine::model::NAND>());
            r.add(details::make_entry<::phy_engine::model::NIMP>());
            r.add(details::make_entry<::phy_engine::model::NOR>());
            r.add(details::make_entry<::phy_engine::model::NOT>());
            r.add(details::make_entry<::phy_engine::model::OR>());
            r.add(details::make_entry<::phy_engine::model::OUTPUT>());
            r.add(details::make_entry<::phy_engine::model::RESOLVE2>());
            r.add(details::make_entry<::phy_engine::model::SCHMITT_TRIGGER>());
            r.add(details::make_entry<::phy_engine::model::TICK_DELAY>());
            r.add(details::make_entry<::phy_engine::model::TRI>());
            r.add(details::make_entry<::phy_engine::model::XNOR>());
            r.add(details::make_entry<::phy_engine::model::XOR>());
            r.add(details::make_entry<::phy_engine::model::YES>());

            // digital (verilog)
            r.add(details::make_entry<::phy_engine::model::VERILOG_MODULE>());
            r.add(details::make_entry<::phy_engine::model::VERILOG_PORTS>());

            // generators
            r.add(details::make_entry<::phy_engine::model::pulse_gen>());
            r.add(details::make_entry<::phy_engine::model::sawtooth_gen>());
            r.add(details::make_entry<::phy_engine::model::square_gen>());
            r.add(details::make_entry<::phy_engine::model::triangle_gen>());

            // linear
            r.add(details::make_entry<::phy_engine::model::CCCS>());
            r.add(details::make_entry<::phy_engine::model::CCVS>());
            r.add(details::make_entry<::phy_engine::model::IAC>());
            r.add(details::make_entry<::phy_engine::model::IDC>());
            r.add(details::make_entry<::phy_engine::model::VAC>());
            r.add(details::make_entry<::phy_engine::model::VCCS>());
            r.add(details::make_entry<::phy_engine::model::VCVS>());
            r.add(details::make_entry<::phy_engine::model::VDC>());
            r.add(details::make_entry<::phy_engine::model::capacitor>());
            r.add(details::make_entry<::phy_engine::model::coupled_inductors>());
            r.add(details::make_entry<::phy_engine::model::inductor>());
            r.add(details::make_entry<::phy_engine::model::op_amp>());
            r.add(details::make_entry<::phy_engine::model::resistance>());
            r.add(details::make_entry<::phy_engine::model::transformer>());
            r.add(details::make_entry<::phy_engine::model::transformer_center_tap>());

            // non-linear
            r.add(details::make_entry<::phy_engine::model::BJT_NPN>());
            r.add(details::make_entry<::phy_engine::model::BJT_PNP>());
            r.add(details::make_entry<::phy_engine::model::PN_junction>());
            r.add(details::make_entry<::phy_engine::model::bsim3v32_nmos>());
            r.add(details::make_entry<::phy_engine::model::bsim3v32_pmos>());
            r.add(details::make_entry<::phy_engine::model::full_bridge_rectifier>());
            r.add(details::make_entry<::phy_engine::model::nmosfet>());
            r.add(details::make_entry<::phy_engine::model::pmosfet>());

            return r;
        }();
        return reg;
    }
}  // namespace phy_engine::pe_nl_fileformat
