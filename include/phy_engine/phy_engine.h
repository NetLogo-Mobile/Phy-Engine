#pragma once
#include <complex>
#include <fast_io/fast_io.h>
#include "model/model_refs/base.h"
#include "netlist/impl.h"
#include "circuits/circuit.h"

// models
#include "model/models/linear/capacitor.h"
#include "model/models/linear/CCCS.h"
#include "model/models/linear/CCVS.h"
#include "model/models/linear/IAC.h"
#include "model/models/linear/IDC.h"
#include "model/models/linear/inductor.h"
#include "model/models/linear/resistance.h"
#include "model/models/linear/VAC.h"
#include "model/models/linear/VCCS.h"
#include "model/models/linear/VCVS.h"
#include "model/models/linear/VDC.h"

// controller
#include "model/models/controller/switch.h"

// non-linear
#include "model/models/non-linear/PN_junction.h"

// Digital
#include "model/models/digital/logical/and.h"
#include "model/models/digital/logical/or.h"
#include "model/models/digital/logical/not.h"
#include "model/models/digital/logical/xor.h"
#include "model/models/digital/logical/xnor.h"
#include "model/models/digital/logical/nand.h"
#include "model/models/digital/logical/nor.h"
#include "model/models/digital/logical/implication.h"
#include "model/models/digital/logical/non_implication.h"
#include "model/models/digital/logical/input.h"
#include "model/models/digital/logical/output.h"

// Combinational (no delay)
#include "model/models/digital/combinational/half_adder.h"
#include "model/models/digital/combinational/full_adder.h"
#include "model/models/digital/combinational/half_subtractor.h"
#include "model/models/digital/combinational/full_subtractor.h"
#include "model/models/digital/combinational/mul2.h"
#include "model/models/digital/combinational/d_ff.h"
#include "model/models/digital/combinational/jk_ff.h"
#include "model/models/digital/combinational/t_ff.h"
#include "model/models/digital/combinational/t_bar_ff.h"
