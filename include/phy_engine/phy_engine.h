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
#include "model/models/digital/and.h"
#include "model/models/digital/or.h"
#include "model/models/digital/not.h"
#include "model/models/digital/xor.h"
#include "model/models/digital/xnor.h"
#include "model/models/digital/nand.h"
#include "model/models/digital/nor.h"
#include "model/models/digital/implication.h"
#include "model/models/digital/non_implication.h"
#include "model/models/digital/input.h"
#include "model/models/digital/output.h"
