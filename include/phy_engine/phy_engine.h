﻿#pragma once
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

#include "model/models/controller/switch.h"

