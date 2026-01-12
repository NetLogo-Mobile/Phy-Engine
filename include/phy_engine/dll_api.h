#pragma once

// C ABI helpers for embedding Phy-Engine as a shared library.
// This header documents the exported functions implemented in `src/dll_main.cpp`.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Matches `phy_engine::analyze_type` (see `include/phy_engine/circuits/analyze.h`).
enum phy_engine_analyze_type : uint32_t
{
    PHY_ENGINE_ANALYZE_OP = 0,
    PHY_ENGINE_ANALYZE_DC = 1,
    PHY_ENGINE_ANALYZE_AC = 2,
    PHY_ENGINE_ANALYZE_ACOP = 3,
    PHY_ENGINE_ANALYZE_TR = 4,
    PHY_ENGINE_ANALYZE_TROP = 5,
};

// Digital 4-state values (matches `phy_engine::model::digital_node_statement_t`).
enum phy_engine_digital_state : uint8_t
{
    PHY_ENGINE_D_L = 0,
    PHY_ENGINE_D_H = 1,
    PHY_ENGINE_D_X = 2,
    PHY_ENGINE_D_Z = 3,
};

// Element codes supported by `create_circuit()` / `create_circuit_ex()`.
// Property consumption is positional via the `properties` array.
enum phy_engine_element_code : int
{
    // Linear sources/passives
    PHY_ENGINE_E_RESISTOR = 1,   // properties: r
    PHY_ENGINE_E_CAPACITOR = 2,  // properties: C (stored as `m_kZimag`)
    PHY_ENGINE_E_INDUCTOR = 3,   // properties: L (stored as `m_kZimag`)
    PHY_ENGINE_E_VDC = 4,        // properties: V
    PHY_ENGINE_E_VAC = 5,        // properties: Vp, freq(Hz), phase(deg)
    PHY_ENGINE_E_IDC = 6,        // properties: I
    PHY_ENGINE_E_IAC = 7,        // properties: Ip, freq(Hz), phase(deg)

    // Dependent sources
    PHY_ENGINE_E_VCCS = 8,  // properties: G
    PHY_ENGINE_E_VCVS = 9,  // properties: Mu
    PHY_ENGINE_E_CCCS = 10, // properties: alpha
    PHY_ENGINE_E_CCVS = 11, // properties: r

    // Controllers / switches
    PHY_ENGINE_E_SWITCH_SPST = 12, // properties: cut_through (0/1)

    // Non-linear
    PHY_ENGINE_E_PN_JUNCTION = 13, // properties: Is,N,Isr,Nr,Temp,Ibv,Bv,Bv_set(0/1),Area

    // Coupled devices
    PHY_ENGINE_E_TRANSFORMER = 14,       // properties: n
    PHY_ENGINE_E_COUPLED_INDUCTORS = 15, // properties: L1,L2,k

    // Additional linear/controller/generator
    PHY_ENGINE_E_TRANSFORMER_CENTER_TAP = 16, // properties: n_total
    PHY_ENGINE_E_OP_AMP = 17,                 // properties: mu
    PHY_ENGINE_E_RELAY = 18,                  // properties: Von,Voff
    PHY_ENGINE_E_COMPARATOR = 19,             // properties: Ll,Hl

    PHY_ENGINE_E_SAWTOOTH = 20, // properties: Vh,Vl,freq(Hz),phase(rad)
    PHY_ENGINE_E_SQUARE = 21,   // properties: Vh,Vl,freq(Hz),duty,phase(rad)
    PHY_ENGINE_E_PULSE = 22,    // properties: Vh,Vl,freq(Hz),duty,phase(rad),tr,tf
    PHY_ENGINE_E_TRIANGLE = 23, // properties: Vh,Vl,freq(Hz),phase(rad)

    // Additional non-linear devices
    PHY_ENGINE_E_BJT_NPN = 50, // properties: Is,N,BetaF,Temp,Area
    PHY_ENGINE_E_BJT_PNP = 51, // properties: Is,N,BetaF,Temp,Area
    PHY_ENGINE_E_NMOSFET = 52, // properties: Kp,lambda,Vth
    PHY_ENGINE_E_PMOSFET = 53, // properties: Kp,lambda,Vth
    PHY_ENGINE_E_FULL_BRIDGE_RECTIFIER = 54,

    // Digital (logic)
    PHY_ENGINE_E_DIGITAL_INPUT = 200, // properties: state (0=L,1=H,2=X,3=Z)
    PHY_ENGINE_E_DIGITAL_OUTPUT = 201,
    PHY_ENGINE_E_DIGITAL_OR = 202,
    PHY_ENGINE_E_DIGITAL_YES = 203,
    PHY_ENGINE_E_DIGITAL_AND = 204,
    PHY_ENGINE_E_DIGITAL_NOT = 205,
    PHY_ENGINE_E_DIGITAL_XOR = 206,
    PHY_ENGINE_E_DIGITAL_XNOR = 207,
    PHY_ENGINE_E_DIGITAL_NAND = 208,
    PHY_ENGINE_E_DIGITAL_NOR = 209,
    PHY_ENGINE_E_DIGITAL_TRI = 210,
    PHY_ENGINE_E_DIGITAL_IMP = 211,
    PHY_ENGINE_E_DIGITAL_NIMP = 212,

    // Digital (combinational blocks)
    PHY_ENGINE_E_DIGITAL_HALF_ADDER = 220,
    PHY_ENGINE_E_DIGITAL_FULL_ADDER = 221,
    PHY_ENGINE_E_DIGITAL_HALF_SUBTRACTOR = 222,
    PHY_ENGINE_E_DIGITAL_FULL_SUBTRACTOR = 223,
    PHY_ENGINE_E_DIGITAL_MUL2 = 224,
    PHY_ENGINE_E_DIGITAL_DFF = 225,
    PHY_ENGINE_E_DIGITAL_TFF = 226,
    PHY_ENGINE_E_DIGITAL_T_BAR_FF = 227,
    PHY_ENGINE_E_DIGITAL_JKFF = 228,

    // Verilog module (only supported via `create_circuit_ex`)
    PHY_ENGINE_E_VERILOG_MODULE = 300,
};

// Create a circuit from element codes + wire connections.
// - `elements`: array of element codes, length `ele_size`; code 0 is treated as "ground placeholder" by the wiring algorithm.
// - `wires`: array of int quads (ele1,pin1,ele2,pin2); `wires_size` is the number of ints (must be multiple of 4).
// - `properties`: positional property stream consumed by element constructors.
// - `vec_pos/chunk_pos/comp_size`: outputs for locating the created models in the netlist (component order is "non-ground elements in ascending element index").
void* create_circuit(int* elements,
                     size_t ele_size,
                     int* wires,
                     size_t wires_size,
                     double* properties,
                     size_t** vec_pos,
                     size_t** chunk_pos,
                     size_t* comp_size);

// Like `create_circuit`, but supports creating Verilog modules from source text.
// - `texts/text_sizes`: string table.
// - `element_src_index/element_top_index`: per-element indices into `texts` for Verilog source + top name (used when element code is `PHY_ENGINE_E_VERILOG_MODULE`).
void* create_circuit_ex(int* elements,
                        size_t ele_size,
                        int* wires,
                        size_t wires_size,
                        double* properties,
                        char const* const* texts,
                        size_t const* text_sizes,
                        size_t text_count,
                        size_t const* element_src_index,
                        size_t const* element_top_index,
                        size_t** vec_pos,
                        size_t** chunk_pos,
                        size_t* comp_size);

void destroy_circuit(void* circuit_ptr, size_t* vec_pos, size_t* chunk_pos);

// Simulation control
int circuit_set_analyze_type(void* circuit_ptr, uint32_t analyze_type_value);
int circuit_set_tr(void* circuit_ptr, double t_step, double t_stop);
int circuit_set_ac_omega(void* circuit_ptr, double omega);
int circuit_analyze(void* circuit_ptr);
int circuit_digital_clk(void* circuit_ptr);

// Sample the current state without running `analyze()`.
// Output arrays are in component order (same order as `vec_pos/chunk_pos`).
int circuit_sample(void* circuit_ptr,
                   size_t* vec_pos,
                   size_t* chunk_pos,
                   size_t comp_size,
                   double* voltage,
                   size_t* voltage_ord,
                   double* current,
                   size_t* current_ord,
                   bool* digital,
                   size_t* digital_ord);

// Like `circuit_sample`, but writes digital pin states as bytes (0/1).
// This is friendlier for FFI/wasm bindings and avoids `std::vector<bool>` issues in C++ callers.
int circuit_sample_u8(void* circuit_ptr,
                      size_t* vec_pos,
                      size_t* chunk_pos,
                      size_t comp_size,
                      double* voltage,
                      size_t* voltage_ord,
                      double* current,
                      size_t* current_ord,
                      uint8_t* digital,
                      size_t* digital_ord);

// Set a model's digital attribute (commonly used for `PHY_ENGINE_E_DIGITAL_INPUT` at attribute_index=0).
int circuit_set_model_digital(void* circuit_ptr, size_t vec_pos, size_t chunk_pos, size_t attribute_index, uint8_t state);

// Analyze + (optional) property updates + sample.
int analyze_circuit(void* circuit_ptr,
                    size_t* vec_pos,
                    size_t* chunk_pos,
                    size_t comp_size,
                    int* changed_ele,
                    size_t* changed_ind,
                    double* changed_prop,
                    size_t prop_size,
                    double* voltage,
                    size_t* voltage_ord,
                    double* current,
                    size_t* current_ord,
                    bool* digital,
                    size_t* digital_ord);

#ifdef __cplusplus
}  // extern "C"
#endif
