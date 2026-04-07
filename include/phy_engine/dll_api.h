#pragma once

// C ABI helpers for embedding Phy-Engine as a shared library.
// This header documents the exported functions implemented in `src/dll_main.cpp`.

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
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

    // Verilog top-level port directions (matches `phy_engine::verilog::digital::port_dir`).
    enum phy_engine_verilog_port_dir : uint8_t
    {
        PHY_ENGINE_V_PORT_UNKNOWN = 0,
        PHY_ENGINE_V_PORT_INPUT = 1,
        PHY_ENGINE_V_PORT_OUTPUT = 2,
        PHY_ENGINE_V_PORT_INOUT = 3,
    };

    // Thread-local diagnostic string for the most recent failing API call.
    // Pointer remains valid until the next API call on the same thread.
    char const* phy_engine_last_error(void);
    void phy_engine_clear_error(void);
    void phy_engine_string_free(char* s);

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
        PHY_ENGINE_E_VCCS = 8,   // properties: G
        PHY_ENGINE_E_VCVS = 9,   // properties: Mu
        PHY_ENGINE_E_CCCS = 10,  // properties: alpha
        PHY_ENGINE_E_CCVS = 11,  // properties: r

        // Controllers / switches
        PHY_ENGINE_E_SWITCH_SPST = 12,  // properties: cut_through (0/1)

        // Non-linear
        PHY_ENGINE_E_PN_JUNCTION = 13,  // properties: Is,N,Isr,Nr,Temp,Ibv,Bv,Bv_set(0/1),Area

        // Coupled devices
        PHY_ENGINE_E_TRANSFORMER = 14,        // properties: n
        PHY_ENGINE_E_COUPLED_INDUCTORS = 15,  // properties: L1,L2,k

        // Additional linear/controller/generator
        PHY_ENGINE_E_TRANSFORMER_CENTER_TAP = 16,  // properties: n_total
        PHY_ENGINE_E_OP_AMP = 17,                  // properties: mu
        PHY_ENGINE_E_RELAY = 18,                   // properties: Von,Voff
        PHY_ENGINE_E_COMPARATOR = 19,              // properties: Ll,Hl

        PHY_ENGINE_E_SAWTOOTH = 20,  // properties: Vh,Vl,freq(Hz),phase(rad)
        PHY_ENGINE_E_SQUARE = 21,    // properties: Vh,Vl,freq(Hz),duty,phase(rad)
        PHY_ENGINE_E_PULSE = 22,     // properties: Vh,Vl,freq(Hz),duty,phase(rad),tr,tf
        PHY_ENGINE_E_TRIANGLE = 23,  // properties: Vh,Vl,freq(Hz),phase(rad)

        // Additional non-linear devices
        PHY_ENGINE_E_BJT_NPN = 50,  // properties: Is,N,BetaF,Temp,Area
        PHY_ENGINE_E_BJT_PNP = 51,  // properties: Is,N,BetaF,Temp,Area
        PHY_ENGINE_E_NMOSFET = 52,  // properties: Kp,lambda,Vth
        PHY_ENGINE_E_PMOSFET = 53,  // properties: Kp,lambda,Vth
        PHY_ENGINE_E_FULL_BRIDGE_RECTIFIER = 54,
        PHY_ENGINE_E_BSIM3V32_NMOS = 55,  // properties: W,L,Kp,lambda,Vth0,gamma,phi,Cgs,Cgd,Cgb,diode_Is,diode_N,Temp
        PHY_ENGINE_E_BSIM3V32_PMOS = 56,  // properties: W,L,Kp,lambda,Vth0,gamma,phi,Cgs,Cgd,Cgb,diode_Is,diode_N,Temp

        // Digital (logic)
        PHY_ENGINE_E_DIGITAL_INPUT = 200,  // properties: state (0=L,1=H,2=X,3=Z)
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

        // Digital (utility blocks)
        PHY_ENGINE_E_DIGITAL_COUNTER4 = 229,           // properties: init_value (0..15)
        PHY_ENGINE_E_DIGITAL_RANDOM_GENERATOR4 = 230,  // properties: init_state (0..15)
        PHY_ENGINE_E_DIGITAL_EIGHT_BIT_INPUT = 231,    // properties: value (0..255)
        PHY_ENGINE_E_DIGITAL_EIGHT_BIT_DISPLAY = 232,  // properties: none
        PHY_ENGINE_E_DIGITAL_SCHMITT_TRIGGER = 233,    // properties: Vth_low,Vth_high,inverted(0/1),Ll,Hl

        // Verilog module (only supported via `create_circuit_ex`)
        PHY_ENGINE_E_VERILOG_MODULE = 300,
        // Verilog module synthesized into PE digital primitives (supported via `create_circuit_ex`)
        PHY_ENGINE_E_VERILOG_NETLIST = 301,
    };

    // Create a circuit from element codes + wire connections.
    // - `elements`: array of element codes, length `ele_size`; code 0 is treated as "ground placeholder" by the wiring algorithm.
    // - `wires`: array of int quads (ele1,pin1,ele2,pin2); `wires_size` is the number of ints (must be multiple of 4).
    // - `properties`: positional property stream consumed by element constructors.
    // - `vec_pos/chunk_pos/comp_size`: outputs for locating the created models in the netlist (component order is "non-ground elements in ascending element
    // index").
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
    // - `element_src_index/element_top_index`: per-element indices into `texts` for Verilog source + top name (used when element code is
    // `PHY_ENGINE_E_VERILOG_MODULE`).
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
    int circuit_set_temperature(void* circuit_ptr, double temp_c);
    int circuit_set_tnom(void* circuit_ptr, double tnom_c);
    int circuit_set_model_double_by_name(void* circuit_ptr, size_t vec_pos, size_t chunk_pos, char const* name, size_t name_size, double value);
    int circuit_analyze(void* circuit_ptr);
    int circuit_digital_clk(void* circuit_ptr);

    // Query prefix-sum layouts used by `circuit_sample()` / `circuit_sample_u8()`.
    // - `voltage_ord/current_ord/digital_ord` must each have length `comp_size + 1`.
    // - `voltage_ord[i+1]-voltage_ord[i]` is the pin count of component i.
    // - `current_ord[i+1]-current_ord[i]` is the branch count of component i.
    int circuit_sample_layout(void* circuit_ptr,
                              size_t* vec_pos,
                              size_t* chunk_pos,
                              size_t comp_size,
                              size_t* voltage_ord,
                              size_t* current_ord,
                              size_t* digital_ord);

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

    // Like `circuit_sample_u8`, but preserves 4-state digital values:
    // 0=L, 1=H, 2=X, 3=Z. Non-pure-digital/hybrid pins currently report X.
    int circuit_sample_digital_state_u8(void* circuit_ptr,
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

    // Global defaults used by `PHY_ENGINE_E_VERILOG_NETLIST` inside `create_circuit_ex()`.
    void verilog_synth_set_opt_level(uint8_t level);
    uint8_t verilog_synth_get_opt_level(void);
    void verilog_synth_set_assume_binary_inputs(bool value);
    bool verilog_synth_get_assume_binary_inputs(void);
    void verilog_synth_set_allow_inout(bool value);
    bool verilog_synth_get_allow_inout(void);
    void verilog_synth_set_allow_multi_driver(bool value);
    bool verilog_synth_get_allow_multi_driver(void);
    void verilog_synth_set_optimize_wires(bool value);
    bool verilog_synth_get_optimize_wires(void);
    void verilog_synth_set_optimize_mul2(bool value);
    bool verilog_synth_get_optimize_mul2(void);
    void verilog_synth_set_optimize_adders(bool value);
    bool verilog_synth_get_optimize_adders(void);
    void verilog_synth_set_loop_unroll_limit(size_t n);
    size_t verilog_synth_get_loop_unroll_limit(void);

    // Verilog runtime: compile/elaborate one top module and simulate it directly.
    // - `include_dirs/include_dir_sizes`: optional include search roots used by `` `include ``.
    // - `top`: optional top module name; empty means: prefer "top", otherwise first compiled module.
    // - On failure returns null and updates `phy_engine_last_error()`.
    void* verilog_runtime_create(char const* src,
                                 size_t src_size,
                                 char const* top,
                                 size_t top_size,
                                 char const* const* include_dirs,
                                 size_t const* include_dir_sizes,
                                 size_t include_dir_count);
    void verilog_runtime_destroy(void* runtime_ptr);

    // Simulation control.
    uint64_t verilog_runtime_get_tick(void* runtime_ptr);
    int verilog_runtime_reset(void* runtime_ptr);
    int verilog_runtime_step(void* runtime_ptr, uint64_t tick, uint8_t process_sequential);
    int verilog_runtime_tick(void* runtime_ptr);  // increments internal tick then simulates with sequential logic enabled

    // Design/runtime introspection.
    size_t verilog_runtime_module_count(void* runtime_ptr);
    size_t verilog_runtime_port_count(void* runtime_ptr);
    size_t verilog_runtime_signal_count(void* runtime_ptr);

    size_t verilog_runtime_preprocessed_size(void* runtime_ptr);
    int verilog_runtime_copy_preprocessed(void* runtime_ptr, char* out, size_t out_size);

    size_t verilog_runtime_top_module_name_size(void* runtime_ptr);
    int verilog_runtime_copy_top_module_name(void* runtime_ptr, char* out, size_t out_size);

    size_t verilog_runtime_module_name_size(void* runtime_ptr, size_t module_index);
    int verilog_runtime_copy_module_name(void* runtime_ptr, size_t module_index, char* out, size_t out_size);

    size_t verilog_runtime_port_name_size(void* runtime_ptr, size_t port_index);
    int verilog_runtime_copy_port_name(void* runtime_ptr, size_t port_index, char* out, size_t out_size);
    uint8_t verilog_runtime_port_dir(void* runtime_ptr, size_t port_index);
    uint8_t verilog_runtime_get_port_value(void* runtime_ptr, size_t port_index);
    int verilog_runtime_set_port_value(void* runtime_ptr, size_t port_index, uint8_t state);

    size_t verilog_runtime_signal_name_size(void* runtime_ptr, size_t signal_index);
    int verilog_runtime_copy_signal_name(void* runtime_ptr, size_t signal_index, char* out, size_t out_size);
    uint8_t verilog_runtime_get_signal_value(void* runtime_ptr, size_t signal_index);
    int verilog_runtime_set_signal_value(void* runtime_ptr, size_t signal_index, uint8_t state);

    // PhysicsLab experiment handle (`phy_engine::phy_lab_wrapper::experiment`).
    void* pl_experiment_create(int type_value);
    void* pl_experiment_load_from_string(char const* sav_json, size_t sav_json_size);
    void* pl_experiment_load_from_file(char const* path, size_t path_size);
    void pl_experiment_destroy(void* experiment_ptr);

    char* pl_experiment_dump(void* experiment_ptr, int indent);
    int pl_experiment_save(void* experiment_ptr, char const* path, size_t path_size, int indent);

    char* pl_experiment_add_circuit_element(void* experiment_ptr,
                                           char const* model_id,
                                           size_t model_id_size,
                                           double x,
                                           double y,
                                           double z,
                                           uint8_t element_xyz_coords,
                                           uint8_t is_big_element,
                                           uint8_t participate_in_layout);
    int pl_experiment_connect(void* experiment_ptr,
                              char const* src_id,
                              size_t src_id_size,
                              int src_pin,
                              char const* dst_id,
                              size_t dst_id_size,
                              int dst_pin,
                              int color_value);
    int pl_experiment_clear_wires(void* experiment_ptr);
    int pl_experiment_set_xyz_precision(void* experiment_ptr, int decimals);
    int pl_experiment_set_element_xyz(void* experiment_ptr, uint8_t enabled, double origin_x, double origin_y, double origin_z);
    int pl_experiment_set_camera(void* experiment_ptr,
                                 double vision_center_x,
                                 double vision_center_y,
                                 double vision_center_z,
                                 double target_rotation_x,
                                 double target_rotation_y,
                                 double target_rotation_z);
    int pl_experiment_set_element_property_number(void* experiment_ptr,
                                                  char const* element_id,
                                                  size_t element_id_size,
                                                  char const* key,
                                                  size_t key_size,
                                                  double value);
    int pl_experiment_set_element_label(void* experiment_ptr,
                                        char const* element_id,
                                        size_t element_id_size,
                                        char const* label,
                                        size_t label_size);
    int pl_experiment_set_element_position(void* experiment_ptr,
                                           char const* element_id,
                                           size_t element_id_size,
                                           double x,
                                           double y,
                                           double z,
                                           uint8_t element_xyz_coords);
    int pl_experiment_merge(void* dst_experiment_ptr, void* src_experiment_ptr, double offset_x, double offset_y, double offset_z);

    // PhysicsLab -> PE simulation handle (`phy_engine::phy_lab_wrapper::pe::circuit`).
    void* pl_pe_circuit_build(void* experiment_ptr);
    void pl_pe_circuit_destroy(void* pe_circuit_ptr);
    size_t pl_pe_circuit_comp_size(void* pe_circuit_ptr);
    int pl_pe_circuit_set_analyze_type(void* pe_circuit_ptr, uint32_t analyze_type_value);
    int pl_pe_circuit_set_tr(void* pe_circuit_ptr, double t_step, double t_stop);
    int pl_pe_circuit_set_ac_omega(void* pe_circuit_ptr, double omega);
    int pl_pe_circuit_analyze(void* pe_circuit_ptr);
    int pl_pe_circuit_digital_clk(void* pe_circuit_ptr);
    int pl_pe_circuit_sync_inputs_from_pl(void* pe_circuit_ptr, void* experiment_ptr);
    int pl_pe_circuit_write_back_to_pl(void* pe_circuit_ptr, void* experiment_ptr);
    int pl_pe_circuit_write_back_to_pl_ex(void* pe_circuit_ptr,
                                          void* experiment_ptr,
                                          double logic_output_low,
                                          double logic_output_high,
                                          double logic_output_x,
                                          double logic_output_z);
    int pl_pe_circuit_sample_layout(void* pe_circuit_ptr,
                                    size_t* voltage_ord,
                                    size_t* current_ord,
                                    size_t* digital_ord);
    int pl_pe_circuit_sample_u8(void* pe_circuit_ptr,
                                double* voltage,
                                size_t* voltage_ord,
                                double* current,
                                size_t* current_ord,
                                uint8_t* digital,
                                size_t* digital_ord);
    // Like `pl_pe_circuit_sample_u8`, but preserves 4-state digital values:
    // 0=L, 1=H, 2=X, 3=Z.
    int pl_pe_circuit_sample_digital_state_u8(void* pe_circuit_ptr,
                                              double* voltage,
                                              size_t* voltage_ord,
                                              double* current,
                                              size_t* current_ord,
                                              uint8_t* digital,
                                              size_t* digital_ord);

    // PE -> PhysicsLab export.
    void* pe_to_pl_convert(void* circuit_ptr,
                           double fixed_x,
                           double fixed_y,
                           double fixed_z,
                           uint8_t element_xyz_coords,
                           uint8_t keep_pl_macros,
                           uint8_t include_linear,
                           uint8_t include_ground,
                           uint8_t generate_wires,
                           uint8_t keep_unknown_as_placeholders,
                           uint8_t drop_dangling_logic_inputs);

    // PhysicsLab auto-layout.
    int pl_experiment_auto_layout(void* experiment_ptr,
                                  double corner0_x,
                                  double corner0_y,
                                  double corner0_z,
                                  double corner1_x,
                                  double corner1_y,
                                  double corner1_z,
                                  double z_fixed,
                                  int backend_value,
                                  int mode_value,
                                  double step_x,
                                  double step_y,
                                  double margin_x,
                                  double margin_y,
                                  size_t* out_grid_w,
                                  size_t* out_grid_h,
                                  size_t* out_fixed_obstacles,
                                  size_t* out_placed,
                                  size_t* out_skipped);

#ifdef __cplusplus
}  // extern "C"
#endif
