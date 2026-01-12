#pragma once

#include <string_view>

// PhysicsLab "ModelID" English names that are supported by the PE adapter in `pe_sim.h`.
// Keeping these centralized makes it easier to extend PL↔PE mappings without scattering string literals.

namespace phy_engine::phy_lab_wrapper::pl_model_id
{
inline constexpr ::std::string_view ground_component = "Ground Component";

// Linear / passive
inline constexpr ::std::string_view resistor = "Resistor";
inline constexpr ::std::string_view basic_capacitor = "Basic Capacitor";
inline constexpr ::std::string_view basic_inductor = "Basic Inductor";
inline constexpr ::std::string_view battery_source = "Battery Source";

// Controllers / switches
inline constexpr ::std::string_view simple_switch = "Simple Switch";
inline constexpr ::std::string_view push_switch = "Push Switch";
inline constexpr ::std::string_view air_switch = "Air Switch";

// Coupled devices
inline constexpr ::std::string_view transformer = "Transformer";
inline constexpr ::std::string_view mutual_inductor = "Mutual Inductor";

// Non-linear convenience blocks
inline constexpr ::std::string_view rectifier = "Rectifier";

// Digital (logic circuit)
inline constexpr ::std::string_view logic_input = "Logic Input";
inline constexpr ::std::string_view logic_output = "Logic Output";

inline constexpr ::std::string_view or_gate = "Or Gate";
inline constexpr ::std::string_view yes_gate = "Yes Gate";
inline constexpr ::std::string_view and_gate = "And Gate";
inline constexpr ::std::string_view no_gate = "No Gate";
inline constexpr ::std::string_view xor_gate = "Xor Gate";
inline constexpr ::std::string_view xnor_gate = "Xnor Gate";
inline constexpr ::std::string_view nand_gate = "Nand Gate";
inline constexpr ::std::string_view nor_gate = "Nor Gate";
inline constexpr ::std::string_view imp_gate = "Imp Gate";
inline constexpr ::std::string_view nimp_gate = "Nimp Gate";

inline constexpr ::std::string_view half_adder = "Half Adder";
inline constexpr ::std::string_view full_adder = "Full Adder";
inline constexpr ::std::string_view half_subtractor = "Half Subtractor";
inline constexpr ::std::string_view full_subtractor = "Full Subtractor";
inline constexpr ::std::string_view multiplier = "Multiplier";

inline constexpr ::std::string_view d_flipflop = "D Flipflop";
inline constexpr ::std::string_view t_flipflop = "T Flipflop";
inline constexpr ::std::string_view real_t_flipflop = "Real-T Flipflop";
inline constexpr ::std::string_view jk_flipflop = "JK Flipflop";
}  // namespace phy_engine::phy_lab_wrapper::pl_model_id

