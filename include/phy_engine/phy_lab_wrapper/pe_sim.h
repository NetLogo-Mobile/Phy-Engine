#pragma once

// Adapter: PhysicsLab .sav (PL) ↔ Phy-Engine (PE) simulation via the C ABI (`dll_api.h`).
// This file is optional: it requires linking with the implementation of `create_circuit()/analyze_circuit()/...`
// (provided by `src/dll_main.cpp` in this repo, or the built shared library).

#include "physicslab.h"

#include <phy_engine/phy_engine.h>
#include <phy_engine/dll_api.h>
#include <phy_engine/netlist/operation.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace phy_engine::phy_lab_wrapper::pe
{
using json = nlohmann::json;

struct endpoint
{
    std::size_t element_index{};
    int pin{};
};

struct sample
{
    std::vector<double> pin_voltage;
    std::vector<std::size_t> pin_voltage_ord;  // comp_size+1

    std::vector<double> branch_current;
    std::vector<std::size_t> branch_current_ord;  // comp_size+1

    std::vector<std::uint8_t> pin_digital;  // 0/1
    std::vector<std::size_t> pin_digital_ord;  // comp_size+1
};

namespace detail
{
inline double to_double(json const& v)
{
    if (v.is_number_float() || v.is_number_integer() || v.is_number_unsigned())
    {
        return v.get<double>();
    }
    if (v.is_boolean())
    {
        return v.get<bool>() ? 1.0 : 0.0;
    }
    if (v.is_string())
    {
        auto const& s = v.get_ref<const std::string&>();
        std::size_t idx{};
        double d = std::stod(s, &idx);
        if (idx != s.size())
        {
            throw std::runtime_error("invalid numeric string: " + s);
        }
        return d;
    }
    throw std::runtime_error("value is not numeric");
}

inline double get_required_float(json const& element_data, std::string_view key)
{
    auto it_props = element_data.find("Properties");
    if (it_props == element_data.end() || !it_props->is_object())
    {
        throw std::runtime_error("element missing Properties");
    }
    auto it = it_props->find(std::string(key));
    if (it == it_props->end())
    {
        throw std::runtime_error("element missing property: " + std::string(key));
    }
    return to_double(*it);
}

inline int get_required_int01(json const& element_data, std::string_view key)
{
    auto v = get_required_float(element_data, key);
    return (v != 0.0) ? 1 : 0;
}

inline std::pair<int, std::vector<double>> to_phy_engine_code_and_props(json const& element_data)
{
    auto model_id = element_data.value("ModelID", "");
    if (model_id.empty())
    {
        throw std::runtime_error("element missing ModelID");
    }

    // Ground is represented as code 0 (special-cased by the wiring algorithm).
    if (model_id == "Ground Component")
    {
        return {0, {}};
    }

    // Linear / passive
    if (model_id == "Resistor")
    {
        return {PHY_ENGINE_E_RESISTOR, {get_required_float(element_data, "电阻")}};
    }
    if (model_id == "Basic Capacitor")
    {
        return {PHY_ENGINE_E_CAPACITOR, {get_required_float(element_data, "电容")}};
    }
    if (model_id == "Basic Inductor")
    {
        return {PHY_ENGINE_E_INDUCTOR, {get_required_float(element_data, "电感")}};
    }
    if (model_id == "Battery Source")
    {
        return {PHY_ENGINE_E_VDC, {get_required_float(element_data, "电压")}};
    }

    // Controller
    if (model_id == "Simple Switch" || model_id == "Push Switch" || model_id == "Air Switch")
    {
        return {PHY_ENGINE_E_SWITCH_SPST, {static_cast<double>(get_required_int01(element_data, "开关"))}};
    }

    // Coupled devices
    if (model_id == "Transformer")
    {
        auto vp = get_required_float(element_data, "输入电压");
        auto vs = get_required_float(element_data, "输出电压");
        if (vs == 0.0)
        {
            throw std::runtime_error("Transformer 输出电压 must be non-zero");
        }
        return {PHY_ENGINE_E_TRANSFORMER, {vp / vs}};  // n = Vp/Vs
    }
    if (model_id == "Mutual Inductor")
    {
        return {PHY_ENGINE_E_COUPLED_INDUCTORS,
                {get_required_float(element_data, "电感1"), get_required_float(element_data, "电感2"), get_required_float(element_data, "耦合系数")}};
    }

    // Non-linear convenience blocks
    if (model_id == "Rectifier")
    {
        return {PHY_ENGINE_E_FULL_BRIDGE_RECTIFIER, {}};
    }

    // Digital (logic circuit)
    if (model_id == "Logic Input")
    {
        int state = get_required_int01(element_data, "开关") ? 1 : 0;
        return {PHY_ENGINE_E_DIGITAL_INPUT, {static_cast<double>(state)}};
    }
    if (model_id == "Logic Output") return {PHY_ENGINE_E_DIGITAL_OUTPUT, {}};
    if (model_id == "Or Gate") return {PHY_ENGINE_E_DIGITAL_OR, {}};
    if (model_id == "Yes Gate") return {PHY_ENGINE_E_DIGITAL_YES, {}};
    if (model_id == "And Gate") return {PHY_ENGINE_E_DIGITAL_AND, {}};
    if (model_id == "No Gate") return {PHY_ENGINE_E_DIGITAL_NOT, {}};
    if (model_id == "Xor Gate") return {PHY_ENGINE_E_DIGITAL_XOR, {}};
    if (model_id == "Xnor Gate") return {PHY_ENGINE_E_DIGITAL_XNOR, {}};
    if (model_id == "Nand Gate") return {PHY_ENGINE_E_DIGITAL_NAND, {}};
    if (model_id == "Nor Gate") return {PHY_ENGINE_E_DIGITAL_NOR, {}};
    if (model_id == "Imp Gate") return {PHY_ENGINE_E_DIGITAL_IMP, {}};
    if (model_id == "Nimp Gate") return {PHY_ENGINE_E_DIGITAL_NIMP, {}};

    if (model_id == "Half Adder") return {PHY_ENGINE_E_DIGITAL_HALF_ADDER, {}};
    if (model_id == "Full Adder") return {PHY_ENGINE_E_DIGITAL_FULL_ADDER, {}};
    if (model_id == "Half Subtractor") return {PHY_ENGINE_E_DIGITAL_HALF_SUBTRACTOR, {}};
    if (model_id == "Full Subtractor") return {PHY_ENGINE_E_DIGITAL_FULL_SUBTRACTOR, {}};
    if (model_id == "Multiplier") return {PHY_ENGINE_E_DIGITAL_MUL2, {}};

    if (model_id == "D Flipflop") return {PHY_ENGINE_E_DIGITAL_DFF, {}};
    if (model_id == "T Flipflop") return {PHY_ENGINE_E_DIGITAL_TFF, {}};
    if (model_id == "Real-T Flipflop") return {PHY_ENGINE_E_DIGITAL_T_BAR_FF, {}};
    if (model_id == "JK Flipflop") return {PHY_ENGINE_E_DIGITAL_JKFF, {}};

    // Higher-level modules are expanded by the adapter (see `circuit::build_`).
    if (model_id == "Counter" || model_id == "Random Generator" || model_id == "8bit Input" || model_id == "8bit Display")
    {
        throw std::runtime_error("internal: high-level module should be expanded by adapter: " + model_id);
    }

    throw std::runtime_error("Phy-Engine backend does not support element ModelID=" + model_id);
}

inline std::size_t expected_prop_arity(int element_code)
{
    switch (element_code)
    {
        case 0: return 0;
        case PHY_ENGINE_E_RESISTOR: return 1;
        case PHY_ENGINE_E_CAPACITOR: return 1;
        case PHY_ENGINE_E_INDUCTOR: return 1;
        case PHY_ENGINE_E_VDC: return 1;
        case PHY_ENGINE_E_SWITCH_SPST: return 1;
        case PHY_ENGINE_E_TRANSFORMER: return 1;
        case PHY_ENGINE_E_COUPLED_INDUCTORS: return 3;
        case PHY_ENGINE_E_FULL_BRIDGE_RECTIFIER: return 0;
        case PHY_ENGINE_E_DIGITAL_INPUT: return 1;

        case PHY_ENGINE_E_DIGITAL_OUTPUT: return 0;
        case PHY_ENGINE_E_DIGITAL_OR: return 0;
        case PHY_ENGINE_E_DIGITAL_YES: return 0;
        case PHY_ENGINE_E_DIGITAL_AND: return 0;
        case PHY_ENGINE_E_DIGITAL_NOT: return 0;
        case PHY_ENGINE_E_DIGITAL_XOR: return 0;
        case PHY_ENGINE_E_DIGITAL_XNOR: return 0;
        case PHY_ENGINE_E_DIGITAL_NAND: return 0;
        case PHY_ENGINE_E_DIGITAL_NOR: return 0;
        case PHY_ENGINE_E_DIGITAL_IMP: return 0;
        case PHY_ENGINE_E_DIGITAL_NIMP: return 0;

        case PHY_ENGINE_E_DIGITAL_HALF_ADDER: return 0;
        case PHY_ENGINE_E_DIGITAL_FULL_ADDER: return 0;
        case PHY_ENGINE_E_DIGITAL_HALF_SUBTRACTOR: return 0;
        case PHY_ENGINE_E_DIGITAL_FULL_SUBTRACTOR: return 0;
        case PHY_ENGINE_E_DIGITAL_MUL2: return 0;
        case PHY_ENGINE_E_DIGITAL_DFF: return 0;
        case PHY_ENGINE_E_DIGITAL_TFF: return 0;
        case PHY_ENGINE_E_DIGITAL_T_BAR_FF: return 0;
        case PHY_ENGINE_E_DIGITAL_JKFF: return 0;
        case PHY_ENGINE_E_DIGITAL_COUNTER4: return 1;
        case PHY_ENGINE_E_DIGITAL_RANDOM_GENERATOR4: return 1;
        default: throw std::runtime_error("unknown property arity for PE element code: " + std::to_string(element_code));
    }
}

inline void ensure_object(json& j, char const* key)
{
    auto it = j.find(key);
    if (it == j.end() || !it->is_object())
    {
        j[key] = json::object();
    }
}
}  // namespace detail

class circuit
{
public:
    static circuit build_from(experiment const& ex)
    {
        if (ex.type() != experiment_type::circuit)
        {
            throw std::runtime_error("pe::circuit only supports circuit experiments");
        }

        circuit out;
        out.build_(ex);
        return out;
    }

    circuit() = default;
    circuit(circuit&& other) noexcept { *this = std::move(other); }
    circuit& operator=(circuit&& other) noexcept
    {
        if (this == &other) return *this;
        close();
        circuit_ptr_ = other.circuit_ptr_;
        vec_pos_ = other.vec_pos_;
        chunk_pos_ = other.chunk_pos_;
        comp_size_ = other.comp_size_;
        comp_element_ids_ = std::move(other.comp_element_ids_);
        comp_codes_ = std::move(other.comp_codes_);
        other.circuit_ptr_ = nullptr;
        other.vec_pos_ = nullptr;
        other.chunk_pos_ = nullptr;
        other.comp_size_ = 0;
        return *this;
    }

    circuit(circuit const&) = delete;
    circuit& operator=(circuit const&) = delete;

    ~circuit() { close(); }

    void close() noexcept
    {
        if (circuit_ptr_ != nullptr)
        {
            destroy_circuit(circuit_ptr_, vec_pos_, chunk_pos_);
            circuit_ptr_ = nullptr;
            vec_pos_ = nullptr;
            chunk_pos_ = nullptr;
            comp_size_ = 0;
        }
    }

    [[nodiscard]] std::size_t comp_size() const noexcept { return comp_size_; }

    void set_analyze_type(phy_engine_analyze_type type)
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");
        if (circuit_set_analyze_type(circuit_ptr_, static_cast<std::uint32_t>(type)) != 0)
        {
            throw std::runtime_error("circuit_set_analyze_type failed");
        }
    }

    void set_tr(double t_step, double t_stop)
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");
        if (circuit_set_tr(circuit_ptr_, t_step, t_stop) != 0)
        {
            throw std::runtime_error("circuit_set_tr failed");
        }
    }

    void set_ac_omega(double omega)
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");
        if (circuit_set_ac_omega(circuit_ptr_, omega) != 0)
        {
            throw std::runtime_error("circuit_set_ac_omega failed");
        }
    }

    void analyze()
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");
        if (circuit_analyze(circuit_ptr_) != 0)
        {
            throw std::runtime_error("circuit_analyze failed");
        }
    }

    void digital_clk()
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");
        if (circuit_digital_clk(circuit_ptr_) != 0)
        {
            throw std::runtime_error("circuit_digital_clk failed");
        }
    }

    sample sample_now() const
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");

        sample s;
        s.pin_voltage_ord.assign(comp_size_ + 1, 0);
        s.branch_current_ord.assign(comp_size_ + 1, 0);
        s.pin_digital_ord.assign(comp_size_ + 1, 0);

        auto* c = static_cast<::phy_engine::circult*>(circuit_ptr_);
        auto& nl = c->get_netlist();

        std::size_t total_pins{};
        std::size_t total_branches{};
        for (std::size_t i{}; i < comp_size_; ++i)
        {
            auto* model = ::phy_engine::netlist::get_model(nl, ::phy_engine::netlist::model_pos{vec_pos_[i], chunk_pos_[i]});
            if (model == nullptr || model->ptr == nullptr)
            {
                continue;
            }
            total_pins += model->ptr->generate_pin_view().size;
            total_branches += model->ptr->generate_branch_view().size;
        }

        // The C ABI requires non-null pointers even when the logical size is 0.
        s.pin_voltage.assign(total_pins == 0 ? 1 : total_pins, 0.0);
        s.branch_current.assign(total_branches == 0 ? 1 : total_branches, 0.0);
        s.pin_digital.assign(total_pins == 0 ? 1 : total_pins, 0);

        if (circuit_sample_u8(circuit_ptr_,
                              vec_pos_,
                              chunk_pos_,
                              comp_size_,
                              s.pin_voltage.data(),
                              s.pin_voltage_ord.data(),
                              s.branch_current.data(),
                              s.branch_current_ord.data(),
                              s.pin_digital.data(),
                              s.pin_digital_ord.data()) != 0)
        {
            throw std::runtime_error("circuit_sample_u8 failed");
        }

        s.pin_voltage.resize(s.pin_voltage_ord.back());
        s.branch_current.resize(s.branch_current_ord.back());
        s.pin_digital.resize(s.pin_digital_ord.back());
        return s;
    }

    void sync_inputs_from_pl(experiment const& ex)
    {
        if (circuit_ptr_ == nullptr) throw std::runtime_error("circuit is closed");
        if (ex.type() != experiment_type::circuit) throw std::runtime_error("expected circuit experiment");

        for (std::size_t i{}; i < comp_size_; ++i)
        {
            if (comp_codes_[i] != PHY_ENGINE_E_DIGITAL_INPUT)
            {
                continue;
            }
            if (comp_element_ids_[i].empty())
            {
                continue;
            }

            auto const& el = ex.get_element(comp_element_ids_[i]).data();
            int state01 = detail::get_required_int01(el, "开关") ? 1 : 0;
            if (circuit_set_model_digital(circuit_ptr_, vec_pos_[i], chunk_pos_[i], 0, static_cast<std::uint8_t>(state01)) != 0)
            {
                throw std::runtime_error("circuit_set_model_digital failed");
            }
        }
    }

    void write_back_to_pl(experiment& ex, sample const& s) const
    {
        if (ex.type() != experiment_type::circuit) throw std::runtime_error("expected circuit experiment");
        if (s.pin_voltage_ord.size() != comp_size_ + 1 || s.branch_current_ord.size() != comp_size_ + 1 || s.pin_digital_ord.size() != comp_size_ + 1)
        {
            throw std::runtime_error("invalid sample ord sizes");
        }

        for (std::size_t i{}; i < comp_size_; ++i)
        {
            if (comp_element_ids_[i].empty())
            {
                continue;
            }

            auto& el = ex.get_element(comp_element_ids_[i]).data();
            auto pins_n = s.pin_voltage_ord[i + 1] - s.pin_voltage_ord[i];
            auto branches_n = s.branch_current_ord[i + 1] - s.branch_current_ord[i];

            double v0 = (pins_n >= 1) ? s.pin_voltage[s.pin_voltage_ord[i] + 0] : 0.0;
            double v1 = (pins_n >= 2) ? s.pin_voltage[s.pin_voltage_ord[i] + 1] : 0.0;
            double dv = (pins_n >= 2) ? (v0 - v1) : v0;

            double i0 = (branches_n >= 1) ? s.branch_current[s.branch_current_ord[i] + 0] : 0.0;

            if (comp_codes_[i] == PHY_ENGINE_E_DIGITAL_OUTPUT)
            {
                auto d_n = s.pin_digital_ord[i + 1] - s.pin_digital_ord[i];
                bool bit = (d_n >= 1) ? (s.pin_digital[s.pin_digital_ord[i] + 0] != 0) : false;
                detail::ensure_object(el, "Properties");
                el["Properties"]["状态"] = bit ? 1.0 : 0.0;
                continue;
            }

            // Best-effort statistics update (many PL elements use these keys).
            detail::ensure_object(el, "Statistics");
            auto& st = el["Statistics"];
            st["电压"] = dv;
            st["电流"] = i0;
            st["功率"] = dv * i0;
        }
    }

private:
    void build_(experiment const& ex)
    {
        auto const& els = ex.elements();
        if (els.empty())
        {
            throw std::runtime_error("experiment has no elements");
        }

        std::unordered_map<std::string, std::size_t> pl_id_to_pl_index;
        pl_id_to_pl_index.reserve(els.size());
        for (std::size_t i{}; i < els.size(); ++i)
        {
            pl_id_to_pl_index.emplace(els[i].identifier(), i);
        }

        std::unordered_map<std::string, std::unordered_map<int, endpoint>> pin_map;
        pin_map.reserve(els.size());

        std::vector<int> element_codes;
        std::vector<double> properties;
        std::vector<std::string> comp_element_ids;
        std::vector<int> comp_codes;

        std::vector<int> internal_wires_flat;

        auto add_pe_element = [&](int code, std::vector<double> props, std::string bind_id) -> std::size_t {
            std::size_t idx = element_codes.size();
            element_codes.push_back(code);
            if (code != 0)
            {
                auto expected = detail::expected_prop_arity(code);
                if (props.size() != expected)
                {
                    throw std::runtime_error("element has wrong property count for PE code=" + std::to_string(code));
                }
                properties.insert(properties.end(), props.begin(), props.end());
                comp_codes.push_back(code);
                comp_element_ids.push_back(std::move(bind_id));  // may be empty for internal elements
            }
            return idx;
        };

        auto add_wire = [&](endpoint a, endpoint b) {
            internal_wires_flat.push_back(static_cast<int>(a.element_index));
            internal_wires_flat.push_back(a.pin);
            internal_wires_flat.push_back(static_cast<int>(b.element_index));
            internal_wires_flat.push_back(b.pin);
        };

        // For optional pins on "macro" elements, determine whether they are connected by scanning wires.
        std::unordered_map<std::string, std::unordered_map<int, bool>> pl_pin_used;
        for (auto const& w : ex.wires())
        {
            pl_pin_used[w.source.element_identifier][w.source.pin] = true;
            pl_pin_used[w.target.element_identifier][w.target.pin] = true;
        }

        // Expand PL elements into PE elements (some are 1:1, some are macro expansions).
        for (auto const& e : els)
        {
            auto const pl_id = e.identifier();
            auto const model_id = e.data().value("ModelID", "");

            // ---- PE primitive: 4-bit counter (Counter) ----
            // Implemented via `PHY_ENGINE_E_DIGITAL_COUNTER4` (COUNTER4), not a hand-built macro,
            // so behavior matches PE's digital model library.
            // PL pins (by convention in physicsLab):
            //   outputs: 0=o_up(MSB),1=o_upmid,2=o_lowmid,3=o_low(LSB)
            //   inputs : 4=i_up(clock),5=i_low(enable; if unconnected, treated as enable=1)
            if (model_id == "Counter")
            {
                auto ctr = add_pe_element(PHY_ENGINE_E_DIGITAL_COUNTER4, {0.0}, "");

                // outputs (MSB..LSB): COUNTER4 pins 0..3 are q3..q0.
                pin_map[pl_id][0] = endpoint{ctr, 0};
                pin_map[pl_id][1] = endpoint{ctr, 1};
                pin_map[pl_id][2] = endpoint{ctr, 2};
                pin_map[pl_id][3] = endpoint{ctr, 3};

                // clock/en
                pin_map[pl_id][4] = endpoint{ctr, 4};
                pin_map[pl_id][5] = endpoint{ctr, 5};
                continue;
            }

            // ---- Macro: 4-bit LFSR-like generator (Random Generator) ----
            // PL pins:
            //   outputs: 0=o_up(MSB),1=o_upmid,2=o_lowmid,3=o_low(LSB)
            //   inputs : 4=i_up(clock),5=i_low(reset_n; active-low; if unconnected, treated as reset_n=1)
            if (model_id == "Random Generator")
            {
                bool rstn_connected = pl_pin_used[pl_id][5];

                // PE primitive: `PHY_ENGINE_E_DIGITAL_RANDOM_GENERATOR4` (RANDOM_GENERATOR4),
                // so it stays compact and matches the PE digital model library.
                auto rng = add_pe_element(PHY_ENGINE_E_DIGITAL_RANDOM_GENERATOR4, {1.0}, "");

                // outputs (MSB..LSB): RANDOM_GENERATOR4 pins 0..3 are q3..q0.
                pin_map[pl_id][0] = endpoint{rng, 0};
                pin_map[pl_id][1] = endpoint{rng, 1};
                pin_map[pl_id][2] = endpoint{rng, 2};
                pin_map[pl_id][3] = endpoint{rng, 3};

                // clk/reset_n
                pin_map[pl_id][4] = endpoint{rng, 4};
                pin_map[pl_id][5] = endpoint{rng, 5};

                if (!rstn_connected)
                {
                    auto const1 = add_pe_element(PHY_ENGINE_E_DIGITAL_INPUT, {1.0}, "");
                    add_wire(endpoint{const1, 0}, endpoint{rng, 5});
                }
                continue;
            }

            

            // ---- PL macro: D Flipflop (DFF + optional ~Q inverter) ----
            // physicsLab pin order:
            //   outputs: 0=o_up(Q), 1=o_low(~Q)
            //   inputs : 2=i_up(D), 3=i_low(CLK)
            // PE DFF pins: 0=d, 1=clk, 2=q
            if (model_id == "D Flipflop")
            {
                auto dff = add_pe_element(PHY_ENGINE_E_DIGITAL_DFF, {}, pl_id);
                endpoint d{dff, 0};
                endpoint clk{dff, 1};
                endpoint q{dff, 2};

                pin_map[pl_id][2] = d;
                pin_map[pl_id][3] = clk;
                pin_map[pl_id][0] = q;

                bool nq_used{};
                if (auto it_used = pl_pin_used.find(pl_id); it_used != pl_pin_used.end())
                {
                    if (auto it = it_used->second.find(1); it != it_used->second.end() && it->second)
                    {
                        nq_used = true;
                    }
                }

                if (nq_used)
                {
                    auto inv = add_pe_element(PHY_ENGINE_E_DIGITAL_NOT, {}, "");
                    // NOT pins: 0=i, 1=o
                    add_wire(q, endpoint{inv, 0});
                    pin_map[pl_id][1] = endpoint{inv, 1};
                }

                continue;
            }

            // ---- PL macro: Half/Full Adder/Subtractor (pin order differs from PE model pin order) ----
            //
            // physicsLab(Half Adder):
            //   outputs: 0=S, 1=C
            //   inputs : 2=B, 3=A
            // PE(HALF_ADDER): ia(A), ib(B), s(S), c(C)
            if (model_id == "Half Adder")
            {
                auto fa = add_pe_element(PHY_ENGINE_E_DIGITAL_HALF_ADDER, {}, pl_id);
                pin_map[pl_id][3] = endpoint{fa, 0};  // A -> ia
                pin_map[pl_id][2] = endpoint{fa, 1};  // B -> ib
                pin_map[pl_id][0] = endpoint{fa, 2};  // S -> s
                pin_map[pl_id][1] = endpoint{fa, 3};  // C -> c
                continue;
            }

            // physicsLab(Full Adder):
            //   outputs: 0=S, 1=Cout
            //   inputs : 2=B, 3=Cin, 4=A
            // PE(FULL_ADDER): ia(A), ib(B), cin(Cin), s(S), cout(Cout)
            if (model_id == "Full Adder")
            {
                auto fa = add_pe_element(PHY_ENGINE_E_DIGITAL_FULL_ADDER, {}, pl_id);
                pin_map[pl_id][4] = endpoint{fa, 0};  // A -> ia
                pin_map[pl_id][2] = endpoint{fa, 1};  // B -> ib
                pin_map[pl_id][3] = endpoint{fa, 2};  // Cin -> cin
                pin_map[pl_id][0] = endpoint{fa, 3};  // S -> s
                pin_map[pl_id][1] = endpoint{fa, 4};  // Cout -> cout
                continue;
            }

            // physicsLab(Half Subtractor):
            //   outputs: 0=D, 1=Bout
            //   inputs : 2=B, 3=A
            // PE(HALF_SUB): ia(A), ib(B), d(D), b(Bout)
            if (model_id == "Half Subtractor")
            {
                auto hs = add_pe_element(PHY_ENGINE_E_DIGITAL_HALF_SUBTRACTOR, {}, pl_id);
                pin_map[pl_id][3] = endpoint{hs, 0};  // A -> ia
                pin_map[pl_id][2] = endpoint{hs, 1};  // B -> ib
                pin_map[pl_id][0] = endpoint{hs, 2};  // D -> d
                pin_map[pl_id][1] = endpoint{hs, 3};  // Bout -> b
                continue;
            }

            // physicsLab(Full Subtractor):
            //   outputs: 0=D, 1=Bout
            //   inputs : 2=B, 3=Bin, 4=A
            // PE(FULL_SUB): ia(A), ib(B), bin(Bin), d(D), bout(Bout)
            if (model_id == "Full Subtractor")
            {
                auto fs = add_pe_element(PHY_ENGINE_E_DIGITAL_FULL_SUBTRACTOR, {}, pl_id);
                pin_map[pl_id][4] = endpoint{fs, 0};  // A -> ia
                pin_map[pl_id][2] = endpoint{fs, 1};  // B -> ib
                pin_map[pl_id][3] = endpoint{fs, 2};  // Bin -> bin
                pin_map[pl_id][0] = endpoint{fs, 3};  // D -> d
                pin_map[pl_id][1] = endpoint{fs, 4};  // Bout -> bout
                continue;
            }
// 1:1 element mapping
            auto [code, props] = detail::to_phy_engine_code_and_props(e.data());
            auto idx = add_pe_element(code, std::move(props), pl_id);

            // Default: PL pin numbering matches PE pin numbering for currently mapped 1:1 elements.
            // We only record pins that appear in wires to avoid hardcoding pin counts.
            auto& pm = pin_map[pl_id];
            if (auto it_used = pl_pin_used.find(pl_id); it_used != pl_pin_used.end())
            {
                for (auto const& [pin, used] : it_used->second)
                {
                    if (used)
                    {
                        pm[pin] = endpoint{idx, pin};
                    }
                }
            }
        }

        // Translate PL wires into PE wires via the pin map.
        std::vector<int> wires_flat;
        wires_flat.reserve(ex.wires().size() * 4 + internal_wires_flat.size());
        for (auto const& w : ex.wires())
        {
            auto it_s_el = pin_map.find(w.source.element_identifier);
            auto it_t_el = pin_map.find(w.target.element_identifier);
            if (it_s_el == pin_map.end() || it_t_el == pin_map.end())
            {
                continue;
            }

            auto it_s_pin = it_s_el->second.find(w.source.pin);
            auto it_t_pin = it_t_el->second.find(w.target.pin);
            if (it_s_pin == it_s_el->second.end() || it_t_pin == it_t_el->second.end())
            {
                throw std::runtime_error("wire references unmapped pin");
            }

            wires_flat.push_back(static_cast<int>(it_s_pin->second.element_index));
            wires_flat.push_back(it_s_pin->second.pin);
            wires_flat.push_back(static_cast<int>(it_t_pin->second.element_index));
            wires_flat.push_back(it_t_pin->second.pin);
        }

        wires_flat.insert(wires_flat.end(), internal_wires_flat.begin(), internal_wires_flat.end());

        // Ensure non-null pointers for create_circuit().
        std::vector<double> prop_buf = properties;
        if (prop_buf.empty())
        {
            prop_buf.push_back(0.0);
        }

        auto* ele_ptr = element_codes.data();
        auto* wire_ptr = wires_flat.empty() ? nullptr : wires_flat.data();
        auto* prop_ptr = prop_buf.data();

        std::size_t* vec_pos{};
        std::size_t* chunk_pos{};
        std::size_t comp_size{};
        void* cptr = create_circuit(ele_ptr,
                                    element_codes.size(),
                                    wire_ptr,
                                    wires_flat.size(),
                                    prop_ptr,
                                    &vec_pos,
                                    &chunk_pos,
                                    &comp_size);
        if (cptr == nullptr)
        {
            throw std::runtime_error("create_circuit failed");
        }

        circuit_ptr_ = cptr;
        vec_pos_ = vec_pos;
        chunk_pos_ = chunk_pos;
        comp_size_ = comp_size;
        comp_element_ids_ = std::move(comp_element_ids);
        comp_codes_ = std::move(comp_codes);

        if (comp_element_ids_.size() != comp_size_ || comp_codes_.size() != comp_size_)
        {
            throw std::runtime_error("internal error: component mapping size mismatch");
        }
    }

private:
    void* circuit_ptr_{};
    std::size_t* vec_pos_{};
    std::size_t* chunk_pos_{};
    std::size_t comp_size_{};

    std::vector<std::string> comp_element_ids_;
    std::vector<int> comp_codes_;
};

}  // namespace phy_engine::phy_lab_wrapper::pe
