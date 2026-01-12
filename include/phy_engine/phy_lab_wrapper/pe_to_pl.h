#pragma once

#include "physicslab.h"
#include "pe_model_id.h"

#include <phy_engine/model/model_refs/base.h>
#include <phy_engine/netlist/netlist.h>

#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace phy_engine::phy_lab_wrapper::pe_to_pl
{
struct pl_endpoint
{
    std::string element_identifier{};
    int pin{};
};

struct net
{
    ::phy_engine::model::node_t const* pe_node{};
    std::vector<pl_endpoint> endpoints{};
};

struct options
{
    // All elements are placed at this fixed position for now (router will reposition later).
    position fixed_pos{0.0, 0.0, 0.0};

    // If true, use element-xyz coordinates in the generated .sav.
    bool element_xyz_coords{false};

    // Keep higher-level PL macro elements when possible (e.g., COUNTER4 -> "Counter").
    bool keep_pl_macros{true};

    // If true, generate PL wires for each PE net (node) from collected endpoints.
    bool generate_wires{true};

    // If true, include unknown/unsupported PE digital models as placeholders.
    // Default is false because PE->PL is expected to be explicit about what is supported.
    bool keep_unknown_as_placeholders{false};
};

struct result
{
    experiment ex{};
    std::vector<net> nets{};
    std::vector<std::string> warnings{};
};

namespace detail
{
inline std::string u8sv_to_string(::fast_io::u8string_view v)
{
    return std::string(reinterpret_cast<char const*>(v.data()), v.size());
}

struct pl_model_mapping
{
    std::string model_id{};
    bool is_big_element{};

    // Map from PE pin index -> PL pin index; missing entries are ignored.
    std::unordered_map<std::size_t, int> pe_to_pl_pin{};
};

inline pl_model_mapping identity_mapping(std::string mid, std::size_t pin_count, bool big = false)
{
    pl_model_mapping m{};
    m.model_id = std::move(mid);
    m.is_big_element = big;
    m.pe_to_pl_pin.reserve(pin_count);
    for(std::size_t i{}; i < pin_count; ++i) { m.pe_to_pl_pin.emplace(i, static_cast<int>(i)); }
    return m;
}

inline pl_model_mapping map_pe_model_to_pl(::phy_engine::model::model_base const& mb, options const& opt, std::vector<std::string>& warnings)
{
    auto const name_u8 = (mb.ptr == nullptr) ? ::fast_io::u8string_view{} : mb.ptr->get_model_name();
    auto const name = u8sv_to_string(name_u8);

    // Digital I/O
    if(name == "INPUT") { return identity_mapping(std::string(pl_model_id::logic_input), 1); }
    if(name == "OUTPUT") { return identity_mapping(std::string(pl_model_id::logic_output), 1); }

    // Basic gates
    if(name == "YES") { return identity_mapping(std::string(pl_model_id::yes_gate), 2); }
    if(name == "NOT") { return identity_mapping(std::string(pl_model_id::no_gate), 2); }
    if(name == "AND") { return identity_mapping(std::string(pl_model_id::and_gate), 3); }
    if(name == "OR") { return identity_mapping(std::string(pl_model_id::or_gate), 3); }
    if(name == "XOR") { return identity_mapping(std::string(pl_model_id::xor_gate), 3); }
    if(name == "XNOR") { return identity_mapping(std::string(pl_model_id::xnor_gate), 3); }
    if(name == "NAND") { return identity_mapping(std::string(pl_model_id::nand_gate), 3); }
    if(name == "NOR") { return identity_mapping(std::string(pl_model_id::nor_gate), 3); }
    if(name == "IMP") { return identity_mapping(std::string(pl_model_id::imp_gate), 3); }
    if(name == "NIMP") { return identity_mapping(std::string(pl_model_id::nimp_gate), 3); }

    // Arithmetic blocks
    if(name == "HALF_ADDER") { return identity_mapping(std::string(pl_model_id::half_adder), 4); }
    if(name == "FULL_ADDER") { return identity_mapping(std::string(pl_model_id::full_adder), 5); }
    if(name == "HALF_SUB") { return identity_mapping(std::string(pl_model_id::half_subtractor), 4); }
    if(name == "FULL_SUB") { return identity_mapping(std::string(pl_model_id::full_subtractor), 5); }
    if(name == "MUL2") { return identity_mapping(std::string(pl_model_id::multiplier), 8); }

    // Sequential blocks
    if(name == "DFF") { return identity_mapping(std::string(pl_model_id::d_flipflop), 3); }
    if(name == "TFF") { return identity_mapping(std::string(pl_model_id::t_flipflop), 3); }
    if(name == "T_BAR_FF") { return identity_mapping(std::string(pl_model_id::real_t_flipflop), 3); }
    if(name == "JKFF") { return identity_mapping(std::string(pl_model_id::jk_flipflop), 4); }

    // Macros / larger blocks
    if(opt.keep_pl_macros)
    {
        if(name == "COUNTER4") { return identity_mapping(std::string(pl_model_id::counter), 6, true); }
        if(name == "RANDOM_GENERATOR4") { return identity_mapping(std::string(pl_model_id::random_generator), 6, true); }
    }

    // Bus I/O macros (no smaller PL primitives available today).
    if(name == "EIGHT_BIT_INPUT") { return identity_mapping(std::string(pl_model_id::eight_bit_input), 8, true); }
    if(name == "EIGHT_BIT_DISPLAY") { return identity_mapping(std::string(pl_model_id::eight_bit_display), 8, true); }

    // Schmitt trigger (if present in PL, keep it; otherwise it can still be used for layout-only export).
    if(name == "SCHMITT_TRIGGER") { return identity_mapping(std::string(pl_model_id::schmitt_trigger), 2); }

    // PE-only digital primitives -> best-effort degradation for layout export.
    if(name == "RESOLVE2")
    {
        warnings.push_back("pe_to_pl: degrading RESOLVE2 -> Or Gate (drops Z/X resolution semantics)");
        return identity_mapping(std::string(pl_model_id::or_gate), 3);
    }
    if(name == "CASE_EQ")
    {
        warnings.push_back("pe_to_pl: degrading CASE_EQ -> Xnor Gate (drops X/Z-aware === semantics)");
        return identity_mapping(std::string(pl_model_id::xnor_gate), 3);
    }
    if(name == "IS_UNKNOWN")
    {
        warnings.push_back("pe_to_pl: degrading IS_UNKNOWN -> Yes Gate (drops X/Z detection semantics)");
        return identity_mapping(std::string(pl_model_id::yes_gate), 2);
    }
    if(name == "TRI")
    {
        warnings.push_back("pe_to_pl: degrading TRI -> Yes Gate (drops enable/Z semantics)");
        pl_model_mapping m{};
        m.model_id = std::string(pl_model_id::yes_gate);
        m.pe_to_pl_pin.emplace(0, 0);  // i
        m.pe_to_pl_pin.emplace(2, 1);  // o
        return m;
    }
    if(name == "DLATCH")
    {
        warnings.push_back("pe_to_pl: degrading DLATCH -> D Flipflop (treats en as clk)");
        return identity_mapping(std::string(pl_model_id::d_flipflop), 3);
    }
    if(name == "DFF_ARSTN")
    {
        warnings.push_back("pe_to_pl: degrading DFF_ARSTN -> D Flipflop (drops async reset pin)");
        pl_model_mapping m{};
        m.model_id = std::string(pl_model_id::d_flipflop);
        m.pe_to_pl_pin.emplace(0, 0);  // d
        m.pe_to_pl_pin.emplace(1, 1);  // clk
        m.pe_to_pl_pin.emplace(3, 2);  // q
        return m;
    }
    if(name == "TICK_DELAY")
    {
        warnings.push_back("pe_to_pl: degrading TICK_DELAY -> Yes Gate (drops #delay/tick semantics)");
        return identity_mapping(std::string(pl_model_id::yes_gate), 2);
    }

    // Explicitly excluded/high-complexity models (not exported as PL circuits).
    if(name == "VERILOG_MODULE" || name == "VERILOG_PORTS")
    {
        throw std::runtime_error("pe_to_pl: unsupported conversion (excluded model): " + name);
    }

    if(opt.keep_unknown_as_placeholders)
    {
        // Generic placeholder (keeps at least a node with 2 pins for routing experiments).
        warnings.push_back("pe_to_pl: unknown PE digital model '" + name + "' -> placeholder Yes Gate");
        return identity_mapping(std::string(pl_model_id::yes_gate), 2);
    }

    return {};
}

inline void try_set_pl_properties_for_element(experiment& ex,
                                              std::string const& element_id,
                                              ::phy_engine::model::model_base const& mb,
                                              std::vector<std::string>& warnings)
{
    if(mb.ptr == nullptr) { return; }
    auto const name = u8sv_to_string(mb.ptr->get_model_name());

    if(name == "INPUT")
    {
        auto v = mb.ptr->get_attribute(0);
        if(v.type != ::phy_engine::model::variant_type::digital) { return; }
        int sw{};
        if(v.digital == ::phy_engine::model::digital_node_statement_t::true_state) { sw = 1; }
        else if(v.digital == ::phy_engine::model::digital_node_statement_t::false_state) { sw = 0; }
        else
        {
            warnings.push_back("pe_to_pl: Logic Input initial state is X/Z; defaulting to 0");
            sw = 0;
        }
        ex.get_element(element_id).data()["Properties"]["开关"] = sw;
    }
}
}  // namespace detail

inline result convert(::phy_engine::netlist::netlist const& nl, options const& opt = {})
{
    result out;
    out.ex = experiment::create(experiment_type::circuit);

    // PE model -> PL element id + per-pin mapping.
    struct mapped_element
    {
        std::string element_id{};
        std::unordered_map<std::size_t, int> pe_to_pl_pin{};
    };
    std::unordered_map<::phy_engine::model::model_base const*, mapped_element> model_map{};

    // 1) Create PL elements.
    for(auto const& mb: nl.models)
    {
        for(auto const* m = mb.begin; m != mb.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal) { continue; }
            if(m->ptr == nullptr) { continue; }
            if(m->ptr->get_device_type() != ::phy_engine::model::model_device_type::digital) { continue; }

            auto mapping = detail::map_pe_model_to_pl(*m, opt, out.warnings);
            if(mapping.model_id.empty())
            {
                auto const name_u8 = m->ptr->get_model_name();
                auto const name = detail::u8sv_to_string(name_u8);
                throw std::runtime_error("pe_to_pl: unsupported conversion for PE digital model: " + name);
            }

            auto id = out.ex.add_circuit_element(mapping.model_id, opt.fixed_pos, opt.element_xyz_coords, mapping.is_big_element);
            model_map.emplace(m, mapped_element{std::move(id), std::move(mapping.pe_to_pl_pin)});

            detail::try_set_pl_properties_for_element(out.ex, model_map[m].element_id, *m, out.warnings);
        }
    }

    // 2) Collect nets (PE node -> list of PL endpoints).
    std::unordered_map<::phy_engine::model::node_t const*, std::vector<pl_endpoint>> node_eps{};

    for(auto const& mb: nl.models)
    {
        for(auto const* m = mb.begin; m != mb.curr; ++m)
        {
            if(m->type != ::phy_engine::model::model_type::normal) { continue; }
            if(m->ptr == nullptr) { continue; }
            if(m->ptr->get_device_type() != ::phy_engine::model::model_device_type::digital) { continue; }

            auto it_m = model_map.find(m);
            if(it_m == model_map.end()) { continue; }

            auto pv = m->ptr->generate_pin_view();
            for(std::size_t pi{}; pi < pv.size; ++pi)
            {
                auto const* node = pv.pins[pi].nodes;
                if(node == nullptr) { continue; }

                auto it_pin = it_m->second.pe_to_pl_pin.find(pi);
                if(it_pin == it_m->second.pe_to_pl_pin.end()) { continue; }

                node_eps[node].push_back(pl_endpoint{it_m->second.element_id, it_pin->second});
            }
        }
    }

    out.nets.reserve(node_eps.size());
    for(auto& kv: node_eps)
    {
        // De-duplicate endpoints (same element/pin can appear multiple times if the PE side re-attaches).
        auto& eps = kv.second;
        std::sort(eps.begin(), eps.end(), [](pl_endpoint const& a, pl_endpoint const& b) {
            if(a.element_identifier != b.element_identifier) { return a.element_identifier < b.element_identifier; }
            return a.pin < b.pin;
        });
        eps.erase(std::unique(eps.begin(), eps.end(), [](pl_endpoint const& a, pl_endpoint const& b) {
                      return a.element_identifier == b.element_identifier && a.pin == b.pin;
                  }),
                  eps.end());

        if(opt.generate_wires && eps.size() >= 2)
        {
            // Create a star topology: connect the first endpoint to all others.
            // This preserves net connectivity without needing geometry-based routing.
            auto const& root = eps.front();
            for(std::size_t i{1}; i < eps.size(); ++i)
            {
                auto const& e = eps[i];
                out.ex.connect(root.element_identifier, root.pin, e.element_identifier, e.pin);
            }
        }

        out.nets.push_back(net{kv.first, std::move(kv.second)});
    }
    return out;
}
}  // namespace phy_engine::phy_lab_wrapper::pe_to_pl
