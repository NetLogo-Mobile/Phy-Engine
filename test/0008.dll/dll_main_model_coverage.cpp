#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

extern "C" void* create_circuit(int* elements,
                                ::std::size_t ele_size,
                                int* wires,
                                ::std::size_t wires_size,
                                double* properties,
                                ::std::size_t** vec_pos,
                                ::std::size_t** chunk_pos,
                                ::std::size_t* comp_size);

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos);

// Compile the DLL entrypoints into this test binary.
#include "../../src/dll_main.cpp"

namespace
{
    inline bool near(double a, double b, double eps = 1e-12) noexcept { return ::std::fabs(a - b) <= eps; }

    inline double get_attr_d(::phy_engine::circult& c, ::std::size_t vec_pos, ::std::size_t chunk_pos, ::std::size_t attr_idx)
    {
        auto& nl = c.get_netlist();
        auto* m = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos, chunk_pos});
        if(m == nullptr || m->ptr == nullptr) { return ::std::numeric_limits<double>::quiet_NaN(); }
        auto const v = m->ptr->get_attribute(attr_idx);
        if(v.type != ::phy_engine::model::variant_type::d) { return ::std::numeric_limits<double>::quiet_NaN(); }
        return v.d;
    }
}  // namespace

int main()
{
    // Keep this list synchronized with the element-code docs in `include/phy_engine/dll_api.h`.
    int elements[] = {
        16,  // transformer_center_tap
        17,  // op_amp
        18,  // relay
        19,  // comparator
        20,  // sawtooth_gen
        21,  // square_gen
        22,  // pulse_gen
        23,  // triangle_gen
        50,  // BJT_NPN
        51,  // BJT_PNP
        52,  // nmosfet
        53,  // pmosfet
        54,  // full_bridge_rectifier

        203, 204, 205, 206, 207, 208, 209, 210, 211, 212,  // digital logical
        220, 221, 222, 223, 224, 225, 226, 227, 228,       // digital combinational
    };

    // Property stream (positional), consumed only by the analog/non-linear models above.
    // Total = 42 doubles.
    double props[] = {
        // 16: transformer_center_tap.n_total
        10.0,
        // 17: op_amp.mu
        1.0e6,
        // 18: relay.Von, relay.Voff
        5.0,
        3.0,
        // 19: comparator.Ll, comparator.Hl
        0.2,
        0.8,
        // 20: sawtooth_gen: Vh, Vl, freq, phase
        5.0,
        0.0,
        1000.0,
        0.1,
        // 21: square_gen: Vh, Vl, freq, duty, phase
        3.3,
        0.0,
        2000.0,
        0.25,
        0.0,
        // 22: pulse_gen: Vh, Vl, freq, duty, phase, tr, tf
        5.0,
        0.0,
        100.0,
        0.1,
        0.0,
        1e-6,
        2e-6,
        // 23: triangle_gen: Vh, Vl, freq, phase
        1.0,
        -1.0,
        50.0,
        0.2,
        // 50: BJT_NPN: Is, N, BetaF, Temp, Area
        1e-15,
        1.2,
        80.0,
        27.0,
        2.0,
        // 51: BJT_PNP: Is, N, BetaF, Temp, Area
        1e-14,
        1.1,
        50.0,
        30.0,
        1.5,
        // 52: nmosfet: Kp, lambda, Vth
        2e-3,
        0.02,
        0.7,
        // 53: pmosfet: Kp, lambda, Vth
        1e-3,
        0.01,
        0.9,
    };

    ::std::size_t* vec_pos{};
    ::std::size_t* chunk_pos{};
    ::std::size_t comp_size{};

    void* cptr = create_circuit(elements,
                                sizeof(elements) / sizeof(elements[0]),
                                nullptr,
                                0,
                                props,
                                &vec_pos,
                                &chunk_pos,
                                &comp_size);
    if(cptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr) { return 1; }
    if(comp_size != (sizeof(elements) / sizeof(elements[0])))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 2;
    }

    auto& c = *static_cast<::phy_engine::circult*>(cptr);

    // Spot-check attribute wiring for the property-consuming models.
    if(!near(get_attr_d(c, vec_pos[0], chunk_pos[0], 0), 10.0)) { destroy_circuit(cptr, vec_pos, chunk_pos); return 3; }
    if(!near(get_attr_d(c, vec_pos[1], chunk_pos[1], 0), 1.0e6)) { destroy_circuit(cptr, vec_pos, chunk_pos); return 4; }
    if(!near(get_attr_d(c, vec_pos[2], chunk_pos[2], 0), 5.0) || !near(get_attr_d(c, vec_pos[2], chunk_pos[2], 1), 3.0))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 5;
    }
    if(!near(get_attr_d(c, vec_pos[3], chunk_pos[3], 0), 0.2) || !near(get_attr_d(c, vec_pos[3], chunk_pos[3], 1), 0.8))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 6;
    }
    if(!near(get_attr_d(c, vec_pos[5], chunk_pos[5], 0), 3.3) || !near(get_attr_d(c, vec_pos[5], chunk_pos[5], 3), 0.25))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 7;
    }
    if(!near(get_attr_d(c, vec_pos[8], chunk_pos[8], 2), 80.0) || !near(get_attr_d(c, vec_pos[9], chunk_pos[9], 4), 1.5))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 8;
    }
    if(!near(get_attr_d(c, vec_pos[10], chunk_pos[10], 2), 0.7) || !near(get_attr_d(c, vec_pos[11], chunk_pos[11], 1), 0.01))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 9;
    }

    // Ensure all models were created (including digital blocks with no property consumption).
    {
        auto& nl = c.get_netlist();
        for(::std::size_t i{}; i < comp_size; ++i)
        {
            auto* m = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
            if(m == nullptr || m->ptr == nullptr)
            {
                destroy_circuit(cptr, vec_pos, chunk_pos);
                return 10;
            }
        }
    }

    destroy_circuit(cptr, vec_pos, chunk_pos);
    return 0;
}
