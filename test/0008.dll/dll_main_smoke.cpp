#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

extern "C" void* create_circuit(int* elements,
                                ::std::size_t ele_size,
                                int* wires,
                                ::std::size_t wires_size,
                                double* properties,
                                ::std::size_t** vec_pos,
                                ::std::size_t** chunk_pos,
                                ::std::size_t* comp_size);

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos);

extern "C" int analyze_circuit(void* circuit_ptr,
                               ::std::size_t* vec_pos,
                               ::std::size_t* chunk_pos,
                               ::std::size_t comp_size,
                               int* changed_ele,
                               ::std::size_t* changed_ind,
                               double* changed_prop,
                               ::std::size_t prop_size,
                               double* voltage,
                               ::std::size_t* voltage_ord,
                               double* current,
                               ::std::size_t* current_ord,
                               bool* digital,
                               ::std::size_t* digital_ord);

// Compile the DLL entrypoints into this test binary.
#include "../../src/dll_main.cpp"

int main()
{
    // elements: 0=ground, 4=VDC, 1=R
    int elements[] = {0, 4, 1};

    // wires are in quads: (ele1,pin1,ele2,pin2); wires_size is number of ints (must be multiple of 4).
    // Connect: VDC+ -> R.A, R.B -> GND, VDC- -> GND.
    int wires[] = {
        1, 0, 2, 0,
        2, 1, 0, 0,
        1, 1, 0, 0,
    };

    // properties consumed in element order excluding ground: VDC.V, R.r
    double properties[] = {5.0, 1000.0};

    ::std::size_t* vec_pos{};
    ::std::size_t* chunk_pos{};
    ::std::size_t comp_size{};

    void* cptr = create_circuit(elements,
                                sizeof(elements) / sizeof(elements[0]),
                                wires,
                                sizeof(wires) / sizeof(wires[0]),
                                properties,
                                &vec_pos,
                                &chunk_pos,
                                &comp_size);
    if(cptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr) { return 1; }
    if(comp_size != 2) { destroy_circuit(cptr, vec_pos, chunk_pos); return 1; }

    // Force DC analysis for deterministic voltage results.
    auto* c = static_cast<::phy_engine::circult*>(cptr);
    c->set_analyze_type(::phy_engine::analyze_type::DC);

    double voltage[16]{};
    ::std::size_t voltage_ord[3]{};
    double current[16]{};
    ::std::size_t current_ord[3]{};
    bool digital[16]{};
    ::std::size_t digital_ord[3]{};

    int const rc = analyze_circuit(cptr,
                                   vec_pos,
                                   chunk_pos,
                                   comp_size,
                                   nullptr,
                                   nullptr,
                                   nullptr,
                                   0,
                                   voltage,
                                   voltage_ord,
                                   current,
                                   current_ord,
                                   digital,
                                   digital_ord);
    if(rc != 0) { destroy_circuit(cptr, vec_pos, chunk_pos); return 1; }

    if(voltage_ord[0] != 0 || voltage_ord[1] != 2 || voltage_ord[2] != 4)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 1;
    }

    auto near = [&](double a, double b) noexcept { return ::std::fabs(a - b) <= 1e-6; };

    // Order: first non-ground element is VDC, then R.
    if(!near(voltage[0], 5.0) || !near(voltage[1], 0.0) || !near(voltage[2], 5.0) || !near(voltage[3], 0.0))
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 1;
    }

    destroy_circuit(cptr, vec_pos, chunk_pos);
    return 0;
}

