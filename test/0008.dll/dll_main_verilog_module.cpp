#include <cstddef>
#include <cstdint>

extern "C" int circuit_set_analyze_type(void* circuit_ptr, ::std::uint32_t analyze_type_value);
extern "C" int circuit_set_tr(void* circuit_ptr, double t_step, double t_stop);
extern "C" int circuit_analyze(void* circuit_ptr);
extern "C" int circuit_digital_clk(void* circuit_ptr);
extern "C" int circuit_sample(void* circuit_ptr,
                              ::std::size_t* vec_pos,
                              ::std::size_t* chunk_pos,
                              ::std::size_t comp_size,
                              double* voltage,
                              ::std::size_t* voltage_ord,
                              double* current,
                              ::std::size_t* current_ord,
                              bool* digital,
                              ::std::size_t* digital_ord);
extern "C" int circuit_set_model_digital(void* circuit_ptr,
                                         ::std::size_t vec_pos,
                                         ::std::size_t chunk_pos,
                                         ::std::size_t attribute_index,
                                         ::std::uint8_t state);

extern "C" void* create_circuit_ex(int* elements,
                                   ::std::size_t ele_size,
                                   int* wires,
                                   ::std::size_t wires_size,
                                   double* properties,
                                   char const* const* texts,
                                   ::std::size_t const* text_sizes,
                                   ::std::size_t text_count,
                                   ::std::size_t const* element_src_index,
                                   ::std::size_t const* element_top_index,
                                   ::std::size_t** vec_pos,
                                   ::std::size_t** chunk_pos,
                                   ::std::size_t* comp_size);

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos);

// Compile the DLL entrypoints into this test binary.
#include "../../src/dll_main.cpp"

namespace
{
    inline constexpr int PE_INPUT = 200;
    inline constexpr int PE_OUTPUT = 201;
    inline constexpr int PE_VERILOG = 300;
}  // namespace

int main()
{
    // Circuit:
    //   INPUT a, INPUT b -> verilog module and2 -> OUTPUT y
    int elements[] = {PE_INPUT, PE_INPUT, PE_VERILOG, PE_OUTPUT};

    int wires[] = {
        0, 0, 2, 0,  // a.o -> vmod.a
        1, 0, 2, 1,  // b.o -> vmod.b
        2, 2, 3, 0,  // vmod.y -> out.i
    };

    // properties: INPUT state for a, INPUT state for b
    double props[] = {0.0, 0.0};

    decltype(auto) src = u8R"(
module top(input a, input b, output y);
  assign y = a & b;
endmodule
)";
    constexpr char top_name[] = "top";

    char const* texts[] = {
        reinterpret_cast<char const*>(src),
        top_name,
    };
    ::std::size_t text_sizes[] = {
        sizeof(src) - 1,
        sizeof(top_name) - 1,
    };

    ::std::size_t src_idx[] = {SIZE_MAX, SIZE_MAX, 0, SIZE_MAX};
    ::std::size_t top_idx[] = {SIZE_MAX, SIZE_MAX, 1, SIZE_MAX};

    ::std::size_t* vec_pos{};
    ::std::size_t* chunk_pos{};
    ::std::size_t comp_size{};

    void* cptr = create_circuit_ex(elements,
                                   sizeof(elements) / sizeof(elements[0]),
                                   wires,
                                   sizeof(wires) / sizeof(wires[0]),
                                   props,
                                   texts,
                                   text_sizes,
                                   sizeof(texts) / sizeof(texts[0]),
                                   src_idx,
                                   top_idx,
                                   &vec_pos,
                                   &chunk_pos,
                                   &comp_size);
    if(cptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr) { return 1; }
    if(comp_size != 4) { destroy_circuit(cptr, vec_pos, chunk_pos); return 2; }

    if(circuit_set_analyze_type(cptr, static_cast<::std::uint32_t>(::phy_engine::analyze_type::TR)) != 0)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 3;
    }
    if(circuit_set_tr(cptr, 1e-9, 1e-9) != 0)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 4;
    }

    // Prepare internal tables.
    if(circuit_analyze(cptr) != 0)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 5;
    }

    auto sample_y = [&]() -> bool
    {
        double voltage[64]{};
        ::std::size_t voltage_ord[8]{};
        double current[64]{};
        ::std::size_t current_ord[8]{};
        bool digital[64]{};
        ::std::size_t digital_ord[8]{};

        if(circuit_sample(cptr, vec_pos, chunk_pos, comp_size, voltage, voltage_ord, current, current_ord, digital, digital_ord) != 0) { return false; }

        // OUTPUT is component index 3 and has 1 pin => digital at digital_ord[3].
        return digital[digital_ord[3]];
    };

    auto set_in = [&](::std::size_t comp_index, ::std::uint8_t st) -> bool
    {
        if(comp_index >= comp_size) { return false; }
        return circuit_set_model_digital(cptr, vec_pos[comp_index], chunk_pos[comp_index], 0, st) == 0;
    };

    // a=0,b=0 => y=0
    if(!set_in(0, 0) || !set_in(1, 0)) { destroy_circuit(cptr, vec_pos, chunk_pos); return 6; }
    if(circuit_digital_clk(cptr) != 0) { destroy_circuit(cptr, vec_pos, chunk_pos); return 7; }
    if(sample_y()) { destroy_circuit(cptr, vec_pos, chunk_pos); return 8; }

    // a=1,b=0 => y=0
    if(!set_in(0, 1) || !set_in(1, 0)) { destroy_circuit(cptr, vec_pos, chunk_pos); return 9; }
    if(circuit_digital_clk(cptr) != 0) { destroy_circuit(cptr, vec_pos, chunk_pos); return 10; }
    if(sample_y()) { destroy_circuit(cptr, vec_pos, chunk_pos); return 11; }

    // a=1,b=1 => y=1
    if(!set_in(0, 1) || !set_in(1, 1)) { destroy_circuit(cptr, vec_pos, chunk_pos); return 12; }
    if(circuit_digital_clk(cptr) != 0) { destroy_circuit(cptr, vec_pos, chunk_pos); return 13; }
    if(!sample_y()) { destroy_circuit(cptr, vec_pos, chunk_pos); return 14; }

    destroy_circuit(cptr, vec_pos, chunk_pos);
    return 0;
}

