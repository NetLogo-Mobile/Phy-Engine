#include <cstddef>
#include <cstdint>

extern "C" int circuit_set_analyze_type(void* circuit_ptr, ::std::uint32_t analyze_type_value);
extern "C" int circuit_set_tr(void* circuit_ptr, double t_step, double t_stop);
extern "C" int circuit_analyze(void* circuit_ptr);
extern "C" int circuit_digital_clk(void* circuit_ptr);
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
    inline constexpr int PE_VERILOG_NETLIST = 301;
}  // namespace

int main()
{
    // Circuit:
    //   INPUT clk, INPUT rst_n, INPUT d -> verilog module (synthesized) -> OUTPUT q
    int elements[] = {PE_INPUT, PE_INPUT, PE_INPUT, PE_VERILOG_NETLIST, PE_OUTPUT};

    int wires[] = {
        0, 0, 3, 0,  // clk.o -> vmod.clk
        1, 0, 3, 1,  // rst_n.o -> vmod.rst_n
        2, 0, 3, 2,  // d.o -> vmod.d
        3, 3, 4, 0,  // vmod.q -> out.i
    };

    // properties: INPUT states for clk, rst_n, d
    double props[] = {0.0, 0.0, 0.0};

    decltype(auto) src = u8R"(
module top(input clk, input rst_n, input d, output reg q);
  always @(posedge clk or negedge rst_n) begin
    if(!rst_n) q <= 0;
    else q <= d;
  end
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

    ::std::size_t src_idx[] = {SIZE_MAX, SIZE_MAX, SIZE_MAX, 0, SIZE_MAX};
    ::std::size_t top_idx[] = {SIZE_MAX, SIZE_MAX, SIZE_MAX, 1, SIZE_MAX};

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
    if(comp_size != 5) { destroy_circuit(cptr, vec_pos, chunk_pos); return 2; }

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
    if(circuit_analyze(cptr) != 0)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 5;
    }

    auto set_in = [&](::std::size_t comp_index, ::std::uint8_t st) -> bool
    {
        if(comp_index >= comp_size) { return false; }
        return circuit_set_model_digital(cptr, vec_pos[comp_index], chunk_pos[comp_index], 0, st) == 0;
    };

    auto q_state = [&]() -> ::phy_engine::model::digital_node_statement_t
    {
        auto* c = static_cast<::phy_engine::circult*>(cptr);
        auto* out = ::phy_engine::netlist::get_model(c->get_netlist(), {vec_pos[4], chunk_pos[4]});
        if(out == nullptr || out->ptr == nullptr) { return ::phy_engine::model::digital_node_statement_t::X; }
        auto pv = out->ptr->generate_pin_view();
        if(pv.size == 0 || pv.pins[0].nodes == nullptr) { return ::phy_engine::model::digital_node_statement_t::X; }
        return pv.pins[0].nodes->node_information.dn.state;
    };

    auto clk0 = [&]() -> bool { return set_in(0, 0) && circuit_digital_clk(cptr) == 0; };
    auto clk1 = [&]() -> bool { return set_in(0, 1) && circuit_digital_clk(cptr) == 0; };

    // rst asserted => q resets to 0 without any clock edge.
    if(!set_in(1, 0) || !set_in(2, 1) || !clk0()) { destroy_circuit(cptr, vec_pos, chunk_pos); return 6; }
    if(q_state() != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 7;
    }

    // deassert reset, then rising edge captures d=1 => q=1
    if(!set_in(1, 1) || circuit_digital_clk(cptr) != 0) { destroy_circuit(cptr, vec_pos, chunk_pos); return 8; }
    if(!clk1()) { destroy_circuit(cptr, vec_pos, chunk_pos); return 9; }
    if(q_state() != ::phy_engine::model::digital_node_statement_t::true_state)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 10;
    }

    // async reset again (no clock edge) => q=0
    if(!set_in(1, 0) || circuit_digital_clk(cptr) != 0) { destroy_circuit(cptr, vec_pos, chunk_pos); return 11; }
    if(q_state() != ::phy_engine::model::digital_node_statement_t::false_state)
    {
        destroy_circuit(cptr, vec_pos, chunk_pos);
        return 12;
    }

    destroy_circuit(cptr, vec_pos, chunk_pos);
    return 0;
}

