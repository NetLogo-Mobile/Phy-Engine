#include "../../src/dll_main.cpp"
#include <iostream>

int main() {
    void* c;
    // 仿照capacitor_tr

    int elements[] = {0, 1, 2, 4}; // GND, R, C, VDC
    int wires[] = {1, 1, 2, 0, 2, 1, 0, 0, 3, 1, 0, 0, 1, 0, 3, 0};
    double properties[] = {10.0, 1e-5, 3.0};
    std::size_t *vec_pos, *chunk_pos, comp_size; // comp_size将存储去除接地元件后的ele_size（即pos的size）
    c = create_circuit(elements, 4, wires, 4, properties, &vec_pos, &chunk_pos, &comp_size);

    double voltage[6], current[3];
    bool digital[1];
    std::size_t voltage_ord[3], current_ord[3], digital_ord[3];
    for (int i = 0; i < 10; ++i) {
        ::fast_io::io::print("========================\n" "round = ", i , "\n========================\n");
        analyze_circuit(c, vec_pos, chunk_pos, comp_size, NULL, NULL, NULL, 0, voltage, voltage_ord, current, current_ord, digital, digital_ord);
        ::fast_io::io::print("C1: U=", voltage[1], "\n");
    }

    destroy_circuit(c, vec_pos, chunk_pos);
}