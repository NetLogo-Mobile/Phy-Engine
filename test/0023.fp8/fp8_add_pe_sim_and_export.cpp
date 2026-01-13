#include "fp8_common.h"

int main()
{
    return fp8_test::run_pe_sim_and_export(fp8_test::op_kind::add, "fp8_add.v", u8"fp8_add_top", "fp8_add_pe_to_pl.sav");
}

