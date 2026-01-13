#include "fp8_common.h"

int main()
{
    return fp8_test::run_pe_sim_and_export(fp8_test::op_kind::div, "fp8_div.v", u8"fp8_div_top", "fp8_div_pe_to_pl.sav");
}

