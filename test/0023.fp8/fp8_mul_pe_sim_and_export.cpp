#include "fp8_common.h"

int main()
{
    return fp8_test::run_pe_sim_and_export(fp8_test::op_kind::mul, "fp8_mul.v", u8"fp8_mul_top", "fp8_mul_pe_to_pl.sav");
}

