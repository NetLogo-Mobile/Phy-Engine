#include "fp8_common.h"

int main()
{
    return fp8_test::run_pe_sim_and_export(fp8_test::op_kind::sub, "fp8_sub.v", u8"fp8_sub_top", "fp8_sub_pe_to_pl.sav");
}

