/*************
* Phy Engine *
*************/
#include "../phy_engine_utils/fast_io/fast_io.h"
int main() {
    {
        auto se{::fast_io::u8err()};
        ::fast_io::io::println(se, u8"Phy Engine");
    }
    return 0;
}