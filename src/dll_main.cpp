#include <phy_engine/phy_engine.h>

extern "C"
void parse_status_save(char const* status_save)
{
    if (status_save == nullptr) [[unlikely]] {
        return;
    }

    ::phy_engine::circult c{};
}