#include <cmath>
#include <cstddef>
#include <cstring>
#include <limits>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/non-linear/bsim3v32.h>
#include <phy_engine/netlist/impl.h>

static std::size_t find_attr_exact(::phy_engine::model::model_base* m, char const* name) noexcept
{
    if(m == nullptr || name == nullptr) { return SIZE_MAX; }
    std::size_t const want_len{std::strlen(name)};

    constexpr std::size_t kMaxScan{512};
    for(std::size_t idx{}; idx < kMaxScan; ++idx)
    {
        auto const n = m->ptr->get_attribute_name(idx);
        if(n.empty() || n.size() != want_len) { continue; }
        auto const* p = reinterpret_cast<char const*>(n.data());
        if(std::memcmp(p, name, want_len) == 0) { return idx; }
    }
    return SIZE_MAX;
}

static bool set_d(::phy_engine::model::model_base* m, std::size_t idx, double v) noexcept
{
    if(m == nullptr || idx == SIZE_MAX) { return false; }
    return m->ptr->set_attribute(idx, {.d{v}, .type{::phy_engine::model::variant_type::d}});
}

static double get_d(::phy_engine::model::model_base* m, std::size_t idx) noexcept
{
    if(m == nullptr || idx == SIZE_MAX) { return std::numeric_limits<double>::quiet_NaN(); }
    auto const v = m->ptr->get_attribute(idx);
    if(v.type != ::phy_engine::model::variant_type::d) { return std::numeric_limits<double>::quiet_NaN(); }
    return v.d;
}

int main()
{
    // No simulation needed; just validate exact alias names exist and map to the same storage.
    ::phy_engine::circult c{};
    auto& nl = c.get_netlist();
    auto [m1, m1_pos] = add_model(nl, ::phy_engine::model::bsim3v32_nmos{});

    struct alias_pair
    {
        char const* canonical;
        char const* alias;
        double value;
    };

    alias_pair const pairs[] = {
        {"W", "w", 2e-6},
        {"L", "l", 1.5e-6},
        {"capMod", "capmod", 1.0},
        {"Kp", "kp", 123e-6},
        {"Vth0", "vth0", 0.55},
        {"Temp", "temp", 77.0},
        {"Rd", "rd", 12.0},
        {"Rs", "rs", 34.0},
        {"Rb", "rb", 56.0},
        {"Cgs", "cgs", 1e-12},
        {"Cgd", "cgd", 2e-12},
        {"Cgb", "cgb", 3e-12},
    };

    for(std::size_t i{}; i < (sizeof(pairs) / sizeof(pairs[0])); ++i)
    {
        auto const idx_c = find_attr_exact(m1, pairs[i].canonical);
        auto const idx_a = find_attr_exact(m1, pairs[i].alias);
        if(idx_c == SIZE_MAX || idx_a == SIZE_MAX) { return static_cast<int>(10 + i); }

        if(!set_d(m1, idx_a, pairs[i].value)) { return static_cast<int>(40 + i); }
        double const got = get_d(m1, idx_c);
        if(!std::isfinite(got)) { return static_cast<int>(70 + i); }
        if(!(std::abs(got - pairs[i].value) <= std::abs(pairs[i].value) * 1e-12 + 1e-30)) { return static_cast<int>(100 + i); }
    }

    return 0;
}
