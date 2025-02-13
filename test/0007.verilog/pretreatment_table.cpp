#include <iostream>
#include <string>
#include <fast_io/fast_io.h>
#include <fast_io/fast_io_dsal/string.h>
#include <phy_engine/verilog/parser/pretreatment/constexpr_hash_table.h>

namespace test
{

    inline constexpr void pretreatment_f(::phy_engine::verilog::Verilog_module& vmod, char8_t const* begin) noexcept {}

    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_1{u8"define", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_2{u8"test", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_3{u8"test2", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_4{u8"test3", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_5{u8"114514", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_6{u8"19198###0", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_7{u8"fff", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_8{u8"ff4", __builtin_addressof(pretreatment_f)};
    constexpr ::phy_engine::verilog::constexpr_hash::pretreatment pretreatment_define_9{u8"ff444455", __builtin_addressof(pretreatment_f)};

    namespace details
    {
        inline constexpr ::phy_engine::verilog::constexpr_hash::pretreatment const* pretreatment_unsort[]  //
            {__builtin_addressof(pretreatment_define_1),
             __builtin_addressof(pretreatment_define_2),
             __builtin_addressof(pretreatment_define_3),
             __builtin_addressof(pretreatment_define_4),
             __builtin_addressof(pretreatment_define_5),
             __builtin_addressof(pretreatment_define_6),
             __builtin_addressof(pretreatment_define_7),
             __builtin_addressof(pretreatment_define_8),
             __builtin_addressof(pretreatment_define_9)};
    };

    inline constexpr auto pretreatments{::phy_engine::verilog::constexpr_hash::pretreatment_sort(details::pretreatment_unsort)};
#if 0
        inline constexpr ::std::size_t max_pretreatments_size{::phy_engine::verilog::constexpr_hash::calculate_max_pretreatment_size(pretreatments)};
#endif
    inline constexpr auto hash_table_size{::phy_engine::verilog::constexpr_hash::calculate_hash_table_size(pretreatments)};
    inline constexpr auto hash_table{
        ::phy_engine::verilog::constexpr_hash::generate_hash_table<hash_table_size.hash_table_size, hash_table_size.extra_size>(pretreatments)};
    [[maybe_unused]] constexpr auto sizeof_hash_table{sizeof(hash_table)};
    inline constexpr auto hash_table_view{::phy_engine::verilog::constexpr_hash::generate_hash_table_view(hash_table)};

}  // namespace test

int main()
{
#if 0
    ::fast_io::io::perrln(test::hash_table_size.hash_table_size, " ", test::hash_table_size.extra_size);
    while(true)
    {
        // fast io cannot scan

        ::std::string a{};
        ::std::cin >> a;
        auto f{::phy_engine::verilog::constexpr_hash::find_from_hash_table_view(
            test::hash_table_view,
            ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(::std::to_address(a.cbegin())), a.size()})};
        ::fast_io::io::perrln(::fast_io::mnp::boolalpha(f != nullptr));
    }
#endif
    return 0;
}
