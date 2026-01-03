#include <phy_engine/phy_engine.h>

static ::std::size_t find_sub(::fast_io::u8string_view hay, ::fast_io::u8string_view needle) noexcept
{
    if(needle.empty()) { return 0; }
    if(needle.size() > hay.size()) { return static_cast<::std::size_t>(-1); }
    for(::std::size_t i{}; i + needle.size() <= hay.size(); ++i)
    {
        bool ok{true};
        for(::std::size_t j{}; j < needle.size(); ++j)
        {
            if(hay[i + j] != needle[j]) { ok = false; break; }
        }
        if(ok) { return i; }
    }
    return static_cast<::std::size_t>(-1);
}

int main()
{
    decltype(auto) src = u8R"(
`define EMPTY

module top(output y);
  assign y = 1 + `EMPTY ;
endmodule
)";

    ::fast_io::u8string_view const sv{src, sizeof(src) - 1};

    ::std::size_t expected_line{};
    ::std::size_t expected_col{};

    {
        ::std::size_t line_no{1};
        char8_t const* p{sv.data()};
        char8_t const* const e{sv.data() + sv.size()};
        while(p < e)
        {
            char8_t const* const lb{p};
            while(p < e && *p != u8'\n') { ++p; }
            char8_t const* le{p};
            if(le > lb && le[-1] == u8'\r') { --le; }
            ::fast_io::u8string_view const lv{lb, static_cast<::std::size_t>(le - lb)};

            auto const needle_pos{find_sub(lv, ::fast_io::u8string_view{u8"assign y = 1 + `EMPTY"})};
            if(needle_pos != static_cast<::std::size_t>(-1))
            {
                for(::std::size_t i{needle_pos}; i < lv.size(); ++i)
                {
                    if(lv[i] == u8';')
                    {
                        expected_line = line_no;
                        expected_col = i + 1;
                        break;
                    }
                }
                break;
            }

            if(p < e && *p == u8'\n') { ++p; }
            ++line_no;
        }
    }

    if(expected_line == 0 || expected_col == 0) { return 1; }

    auto const cr{::phy_engine::verilog::digital::compile(sv)};
    for(auto const& err: cr.errors)
    {
        ::fast_io::u8string_view const msg{err.message.data(), err.message.size()};
        if(msg == ::fast_io::u8string_view{u8"expected expression"} && err.line == expected_line && err.column == expected_col) { return 0; }
    }

    return 1;
}
