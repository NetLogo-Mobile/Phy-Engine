#include <phy_engine/verilog/parser/parser.h>

int main()
{
    decltype(auto) str{
        u8R"(q                                                                                                                     
1 `define     555 
int a
                                                                                                 
)"};
    ::phy_engine::verilog::Verilog_module vmod{};
    ::phy_engine::verilog::parser_file(vmod, str, str + sizeof(str));
}
