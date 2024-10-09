#include <phy_engine/verilog/parser/parser.h>

int main()
{
    decltype(auto) str{
        u8R"(q                                                                                                                     
1 `define
int a
                                                                                                 
)"};

    ::phy_engine::verilog::parser_file(str, str + sizeof(str));
}
