#include <phy_engine/verilog/digital/digital.h>

int main()
{
    using namespace phy_engine::verilog::digital;

    decltype(auto) src = u8R"(
module top(input clk, input d, output reg q);
  always_ff @(posedge clk) begin
    q = d; // blocking assignment is illegal in always_ff
  end
endmodule
)";

    auto cr = compile(::fast_io::u8string_view{src, sizeof(src) - 1});
    return cr.errors.empty() ? 1 : 0;
}

