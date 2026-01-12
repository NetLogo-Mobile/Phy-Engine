# Verilog → PE Netlist 综合测试（`pe_synth_*`）

本目录用于测试 `include/phy_engine/verilog/digital/pe_synth.h` 的“Verilog 子集 → PE 数字网表”综合能力（不走 DLL 入口，直接 `#include <phy_engine/phy_engine.h>`）。

## 未实现/限制（综合层面，`pe_synth`）

- `stmt_node::for_stmt` / `stmt_node::while_stmt`：支持“动态条件”的循环，但采用**有限次展开**（由 `pe_synth_options::loop_unroll_limit` 控制，默认 64）；超过展开上限的迭代不会被建模，电路规模也会随上限快速膨胀。
- `#delay`（`stmt_node::delay_ticks != 0`）：已支持，语义为“以 `digital_clk()` 为单位的 tick-based transport delay”（实现为 `TICK_DELAY` 1-bit 延迟线）；支持 `#N` 与 `#(const_expr)`（见 `pe_synth_delay.cpp`、`pe_synth_delay_constexpr.cpp`）。
- `always_ff` 事件表达式：已放宽为“至少 1 个事件”；支持 `level` reset 事件与 >2 events（取第一个 edge event 作为 clk），复位条件支持更复杂的布尔表达式（见 `pe_synth_async_reset_expr_multi_event.cpp`、`pe_synth_level_event_reset.cpp`）。
- `always_comb`：支持自动推 latch（使用 `DLATCH` 原件建模），不再因为锁存器推断而直接报错（见 `pe_synth_latch_infer.cpp`）。
- 阻塞/非阻塞语义细节：同一 `always` 内的阻塞赋值顺序会被用于表达式替换/建图（见 `pe_synth_blocking_sequence.cpp`）；更复杂的 Verilog 调度语义仍未完整覆盖。

## 未实现/限制（Verilog 子集本身）

该 Verilog 子集是“刻意缩小的可综合子集”。当前表达式/语句覆盖不等价于完整 Verilog（例如更复杂的运算符、系统函数、时序控制等并非全部支持）。

## 与“未来 x86 → 逻辑电路”相关的缺口（方向性）

- 更完整的语句级综合（循环综合/展开、更多控制结构）。
- 更完整的可综合算术/位运算/比较运算支持，以及对应的 PE 原件映射（ALU、移位、加减乘除/模等）。
- 更完整的时序建模与优化（寄存器推断策略、扇出/层级优化、更稳定的传播与收敛策略）。
