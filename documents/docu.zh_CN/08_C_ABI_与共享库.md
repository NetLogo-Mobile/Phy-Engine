# 08 C ABI 与共享库（`dll_api.h`）

本章面向“把 PE 当成动态库嵌入”的用户。C ABI 位于 `include/phy_engine/dll_api.h`，其实现（默认）在 `src/dll_main.cpp`。

按函数逐个说明的调用约定与典型流程另见：`documents/docu.zh_CN/11_API_逐函数参考.md`（I 节）。

## 1) 为什么需要 C ABI

- 方便被其它语言/运行时调用（Python/JS/wasm/…）
- 作为 PhysicsLab 适配层 `pe_sim.h` 的底座（PL→PE 仿真）

## 2) 句柄与生命周期

核心约定：

- `create_circuit(...)` / `create_circuit_ex(...)` 返回 `void* circuit_ptr`（内部通常是 `phy_engine::circult*`）
- 释放：`destroy_circuit(circuit_ptr, vec_pos, chunk_pos)`
- `vec_pos/chunk_pos` 数组由 `create_*` 分配并返回给调用者；释放由 `destroy_circuit` 负责（不要自行 free）

## 3) 元件码（`phy_engine_element_code`）

`dll_api.h` 把 PE 的模型库映射成整数 code，并用 `properties` 数组按顺序提供参数。

覆盖范围包括：

- 线性源/无源：R/C/L/VDC/VAC/IDC/IAC
- 受控源：VCCS/VCVS/CCCS/CCVS
- 开关：SPST
- 非线性：PN 结、BJT、MOS、整流桥等
- 数字：INPUT/OUTPUT 与各类门/触发器/计数器/8bit 宏等
- Verilog：`PHY_ENGINE_E_VERILOG_MODULE` / `PHY_ENGINE_E_VERILOG_NETLIST`（仅 `create_circuit_ex`）

具体列表与每个 code 的 property 数量见：`include/phy_engine/dll_api.h`。

## 4) 仿真控制与采样

- `circuit_set_analyze_type(ptr, analyze_type)`：设置 OP/DC/AC/TR…
- `circuit_set_tr(ptr, t_step, t_stop)`：设置 TR
- `circuit_set_ac_omega(ptr, omega)`：设置 AC 单点角频率
- `circuit_analyze(ptr)`：运行一次分析（TR 会在内部循环多个步）
- `circuit_digital_clk(ptr)`：推进数字 tick

采样：

- `circuit_sample(...)`：数字输出为 `bool* digital`（注意 C++ 的 `vector<bool>` 语义，不建议跨语言）
- `circuit_sample_u8(...)`：数字输出为 `uint8_t* digital`（推荐跨语言）

输入驱动：

- `circuit_set_model_digital(ptr, vec_pos, chunk_pos, attribute_index, state)`：设置某个模型的数字属性（常用于数字输入）

一站式：

- `analyze_circuit(...)`：可携带“属性变更列表”，然后 analyze + sample

## 5) 重要实现位置

- 接口声明：`include/phy_engine/dll_api.h`
- 默认实现：`src/dll_main.cpp`
- PL 适配器封装：`include/phy_engine/phy_lab_wrapper/pe_sim.h`
