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
- `pl_experiment_create/load_*` 返回 `void* experiment_ptr`
- 释放：`pl_experiment_destroy(experiment_ptr)`
- `pl_pe_circuit_build(...)` 返回 `void* pe_circuit_ptr`
- 释放：`pl_pe_circuit_destroy(pe_circuit_ptr)`
- 任何“由库分配、调用者接收”的 `char*`（例如 `pl_experiment_dump()`、`pl_experiment_add_circuit_element()`）都必须用 `phy_engine_string_free()` 释放

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

- `circuit_sample_layout(...)`：先查询每个元件的 pin/branch 前缀和布局（Python/FFI 推荐先调这个）
- `circuit_sample(...)`：数字输出为 `bool* digital`（注意 C++ 的 `vector<bool>` 语义，不建议跨语言）
- `circuit_sample_u8(...)`：数字输出为 `uint8_t* digital`（推荐跨语言）
- `circuit_sample_digital_state_u8(...)`：数字输出为 4 态 `uint8_t`（0=L,1=H,2=X,3=Z；Python 包装层默认走这个）

输入驱动：

- `circuit_set_model_digital(ptr, vec_pos, chunk_pos, attribute_index, state)`：设置某个模型的数字属性（常用于数字输入）

一站式：

- `analyze_circuit(...)`：可携带“属性变更列表”，然后 analyze + sample

错误信息：

- `phy_engine_last_error()`：读取最近一次失败 API 的线程局部错误字符串
- `phy_engine_clear_error()`：清空错误字符串

## 5) Verilog 相关 C ABI

共享库现在还额外暴露两组面向绑定层的 Verilog 接口：

- 全局综合默认值（影响 `PHY_ENGINE_E_VERILOG_NETLIST`）：
  - `verilog_synth_set_opt_level(...)`
  - `verilog_synth_set_allow_inout(...)`
  - `verilog_synth_set_allow_multi_driver(...)`
  - `verilog_synth_set_assume_binary_inputs(...)`
  - `verilog_synth_set_optimize_wires(...)`
  - `verilog_synth_set_optimize_mul2(...)`
  - `verilog_synth_set_optimize_adders(...)`
  - `verilog_synth_set_loop_unroll_limit(...)`

- 直接 Verilog runtime（编译/展开/仿真）：
  - `verilog_runtime_create(...)`
  - `verilog_runtime_reset(...)`
  - `verilog_runtime_step(...)`
  - `verilog_runtime_tick(...)`
  - `verilog_runtime_get/set_port_value(...)`
  - `verilog_runtime_get/set_signal_value(...)`
  - `verilog_runtime_*_name_size/copy_*_name(...)`

它们主要服务于 `python/phy_engine/` 这种外部语言包装层。

## 6) PhysicsLab / `.sav` 相关 C ABI

这一组接口把 `phy_lab_wrapper` 也暴露给了外部语言：

- `pl_experiment_create(...)`
- `pl_experiment_load_from_string(...)`
- `pl_experiment_load_from_file(...)`
- `pl_experiment_dump(...)`
- `pl_experiment_save(...)`
- `pl_experiment_add_circuit_element(...)`
- `pl_experiment_connect(...)`
- `pl_experiment_clear_wires(...)`
- `pl_experiment_set_xyz_precision(...)`
- `pl_experiment_set_element_xyz(...)`
- `pl_experiment_set_camera(...)`
- `pl_experiment_set_element_property_number(...)`
- `pl_experiment_set_element_label(...)`
- `pl_experiment_set_element_position(...)`
- `pl_experiment_merge(...)`

适合：

- 直接生成/修改 PhysicsLab `.sav`
- 在 Python/JS 等语言里做批量布线、改属性、改标签、拼接实验
- 把已有 `.sav` 当成脚本可操作的数据模型

## 7) PL→PE 仿真 / PE→PL 导出 / 自动布局

共享库还新增了三组“互操作级”入口：

- PL→PE 仿真：
  - `pl_pe_circuit_build(...)`
  - `pl_pe_circuit_set_analyze_type(...)`
  - `pl_pe_circuit_set_tr(...)`
  - `pl_pe_circuit_set_ac_omega(...)`
  - `pl_pe_circuit_analyze(...)`
  - `pl_pe_circuit_digital_clk(...)`
  - `pl_pe_circuit_sync_inputs_from_pl(...)`
  - `pl_pe_circuit_write_back_to_pl(...)`
  - `pl_pe_circuit_write_back_to_pl_ex(...)`
  - `pl_pe_circuit_sample_layout(...)`
  - `pl_pe_circuit_sample_u8(...)`
  - `pl_pe_circuit_sample_digital_state_u8(...)`

其中：

- `pl_pe_circuit_write_back_to_pl(...)` 保持历史默认映射：`L->0.0`、`H->1.0`、`X->1.0`、`Z->1.0`
- `pl_pe_circuit_write_back_to_pl_ex(...)` 允许显式指定写回 PhysicsLab `Logic Output.Properties["状态"]` 时，`L/H/X/Z` 分别映射成什么数值

- PE→PL 导出：
  - `pe_to_pl_convert(...)`

- 自动布局：
  - `pl_experiment_auto_layout(...)`

这几组接口正是 `python/phy_engine/physicslab.py` 的底层基础。

## 8) 重要实现位置

- 接口声明：`include/phy_engine/dll_api.h`
- 默认实现：`src/dll_main.cpp`
- PL 适配器封装：`include/phy_engine/phy_lab_wrapper/pe_sim.h`
