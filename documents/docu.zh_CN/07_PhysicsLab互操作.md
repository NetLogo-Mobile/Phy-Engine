# 07 PhysicsLab 互操作（`.sav`）

PE 仓库内置 `phy_lab_wrapper`，用于读写 PhysicsLab 的 `.sav`（本质是 JSON），并提供：

- PL `.sav` → PE：从 PhysicsLab 元件/导线构建 PE 电路并仿真
- PE → PL `.sav`：把 PE 网表导出为 PhysicsLab 的电路实验（便于可视化/布局）

相关头文件都在 `include/phy_engine/phy_lab_wrapper/`。

本章聚焦“接口与流程”；所有 options/配置项的逐项解释请看：`documents/docu.zh_CN/10_Options_与配置参考.md`。

## 1) `.sav` 数据结构：`physicslab.h`

头文件：`include/phy_engine/phy_lab_wrapper/physicslab.h`

提供：

- `experiment`：实验（circuit/celestial/electromagnetism）
- `element` / `wire`：元件与导线
- `experiment::load(path)` / `load_from_string()` / `dump(indent)`：读写 JSON
- 坐标辅助：`position`、`element_xyz`（PhysicsLab 的 element-xyz 与 native 坐标互转）

## 2) PL → PE 仿真适配：`pe_sim.h`

头文件：`include/phy_engine/phy_lab_wrapper/pe_sim.h`

该适配器通过 `dll_api.h`（C ABI）调用 PE 的 shared-library 实现，把 PL 元件映射为 PE 元件码：

- PL 的 `ModelID` 常量集中在 `include/phy_engine/phy_lab_wrapper/pe_model_id.h`
- 映射表与属性读取逻辑主要在 `pe_sim.h` 的 `detail::to_phy_engine_code_and_props()`

对外推荐使用 `phy_engine::phy_lab_wrapper::pe::circuit`：

- `circuit::build_from(experiment const&)`：从电路实验构建 PE 电路
- `set_analyze_type/set_tr/set_ac_omega/analyze/digital_clk`
- `sync_inputs_from_pl(experiment const&)`：把 PL 输入开关同步到 PE（常用于 `Logic Input`）
- `sample_now()`：不改变状态地采样（电压/电流/数字）
- `write_back_to_pl(experiment&, sample const&)`：把仿真结果写回 `.sav` 的 Statistics/Properties

注意：该适配器要求链接 `create_circuit()/analyze_circuit()/...` 的实现（本仓库实现位于 `src/dll_main.cpp`）。

## 3) PE → PL 导出：`pe_to_pl.h`

头文件：`include/phy_engine/phy_lab_wrapper/pe_to_pl.h`

入口：

- `phy_engine::phy_lab_wrapper::pe_to_pl::convert(netlist const&, options)` → `result`

结果包含：

- `result.ex`：生成的 PhysicsLab `experiment`
- `result.nets`：PE 节点到 PL 端点的映射（便于调试连线）
- `result.warnings`：降级/不支持模型的告警

示例：`test/0014.phy_lab_wrapper/pe_to_pl_smoke.cpp`、`test/0021.fp16_fpu/fp16_fpu_pe_sim_and_export.cpp`。

常用导出选项（`options`）：

- `fixed_pos`：默认把所有元件放在同一位置（若不提供 placer）
- `element_xyz_coords`：是否使用 element-xyz 坐标写入 `.sav`
- `keep_pl_macros`：尽量保留 PhysicsLab 的“大元件”（如 COUNTER4 → Counter）
- `generate_wires`：是否生成导线
- `element_placer`：回调，用于按 `pe_model_name/pe_instance_name/pl_model_id` 自定义摆放位置

`pe_to_pl::options` 的逐字段语义与典型策略（例如按位排布 IO、是否保留宏元件）见：`documents/docu.zh_CN/10_Options_与配置参考.md`（E 节）。

## 4) 布局定位辅助：`layout_locator.h`

头文件：`include/phy_engine/phy_lab_wrapper/layout_locator.h`

用于从 `.sav` 中找到四角标记元件并建立一个坐标映射（unit-square/grid → native 坐标），常用于“把导出电路放到指定版图区域”。

## 5) 最小 C API（便于 wasm/绑定）：`c_api.h`

头文件：`include/phy_engine/phy_lab_wrapper/c_api.h`

这是 wrapper 层的最小 C 接口（不是求解器的 C ABI），目前覆盖：

- `plw_experiment_create/load_from_string/dump/connect/...`
- `plw_last_error()` 获取线程局部错误信息
