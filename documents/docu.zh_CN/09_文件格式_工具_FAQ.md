# 09 文件格式 / 工具模块 / FAQ

## 1) 文件格式解析（parser/file_format）

入口头文件：`include/phy_engine/parser/file_format/impl.h`

目前状态：

- `pe-nl`：存在占位定义 `file_format::pe_nl::pe_nl`（`include/phy_engine/parser/file_format/pe-nl/pe-nl.h`），`load_file()` 目前返回 `false`（未实现）
- `hspice` / `pl-sav`：目录存在但 README 为空（未实现或待补文档）

如果你需要导入/导出网表，当前更推荐：

- 用 C++ 直接搭建 `netlist`
- 或用 `phy_lab_wrapper` 读写 PhysicsLab `.sav`

## 2) Verilog legacy parser（`verilog/parser`）

`include/phy_engine/verilog/parser/parser.h` 标注为 legacy：

- 当前行为主要是“词法化并把反引号指令报告为错误”（没有完整 AST）
- 新的可综合子集前端在 `include/phy_engine/verilog/digital/digital.h`

## 3) 小工具与版本信息（utils）

常见头文件：

- `include/phy_engine/utils/version.h`：`phy_engine::version` 与打印支持（仓库内可能通过宏注入 git hash）
- `include/phy_engine/utils/prefetch.h` / `tzset.h` / `io_device.h`：平台/IO 辅助
- `include/phy_engine/utils/ansies/*`、`consolecp/*`：控制台颜色与编码处理（偏基础设施）

## 4) FAQ / 排障

### Q1: 为什么不连地会出问题？

模拟 MNA 需要参考地；不连地会导致矩阵奇异或漂移。可用：

- `phy_engine::floating_subnet::detect(nl)` 找到未接地的子网（见 `test/0010.floating_subnet/detect.cpp`）

### Q2: 数字电路为什么要先 `analyze()`？

`circult::analyze()` 内部会调用 `prepare()`：

- 给 pin 填充 `pin.model`
- 建立节点/支路索引、MNA 结构、数字更新表

之后再 `digital_clk()` 才能正确传播。

### Q3: TR 步长/停止时间设置后 `analyze()` 会做什么？

TR 模式下，`analyze()` 会在一次调用里循环多个时间步（直到 `t_stop`），每步都会求解一次（见 `include/phy_engine/circuits/circuit.h`）。

### Q4: 性能不稳定/很慢？

可尝试：

- 确认网表规模与 `PHY_ENGINE_MNA_REUSE`（默认允许复用 pattern）
- 若编译启用 CUDA，调整 `cuda_policy` 与 `cuda_node_threshold`
- 若存在大量组合环，数字 update-table 可能会触发大量迭代；检查电路是否存在震荡组合逻辑

