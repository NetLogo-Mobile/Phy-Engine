# Phy-Engine 文档（简体中文）

本目录面向 `include/phy_engine/` 的 **公开 API**，覆盖 Phy-Engine（PE）的完整功能：混合模拟/数字求解、Verilog（可综合子集）编译与综合到 PE 网表、以及与 PhysicsLab `.sav` 的导入/导出与 C ABI（共享库）嵌入接口。

## 目录

- `00_头文件索引.md`：`include/phy_engine/` 的模块/头文件地图（按用途选 include）
- `01_快速开始.md`：用最小示例跑通 OP / 数字时钟 / Verilog 综合
- `02_核心概念与数据结构.md`：netlist / model / node / pin / branch / variant 等
- `03_电路求解与分析类型.md`：`phy_engine::circult`、OP/DC/AC/TR 等分析与设置
- `04_网表操作API.md`：`netlist::add_model/create_node/add_to_node/...` 的完整说明
- `05_模型库清单.md`：当前内置器件/模块清单（按头文件分组）
- `06_数字逻辑与Verilog子集.md`：4 态逻辑、`verilog::digital` 编译/仿真、`pe_synth` 综合
- `07_PhysicsLab互操作.md`：`.sav` 读写、PL↔PE 适配、PE→PL 导出、布局辅助
- `08_C_ABI_与共享库.md`：`dll_api.h`（create/analyze/sample…）与内存/调用约定
- `09_文件格式_工具_FAQ.md`：文件格式/工具模块、常见问题、性能与排障
- `10_Options_与配置参考.md`：所有 “options/setting/配置项/环境变量” 的逐项解释与示例
- `11_API_逐函数参考.md`：按模块列出公开入口函数/方法，并给出用法与典型流程

## 参考示例

文档中的用法示例主要来自 `test/`：

- 模拟电路 OP：`test/0004.solver/op.cpp`
- AC：`test/0012.ac/ac_omega.cpp`
- 数字电路：`test/0006.digital/digital.cpp`
- Verilog→PE 综合：`test/0015.verilog_compile/pe_synth_and2.cpp`
- PE→PL 导出：`test/0014.phy_lab_wrapper/pe_to_pl_smoke.cpp`
- 复杂 Verilog 工程示例：`test/0021.fp16_fpu/fp16_fpu_pe_sim_and_export.cpp`
