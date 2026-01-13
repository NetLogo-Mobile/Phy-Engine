# 06 数字逻辑与 Verilog 子集

本章覆盖两条数字路径：

1) 直接使用 PE 内置数字 primitive（`model/models/digital/*`）搭建数字电路  
2) 使用 `verilog::digital` 编译/展开 Verilog 子集，并用 `pe_synth` 综合到 PE 数字 primitive

本章聚焦“能力与流程”；所有可调参数（options/配置项）的逐项解释请看：`documents/docu.zh_CN/10_Options_与配置参考.md`。

## 1) 4 态逻辑与传播

数字状态类型：`phy_engine::model::digital_node_statement_t`（`include/phy_engine/model/node/node.h`）

- `L`/`H`：0/1
- `X`：不确定
- `Z`：高阻

门级模型（如 `AND/OR/NOT`）通过 `circult::digital_clk()` 驱动更新。

提示：当 `node_t::num_of_analog_node != 0` 且不是纯模拟节点时，它是混合节点；数字模型可能会向 MNA 注入“等效电压源”（通过 `digital::need_operate_analog_node_t`）。

## 2) Verilog Digital Subset：编译与仿真

头文件：`include/phy_engine/verilog/digital/digital.h`

该实现故意只支持“可综合子集”，并内置预处理器（`\`define/\`include/...`）与 elaboration。功能列表见：

- `include/phy_engine/verilog/digital/README.md`

常用入口（命名以 `phy_engine::verilog::digital` 命名空间为准）：

- `preprocess(src, preprocess_options)`：预处理（可自定义 include_resolver）
- `lex(src)`：词法
- `compile(src)` / `compile(src, compile_options)`：编译
- `build_design(compilation_result)`：构建设计
- `find_module(design, name)`：查找模块
- `elaborate(design, module)`：展开顶层 instance（含子模块实例）
- `simulate(instance_state, tick, process_sequential)`：推进仿真（`VERILOG_MODULE` 内部使用）

相关 options（逐项解释见 `documents/docu.zh_CN/10_Options_与配置参考.md`）：

- `preprocess_options`：`\`include` 解析/深度限制
- `compile_options`：目前只封装 `preprocess_options`

## 3) Verilog → PE 网表综合（`pe_synth`）

头文件：`include/phy_engine/verilog/digital/pe_synth.h`

目标：把 elaboration 得到的 instance，转换成 PE 的数字 primitive 模型 + 连线，并写入用户提供的 `netlist::netlist`。

核心 API（签名见 `include/phy_engine/verilog/digital/pe_synth.h`）：

```cpp
bool synthesize_to_pe_netlist(
    phy_engine::netlist::netlist& nl,
    phy_engine::verilog::digital::instance_state const& top,
    std::vector<phy_engine::model::node_t*> const& top_port_nodes,
    phy_engine::verilog::digital::pe_synth_error* err = nullptr,
    phy_engine::verilog::digital::pe_synth_options const& opt = {}
) noexcept;
```

关键约束：

- `ports.size()` 必须等于 `top.mod->ports.size()`，并且顺序一致
- `ports[i]` 表示 Verilog 端口 `top.mod->ports[i]` 对应的外部节点
- 对端口位展开的工程（例如 `a[0]`…），常见做法是“每一位一个端口”（见 `test/0021.fp16_fpu/fp16_fpu_pe_sim_and_export.cpp`）

综合选项（`pe_synth_options`）：

- `allow_inout`：允许 `inout`
- `allow_multi_driver`：允许多驱动（会使用解析/三态等模型）
- `support_always_comb` / `support_always_ff`
- `optimize_adders`：best-effort：把门级加法器模式替换成 `HALF_ADDER/FULL_ADDER`
- `loop_unroll_limit`：动态 for/while 的有限展开上限

补充：如果你想查“综合失败时常见原因/如何定位是哪条 net 多驱动/怎么调 allow_multi_driver”，建议直接看 `documents/docu.zh_CN/11_API_逐函数参考.md` 的 E 节与 `documents/docu.zh_CN/10_Options_与配置参考.md` 的 B 节。

## 4) 另一种集成方式：`model::VERILOG_MODULE`

如果你不想把 Verilog “拆成 primitive”，也可以直接把 Verilog 模块当作一个数字器件插入网表：

- 头文件：`include/phy_engine/model/models/digital/verilog_module.h`

特点：

- 引脚列表动态生成，顺序与 Verilog port list 一致
- 运行时直接调用 `verilog::digital::simulate()`
- 支持端口连接到模拟/混合节点（通过 `Ll/Hl` 阈值把模拟电压采样为 0/1/X）

适用场景：

- 快速验证行为、保留层级结构、或不需要导出到 PhysicsLab primitive 网表的情况
