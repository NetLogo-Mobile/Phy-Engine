# 11 API 逐函数参考（公开入口）

本章按头文件/模块列出 **对外入口函数/方法** 的用法与典型调用顺序。

原则：

- 只覆盖“库使用者应该直接调用”的 API（例如 `netlist::add_model`、`circult::analyze`、`pe_to_pl::convert`、`dll_api.h` 的 C 函数等）
- `details::` 命名空间与明显内部 helper 不在本章逐个解释（否则会变成把实现源码复制一遍）

---

## A. `phy_engine::circult`（求解器上下文）

头文件：`include/phy_engine/circuits/circuit.h`

### A1) `get_environment() -> environment&`

用途：读取/修改求解环境参数（容差/温度/r_open…）。

典型用法：

- 修改开关断开电阻：`c.get_environment().r_open = 1e12;`

### A2) `get_netlist() -> netlist::netlist&`

用途：拿到内部网表容器，向里面添加器件与连线。

### A3) `set_analyze_type(analyze_type)`

用途：选择分析模式（OP/DC/AC/TR…）。

注意：

- TR/TROP 还需要设置 `get_analyze_setting().tr`。

### A4) `get_analyze_setting() -> analyzer_storage_t&`

用途：设置 AC/TR 等参数（见 `10_Options_与配置参考.md`）。

### A5) `analyze() -> bool`

用途：执行一次分析。

行为要点：

- OP/DC：一次求解
- AC：单点则一次求解；扫频则写入 `ac_sweep_results`
- TR/TROP：在一次 `analyze()` 调用中循环多个时间步（直到 `t_stop`）

示例：

- OP：`test/0004.solver/op.cpp`
- AC：`test/0012.ac/ac_omega.cpp`
- TR：`test/0004.solver/tr.cpp`

### A6) `digital_clk()`

用途：推进一次数字 tick（组合 settle + 时序更新）。

常见顺序：

1) `c.analyze()`（确保 `prepare()` 已完成）
2) 修改数字输入模型 attribute
3) `c.digital_clk()`

示例：`test/0006.digital/digital.cpp`。

### A7) CUDA 相关（可选）

- `set_cuda_policy(cuda_solve_policy)`
- `set_cuda_node_threshold(size_t)`

---

## B. `phy_engine::netlist`（网表构建/编辑）

头文件：

- `include/phy_engine/netlist/netlist.h`
- `include/phy_engine/netlist/operation.h`

### B1) `get_ground_node(netlist&) -> node_t&`

用途：获取地节点（参考点）。

### B2) `add_model(netlist&, model) -> add_model_retstr`

用途：向网表添加一个器件模型。

返回：

- `model_base* mod`：新模型对象（类型擦除）
- `model_pos mod_pos`：稳定定位（vec_pos/chunk_pos）

示例：

- `auto [r1, r1_pos] = add_model(nl, model::resistance{.r=10.0});`

### B3) `delete_model(netlist&, model_pos) -> bool`

用途：删除/清除某个模型槽位（编辑器/动态构建场景）。

注意：删除的语义是“清空/标 null”，不一定立刻收缩内存。

### B4) `get_model(netlist const&, model_pos) -> model_base*`

用途：通过 `model_pos` 找回模型指针。

### B5) `get_num_of_model<check>(netlist const&) -> size_t`

用途：统计模型数量。

- `check=false`：快路径（利用分块统计）
- `check=true`：慢路径（逐个检查 `model_type`）

### B6) `create_node(netlist&) -> node_t&`

用途：创建一个新节点（网络）。

### B7) `add_to_node(netlist&, model_pos, pin_index, node_t&) -> bool`
### B8) `add_to_node(netlist const&, model_base&, pin_index, node_t&) -> bool`

用途：把某个模型的某个 pin 连接到 node。

实践建议：

- C++ 直接写测试/工具时，多用 `add_to_node(nl, *model, pin, node)`
- 需要稳定定位/跨语言时，用 `model_pos` 版本

### B9) `remove_from_node(...) -> bool`

用途：把 pin 从 node 上断开连接（与 `add_to_node` 相反）。

### B10) `delete_node(netlist const&, node_t&)`

用途：清空节点上的 pins（不会自动改模型结构）。

### B11) `merge_node(netlist const&, node_t& node, node_t& other)`

用途：合并两个节点（把 `other` 的所有 pins 转移到 `node`）。

---

## C. 浮空子网检测：`floating_subnet::detect`

头文件：`include/phy_engine/circuits/floating_subnet/detect.h`

### C1) `detect(netlist&) -> vector<vector<node_t*>>`

用途：找出所有“没有与 ground 连通”的子网。

返回：

- 每个子网是一个 `node_t*` 列表
- 返回空表示没有浮空子网

示例：`test/0010.floating_subnet/detect.cpp`。

---

## D. Verilog Digital Subset（编译/展开/仿真）

头文件：`include/phy_engine/verilog/digital/digital.h`

### D1) `preprocess(src, preprocess_options) -> preprocess_result`

用途：处理 `` `define/`ifdef/`include `` 等预处理指令，并生成 source_map（用于把错误位置映射回原始源码）。

关键点：

- 需要 include 时，提供 `preprocess_options::include_resolver`
- `preprocess_result::errors` 是 `compile_error` 列表

### D2) `lex(src[, source_map]) -> lex_result`

重载：

- `lex(src, source_map_ptr)`：带预处理映射，用于更准确的 line/column
- `lex(src)`：不带映射

用途：词法分析，输出 tokens 与 errors。

### D3) `compile(src[, compile_options]) -> compile_result`

用途：预处理 + 词法 + 解析 `module`，产出模块集合。

注意：

- `compile_result.errors` 非空时通常表示编译失败
- `compile_result.modules` 是“编译单元内所有 module”

### D4) `build_design(compile_result&&) -> compiled_design`

用途：把编译结果组织为“可展开的设计”（解析模块引用、参数等），供 elaboration 使用。

### D5) `find_module(design, name) -> compiled_module const*`

用途：按模块名查找顶层 module。

### D6) `elaborate(design, top_module) -> instance_state`

用途：展开顶层实例（构建层级、参数求值、端口展开等）。

### D7) `simulate(instance_state&, tick, process_sequential)`

用途：推进一次仿真。

- `tick`：外部提供的“时间/步号”，用于 `#delay` 与时序调度
- `process_sequential`：
  - `true`：执行时序块（posedge/negedge/always_ff）+ 组合 settle
  - `false`：只执行组合 settle（常用于某些两阶段调度）

### D8) 其它对外 helper（少量）

`digital.h` 中还暴露了一些表达式求值/调度结构（例如 `eval_expr`），通常只有在你做“自定义仿真器/调试器”时才需要。

---

## E. Verilog → PE 综合：`synthesize_to_pe_netlist`

头文件：`include/phy_engine/verilog/digital/pe_synth.h`

### E1) `synthesize_to_pe_netlist(nl, top, top_port_nodes, err, opt) -> bool`

用途：把 elaboration 的 `instance_state` 转成 PE 数字 primitive，并写入用户的 `netlist`。

参数要点：

- `top_port_nodes`：
  - `size()` 必须等于 `top.mod->ports.size()`
  - 顺序必须一致（第 i 个 node 就是第 i 个 port）
- `err`：
  - 失败时写入 `err->message`（若非空）
- `opt`：
  - 见 `10_Options_与配置参考.md` 的 `pe_synth_options`

示例：

- 最小 AND：`test/0015.verilog_compile/pe_synth_and2.cpp`
- FP16：`test/0021.fp16_fpu/fp16_fpu_pe_sim_and_export.cpp`

---

## F. PhysicsLab `.sav`：`experiment` 与布局

头文件：`include/phy_engine/phy_lab_wrapper/physicslab.h`

### F1) `experiment::create(type) -> experiment`

用途：创建一个带 PhysicsLab 默认模板的实验对象（保证能被官方客户端打开）。

### F2) `experiment::load(path)` / `load_from_string(content)`

用途：从 `.sav` 文件/字符串加载实验。

注意：兼容“完整 .sav（含 {Type, Experiment, Summary…}）”与“只含 Experiment object 的导出形式”。

### F3) `experiment::dump(indent) -> string`

用途：序列化为 JSON 文本。

### F4) `experiment::save(path|indent)`

用途：写出 `.sav`。

### F5) `experiment::add_circuit_element(model_id, pos, element_xyz_coords?, is_big_element?) -> string`

用途：往 circuit 实验里添加元件，返回 `Identifier`。

关键点：

- `element_xyz_coords` 若不传，默认使用 `experiment::element_xyz_enabled()`
- 大元件 `is_big_element=true` 会影响 element-xyz ↔ native 的 y 修正

### F6) `experiment::connect(src_id, src_pin, dst_id, dst_pin, color)`

用途：添加一根导线。

### F7) `experiment::merge(other, offset)`

用途：把另一个实验合并进来，并给所有元件位置加 offset；会重新生成 Identifier 并重连 wires。

### F8) 布局辅助：`layout::corner_locator`

头文件：`include/phy_engine/phy_lab_wrapper/layout_locator.h`

- `corner_locator::from_experiment(ex, markers)`
- `corner_locator::from_sav(path, markers)`
- `map_uv(u,v)` / `map_grid(x,y,w,h)`

用途：用四个角标记元件建立一个“坐标框”，方便把导出电路放到指定矩形区域。

---

## G. PE → PL 导出：`pe_to_pl::convert`

头文件：`include/phy_engine/phy_lab_wrapper/pe_to_pl.h`

### G1) `convert(netlist const&, options) -> result`

用途：把 PE 网表导出为 PhysicsLab circuit 实验。

行为：

- 只导出 **PE 的数字模型**（`model_device_type::digital`）
- 会把 PE 模型映射为 PhysicsLab `ModelID`（见 `map_pe_model_to_pl`）
- 可选择生成 wires（星形连接）
- 结果附带 `nets`（PE node → endpoints）与 `warnings`

示例：`test/0014.phy_lab_wrapper/pe_to_pl_smoke.cpp`。

---

## H. PL → PE 仿真适配：`phy_lab_wrapper::pe::circuit`

头文件：`include/phy_engine/phy_lab_wrapper/pe_sim.h`

这是对 `dll_api.h` 的 C++ 封装，典型调用流程：

1) `auto c = pe::circuit::build_from(experiment);`
2) `c.set_analyze_type(...)` / `c.set_tr(...)` / `c.set_ac_omega(...)`
3) 循环：
   - `c.sync_inputs_from_pl(ex)`（可选：同步逻辑输入开关）
   - `c.analyze()` / `c.digital_clk()`
   - `auto s = c.sample_now()`
   - `c.write_back_to_pl(ex, s)`（可选：写回 Statistics）

该类的公开方法（以头文件为准）：

- `static circuit build_from(experiment const&)`
- `close()` / 析构自动 close
- `comp_size()`
- `set_analyze_type(phy_engine_analyze_type)`
- `set_tr(t_step, t_stop)`
- `set_ac_omega(omega)`
- `analyze()`
- `digital_clk()`
- `sample_now() -> sample`
- `sync_inputs_from_pl(experiment const&)`
- `write_back_to_pl(experiment&, sample const&)`

---

## I. 求解器 C ABI：`dll_api.h`

头文件：`include/phy_engine/dll_api.h`

### I1) 创建/销毁

- `create_circuit(...) -> void*`
- `create_circuit_ex(...) -> void*`（支持 Verilog 文本表）
- `destroy_circuit(void*, vec_pos, chunk_pos)`

### I2) 仿真控制

- `circuit_set_analyze_type(ptr, analyze_type)`
- `circuit_set_tr(ptr, t_step, t_stop)`
- `circuit_set_ac_omega(ptr, omega)`
- `circuit_analyze(ptr)`
- `circuit_digital_clk(ptr)`

### I3) 采样/驱动

- `circuit_sample(...)` / `circuit_sample_u8(...)`
- `circuit_set_model_digital(...)`
- `analyze_circuit(...)`（变更属性 + analyze + sample 的组合接口）

强烈建议配合阅读：`documents/docu.zh_CN/08_C_ABI_与共享库.md`。

---

## J. wrapper 最小 C API：`phy_lab_wrapper/c_api.h`

头文件：`include/phy_engine/phy_lab_wrapper/c_api.h`

用途：给 wasm/绑定提供“操作 `.sav` JSON” 的最小接口（不是求解器 ABI）。

主要函数：

- `plw_last_error()`
- `plw_string_free(char*)`
- `plw_experiment_create(type_value)`
- `plw_experiment_load_from_string(sav_json)`
- `plw_experiment_dump(handle, indent)`
- `plw_experiment_add_circuit_element(handle, model_id, x,y,z, element_xyz_coords)`
- `plw_experiment_connect(handle, src_id, src_pin, dst_id, dst_pin, color_value)`

