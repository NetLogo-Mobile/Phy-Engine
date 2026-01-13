# 10 Options 与配置参考（非常重要）

本章集中解释 PE 里所有“配置项/可调参数”的含义与典型用法。这里的“配置”包含：

- 显式 options struct（例如 `pe_synth_options`、`pe_to_pl::options`、`compile_options`）
- 求解器设置（`circult::analyzer_storage_t`、`environment`、CUDA policy）
- 环境变量（`PHY_ENGINE_*`）

如果你遇到“不知道该调哪个参数”，优先从本章查。

---

## A. Verilog Digital Subset：预处理/编译 options

相关头文件：

- `include/phy_engine/verilog/digital/digital.h`

### A1) `preprocess_options`

定义：`phy_engine::verilog::digital::preprocess_options`

字段：

- `void* user`
  - 透传给 `include_resolver` 的用户指针，常用来传“工程根目录/搜索路径/自定义上下文”。
- `bool (*include_resolver)(void* user, fast_io::u8string_view path, fast_io::u8string& out_text) noexcept`
  - 处理 Verilog 里的 `` `include "..." `` / `` `include <...> ``。
  - 返回 `true` 表示 include 成功，并把文件内容写入 `out_text`。
  - 返回 `false` 表示 include 失败（编译结果会带 error）。
  - 这是“把 PE 当成 HDL 前端嵌入到你自己的工程系统”的关键钩子：你可以做文件系统读、内存映射、虚拟文件系统、或集成 IDE 的 include search。
- `uint32_t include_depth_limit`
  - include 递归深度限制（默认 32），用于避免循环 include 导致无限递归。

最常见用法：文件系统 include（参考 `test/0021.fp16_fpu/fp16_fpu_pe_sim_and_export.cpp`）。

实现思路：

1) 你在外部提供一个 `include_ctx{ base_dir }`  
2) `include_resolver` 把 `path` 拼到 `base_dir` 下读文件  
3) 把读到的 bytes 以 `u8string_view` 形式塞回 `out_text`

### A2) `compile_options`

定义：`phy_engine::verilog::digital::compile_options`

字段：

- `preprocess_options preprocess{}`

说明：

- 当前 `compile_options` 只包了一层 `preprocess`，也就是说：**编译时唯一可配置点就是 include/macro 预处理行为**。
- 如果你只用 `compile(src)`，则使用默认 `compile_options{}`（不启用 include_resolver）。

---

## B. Verilog → PE 网表综合：`pe_synth_options`

相关头文件：

- `include/phy_engine/verilog/digital/pe_synth.h`

定义：`phy_engine::verilog::digital::pe_synth_options`

字段：

- `allow_inout`（默认 `false`）
  - 是否允许综合 `inout` 端口/网络。
  - 若启用，综合器可能会生成三态/解析等结构以支持多驱动。
  - 如果你的 Verilog 工程没有 `inout`，建议保持 `false`，这样失败更早、更明确。
- `allow_multi_driver`（默认 `false`）
  - 是否允许同一 net 被多个 driver 驱动（多驱动）。
  - `false`：遇到多驱动直接报错（更接近“单驱动 RTL”）。
  - `true`：综合后会做 best-effort 的解析/降级（对 Z/X 语义、门级结构会更复杂）。
- `support_always_comb`（默认 `true`）
  - 是否接受 `always_comb`（SystemVerilog 风格）作为组合逻辑块。
  - 如果你只希望用 `always @*`，可以关掉让不符合子集的代码更早失败。
- `support_always_ff`（默认 `true`）
  - 是否接受 `always_ff`（SystemVerilog 风格）作为时序逻辑块。
- `optimize_adders`（默认 `false`）
  - best-effort：把识别到的门级加法器结构替换为 PE 内建 `HALF_ADDER/FULL_ADDER` 模型。
  - 目的：减少模型数量、加快仿真、并利于导出到 PhysicsLab 大元件。
  - 注意：这是一个启发式优化，适合“综合出来的门级网表”或“规律性很强的 RTL 结构”；不保证对所有写法都生效。
- `loop_unroll_limit`（默认 `64`）
  - 对动态 `for/while` 的有限展开上限（bounded unrolling）。
  - 数值越大：能支持更复杂的动态循环，但更容易生成巨大网表/编译更慢。

建议的配置策略：

- RTL/行为级且希望报错严格：`allow_inout=false`、`allow_multi_driver=false`（默认即可）
- 门级网表/存在多驱动：启用 `allow_multi_driver=true`，并配合 `pe_to_pl` 的 warnings 查看是否发生语义降级
- 大规模加法器很多：考虑 `optimize_adders=true`

---

## C. PE 求解器：分析参数与环境参数

相关头文件：

- `include/phy_engine/circuits/circuit.h`
- `include/phy_engine/circuits/analyze.h`
- `include/phy_engine/circuits/analyzer/impl.h`
- `include/phy_engine/circuits/environment/environment.h`

### C1) `analyze_type`

`analyze_type` 选择“这次 `circult::analyze()` 做什么”：

- `OP` / `DC`：直流类
- `AC` / `ACOP`：频域类
- `TR` / `TROP`：时域类

关键点：`TR/TROP` 需要先设置 `analyzer_setting.tr.t_step/t_stop`，否则会直接失败。

### C2) `analyzer_storage_t`（setting）

定义：`phy_engine::analyzer::analyzer_storage_t`，字段：

- `ac`：`AC{ sweep, omega, omega_start, omega_stop, points }`
- `dc`：`DC{ m_currentOmega }`（目前使用较少）
- `op`：`OP{}`（占位）
- `tr`：`TR{ t_stop, t_step }`

建议：

- AC 单点：设置 `ac.omega`
- AC 扫频：设置 `ac.sweep/omega_start/omega_stop/points`
- TR：设置 `tr.t_step/tr.t_stop`

### C3) `environment`（收敛/温度/开路电阻）

定义：`phy_engine::environment`

字段（常用）：

- `V_eps_max` / `V_epsr_max`：电压绝对/相对容差
- `I_eps_max` / `I_epsr_max`：电流绝对/相对容差
- `charge_eps_max`：电荷容差
- `g_min`：GMIN（目前更多用于扩展/预留）
- `r_open`：开路等效电阻（非常关键：开关/继电器断开时的等效电阻）
- `temperature` / `norm_temperature`：温度

经验：

- 模拟电路收敛问题：优先检查有没有浮空子网、以及容差设置是否过于严格
- 数字/混合电路：`r_open` 的默认行为决定“断开”是不是足够接近开路

---

## D. PE 求解器：CUDA policy（可选）

相关头文件：

- `include/phy_engine/circuits/circuit.h`
- `include/phy_engine/circuits/solver/cuda_sparse_lu.h`

当你以 NVCC 编译并链接 CUDA 版本时，`circult` 内部会持有一个 `cuda_sparse_lu`。

### D1) `circult::cuda_solve_policy`

- `auto_select`：自动（默认）
- `force_cpu`：强制 CPU（Eigen/MKL）
- `force_cuda`：强制 CUDA（如果不可用会失败或回退，取决于实现）

### D2) `cuda_node_threshold`

`cuda_node_threshold` 是“节点规模阈值”：

- 小电路：GPU 传输/初始化开销可能更大，不划算
- 大电路：GPU 可能更有优势

你可以：

- `c.set_cuda_policy(...)`
- `c.set_cuda_node_threshold(n)`

---

## E. PE → PhysicsLab 导出：`pe_to_pl::options`

相关头文件：

- `include/phy_engine/phy_lab_wrapper/pe_to_pl.h`

定义：`phy_engine::phy_lab_wrapper::pe_to_pl::options`

字段：

- `fixed_pos`
  - 默认所有元件放到一个固定坐标（除非你提供 `element_placer`）。
- `element_xyz_coords`
  - `true`：以 PhysicsLab “element-xyz 坐标系”写入 `.sav`（更接近 PhysicsLab 内部网格/对齐）
  - `false`：以 native 坐标写入 `.sav`
  - 更详细坐标含义见 `physicslab.h` 的 `element_xyz`。
- `keep_pl_macros`
  - `true`：尽量把 PE 的宏模型映射回 PhysicsLab 的“大元件”（例如 `COUNTER4` → `"Counter"`）。
  - `false`：倾向于映射为更基础的 primitive（若可行）。
- `generate_wires`
  - `true`：为每个 PE net 生成 PhysicsLab 导线（星形连接）。
  - `false`：只生成元件不连线（用于布局/调试）。
- `keep_unknown_as_placeholders`
  - 遇到未知/不支持的 PE 数字模型时，是否用占位元件代替（默认 `false` 会直接报错/拒绝导出）。
- `element_placer`
  - 回调：为每个导出元件提供自定义摆放位置。
  - 输入参数 `placement_context`：
    - `pe_model_name`：PE 模型名（如 `"OUTPUT"`）
    - `pe_instance_name`：PE 实例名（来自 `model_base::name`，如 `"y[0]"`；可能为空）
    - `pl_model_id`：将要创建的 PhysicsLab `ModelID`
    - `is_big_element`：是否为“大元件”
  - 返回 `std::nullopt`：使用 `fixed_pos`

典型用法：按 bit 位排列 IO（参考 `test/0021.fp16_fpu/fp16_fpu_pe_sim_and_export.cpp`）。

---

## F. PhysicsLab `.sav`：全局与 per-call 设置

相关头文件：

- `include/phy_engine/phy_lab_wrapper/physicslab.h`

### F1) `experiment::set_element_xyz(enabled, origin)`

- `enabled=true`：后续 `add_circuit_element()` 默认使用 element-xyz 坐标（除非你传 `element_xyz_coords` 覆盖）
- `origin`：element-xyz ↔ native 坐标变换的原点

### F2) `experiment::open(...)`

用于在“默认 physicsLabSav 目录”里按 sav 名查找/创建：

- `open_mode::load_by_sav_name`
- `open_mode::load_by_filepath`
- `open_mode::crt`（创建）

---

## G. 环境变量（`PHY_ENGINE_*`）

相关位置：`include/phy_engine/circuits/circuit.h`、`include/phy_engine/circuits/solver/cuda_sparse_lu.h`

已知环境变量（当前仓库代码中显式读取的）：

- `PHY_ENGINE_PROFILE_SOLVE=1`：启用求解阶段 profiling（实现内部决定如何输出）
- `PHY_ENGINE_PROFILE_SOLVE_VALIDATE=1`：启用 profiling 的额外校验
- `PHY_ENGINE_MNA_REUSE=0`：禁用 MNA pattern 复用（默认允许复用）
- `PHY_ENGINE_CUDA_PINNED=1`：CUDA 侧启用 pinned host memory（减少 H2D/D2H 开销）

建议：

- 如果你在 benchmark/大电路：先试 `PHY_ENGINE_MNA_REUSE`（默认是开）与 `PHY_ENGINE_CUDA_PINNED`
- 如果你在调试求解性能：打开 `PHY_ENGINE_PROFILE_SOLVE`

