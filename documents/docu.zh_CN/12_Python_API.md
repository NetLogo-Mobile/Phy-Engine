# 12 Python API（`python/phy_engine/`）

本章说明仓库内置的 Python 包装层：`python/phy_engine/`。

设计目标：

- 不依赖 `pybind11` / `nanobind`
- 直接复用 `src/dll_main.cpp` 暴露的共享库 C ABI
- 同时覆盖三类能力：
  - 构建 PE 电路
  - 运行电路仿真并采样
  - 直接编译/仿真 Verilog，或把 Verilog 作为 PE 元件接进电路

---

## 1) 先构建共享库

当前 Python 层通过 `ctypes` 加载 `phyengine` 动态库，所以先需要构建：

```bash
cmake -S src -B build_py -DCMAKE_BUILD_TYPE=Release
cmake --build build_py --target phyengine -j4
```

典型产物：

- macOS: `build_py/libphyengine.dylib`
- Linux: `build_py/libphyengine.so`
- Windows: `build_py/Release/phyengine.dll`

Python 侧默认会尝试在这些常见路径中查找。
如果你的库不在默认路径，设置：

```bash
export PHY_ENGINE_LIB=/abs/path/to/libphyengine.dylib
```

---

## 2) 导入方式

仓库内直接使用：

```bash
PYTHONPATH=python python3
```

然后：

```python
from phy_engine import *
```

主要公开对象：

- `Circuit`
- `Element`
- `Wire`
- `ElementCode`
- `AnalyzeType`
- `DigitalState`
- `NetlistBuilder`
- `VerilogRuntime`
- `VerilogPortDir`
- `get_verilog_synth_config()`
- `set_verilog_synth_config(...)`
- `Experiment`
- `ExperimentCircuit`
- `ExperimentType`
- `WireColor`
- `AutoLayoutBackend`
- `AutoLayoutMode`
- `Position`
- `ExperimentElement`
- `WriteBackOptions`
- `circuit_to_experiment(...)`

---

## 3) 构建/仿真电路

下面用两个数字输入驱动一个 OR 门，再接一个数字输出：

```python
from phy_engine import AnalyzeType, Circuit, DigitalState, Element, ElementCode, Wire

with Circuit(
    elements=[
        Element(ElementCode.DIGITAL_OUTPUT),
        Element(ElementCode.DIGITAL_OR),
        Element(ElementCode.DIGITAL_INPUT, [DigitalState.H]),
        Element(ElementCode.DIGITAL_INPUT, [DigitalState.L]),
    ],
    wires=[
        Wire(0, 0, 1, 2),
        Wire(3, 0, 1, 1),
        Wire(2, 0, 1, 0),
    ],
) as circuit:
    circuit.set_analyze_type(AnalyzeType.ACOP)
    circuit.analyze()
    circuit.digital_clk()
    sample = circuit.sample()
    print(sample.components[0].pin_digital)
```

说明：

- `elements` 顺序就是建模顺序
- `Wire(ele_a, pin_a, ele_b, pin_b)` 对应 C ABI 的四元组连线
- `sample()` 返回按“非 ground 元件顺序”展开的 `ComponentSample`
- `ComponentSample.pin_digital` 是 `DigitalState` 元组，保留 `L/H/X/Z` 四态

常用方法：

- `set_analyze_type(...)`
- `set_tr(t_step, t_stop)`
- `set_ac_omega(omega)`
- `set_temperature(temp_c)`
- `set_tnom(tnom_c)`
- `set_model_digital(component_index, attribute_index, state)`
- `set_model_double_by_name(component_index, name, value)`
- `analyze()`
- `digital_clk()`
- `sample()`
- `analyze_and_sample()`

补充：

- `Circuit.sample_layout()` 可先拿到每个元件的 pin/branch 前缀和布局
- 这对把结果映射回你自己的上层网表结构很有用
- `Circuit.to_experiment(...)` 可直接把当前 PE 电路导出成 PhysicsLab `Experiment`

---

## 4) 用 `NetlistBuilder` 按“网表风格”构建电路

如果你更习惯 `add_model/create_node/add_to_node/merge_node` 这种构图方式，而不是一次性手写 `elements + wires`，可以用纯 Python 的 `NetlistBuilder`：

```python
from phy_engine import AnalyzeType, DigitalState, ElementCode, NetlistBuilder

b = NetlistBuilder()

out = b.add_element(ElementCode.DIGITAL_OUTPUT, name="out")
gate = b.add_element(ElementCode.DIGITAL_OR, name="gate")
a = b.add_element(ElementCode.DIGITAL_INPUT, [DigitalState.H], name="a")
b0 = b.add_element(ElementCode.DIGITAL_INPUT, [DigitalState.L], name="b")

b.connect(out, 0, gate, 2)
b.connect(b0, 0, gate, 1)
b.connect(a.pin(0), gate.pin(0))

with b.build() as circuit:
    circuit.set_analyze_type(AnalyzeType.ACOP)
    circuit.analyze()
    circuit.digital_clk()
    print(circuit.sample().components[0].pin_digital)
```

常用方法：

- `add_element(...)` / `add_model(...)`
- `create_node(...)`
- `add_to_node(node, element, pin)`
- `connect(ele_a, pin_a, ele_b, pin_b)`
- `connect(pin_ref_a, pin_ref_b)`
- `merge_nodes(dst, src)`
- `delete_model(...)`
- `build() -> Circuit`

说明：

- 这是“纯 Python 构图器”，最终会生成已有的 `Circuit`
- 它不是直接暴露 C++ `netlist::add_model(...)` 的 live 编辑句柄
- 但对 Python 侧来说，通常已经足够接近“原生网表编辑”的使用体验

---

## 5) 直接仿真 Verilog

`VerilogRuntime` 走的是“编译 + elaborate + 端口级仿真”路径，不会先综合成 PE primitive。

```python
from phy_engine import DigitalState, VerilogRuntime

src = """
module top(input a, input b, output y);
  assign y = a & b;
endmodule
"""

with VerilogRuntime(src, top="top") as runtime:
    runtime.set_port("a", DigitalState.H)
    runtime.set_port("b", DigitalState.L)
    runtime.tick_once()
    print(runtime.get_port("y"))  # L

    runtime.set_port("b", DigitalState.H)
    runtime.tick_once()
    print(runtime.get_port("y"))  # H
```

常用能力：

- `ports`：顶层 bit-level 端口列表（名称 + 方向）
- `signals`：顶层 signal 名称列表
- `module_names`
- `top_module_name`
- `preprocessed_source`
- `set_port(...)` / `get_port(...)`
- `set_signal(...)` / `get_signal(...)`
- `step(tick, process_sequential=True)`
- `tick_once()`
- `reset()`
- `snapshot_ports()`
- `snapshot_signals()`

如果需要 `\`include`，可以提供 include 搜索目录：

```python
runtime = VerilogRuntime(src, include_dirs=["rtl/include", "rtl/common"])
```

或者：

```python
runtime = VerilogRuntime.from_file("rtl/top.v")
```

`from_file(...)` 会自动把源文件所在目录加入 include 搜索路径。

---

## 6) 把 Verilog 当成电路元件

### 5.1 运行时 Verilog 模块

使用 `ElementCode.VERILOG_MODULE`：

```python
Element(
    ElementCode.VERILOG_MODULE,
    verilog_source=src,
    verilog_top="top",
)
```

这会把 Verilog 模块直接作为一个 PE 数字器件插入电路。

适合：

- 快速行为验证
- 保留模块层级
- 不想先综合成 primitive

### 5.2 综合成 PE 网表

使用 `ElementCode.VERILOG_NETLIST`：

```python
Element(
    ElementCode.VERILOG_NETLIST,
    verilog_source=src,
    verilog_top="top",
)
```

这会在 `create_circuit_ex()` 内部执行：

1. Verilog 编译
2. build_design
3. elaborate
4. `pe_synth`
5. 接到当前电路网表

适合：

- 想让 Verilog 被综合成 PE primitive
- 想和 PE 其它数字/模拟元件一起跑
- 后续还要做 PE/PL 导出

---

## 7) 综合默认选项

`ElementCode.VERILOG_NETLIST` 使用的是共享库里的全局综合默认值。

读取：

```python
from phy_engine import get_verilog_synth_config

cfg = get_verilog_synth_config()
print(cfg)
```

设置：

```python
from phy_engine import set_verilog_synth_config

set_verilog_synth_config(
    opt_level=4,
    assume_binary_inputs=True,
    optimize_wires=True,
    optimize_mul2=True,
    optimize_adders=True,
)
```

---

## 8) PhysicsLab `.sav` / `Experiment` 接口

Python 层现在也包装了 `phy_lab_wrapper` 的核心能力，可以直接构造、读写、布局和仿真 PhysicsLab 电路实验。

### 8.1 创建 / 保存 / 加载实验

```python
from phy_engine import Experiment, ExperimentType

with Experiment(ExperimentType.CIRCUIT) as ex:
    a = ex.add_circuit_element("Logic Input", (-1.0, 0.0, 0.0))
    b = ex.add_circuit_element("Logic Input", (1.0, 0.0, 0.0))
    g = ex.add_circuit_element("And Gate", (0.0, 0.0, 0.0))
    y = ex.add_circuit_element("Logic Output", (0.0, 0.0, 1.0))

    ex.connect(a, 0, g, 0)
    ex.connect(b, 0, g, 1)
    ex.connect(g, 2, y, 0)
    ex.set_element_property_number(a, "开关", 1.0)
    ex.set_element_property_number(b, "开关", 1.0)
    ex.save("and_gate.sav")
```

也可以读取现有 `.sav`：

```python
with Experiment.load_from_file("and_gate.sav") as ex:
    print(ex.dump())
```

常用方法：

- `Experiment.load_from_string(...)`
- `Experiment.load_from_file(...)`
- `dump(indent=2)`
- `elements(model_id=None)`
- `logic_inputs(by="label_or_identifier")`
- `logic_outputs(by="label_or_identifier")`
- `set_logic_inputs({...}, by="label_or_identifier")`
- `save(path, indent=2)`
- `add_circuit_element(...)`
- `connect(...)`
- `clear_wires()`
- `set_xyz_precision(...)`
- `set_element_xyz(...)`
- `set_camera(...)`
- `set_element_property_number(...)`
- `set_element_label(...)`
- `set_element_position(...)`
- `merge(...)`

如果你想批量观察/修改实验里的 `Logic Input` / `Logic Output`，现在可以直接拿 Python dict：

```python
with Experiment(ExperimentType.CIRCUIT) as ex:
    a = ex.add_circuit_element("Logic Input", (-1.0, 0.0, 0.0))
    y = ex.add_circuit_element("Logic Output", (1.0, 0.0, 0.0))
    ex.set_element_label(a, "A")
    ex.set_element_label(y, "Y")

    ex.set_logic_inputs({"A": 1.0}, by="label")
    print(ex.logic_inputs(by="label"))
    print(ex.logic_outputs(by="label"))
```

说明：

- `elements()` 会返回 `ExperimentElement` 元组，包含 `identifier/model_id/label/properties/statistics/raw`
- `by` 支持：
  - `"identifier"`
  - `"label"`
  - `"label_or_identifier"`
- 如果按 `label` 查找且存在重复 label，会抛异常，避免 silently 覆盖

### 8.2 从 PhysicsLab 实验构建 PE 仿真电路

`Experiment.build_pe_circuit()` 会调用 PL→PE 适配器，把 `.sav` 实验直接转成可仿真的 PE 电路：

```python
from phy_engine import AnalyzeType, Experiment

with Experiment.load_from_file("and_gate.sav") as ex:
    with ex.build_pe_circuit() as pe:
        pe.set_analyze_type(AnalyzeType.ACOP)
        pe.sync_inputs_from_pl(ex)
        pe.analyze()
        pe.digital_clk()
        sample = pe.sample()
        print(sample.components[-1].pin_digital)
        pe.write_back_to_pl(ex)
```

`ExperimentCircuit` 常用方法：

- `set_analyze_type(...)`
- `set_tr(...)`
- `set_ac_omega(...)`
- `analyze()`
- `digital_clk()`
- `sync_inputs_from_pl(experiment)`
- `write_back_to_pl(experiment, options=WriteBackOptions(...))`
- `write_back_to_pl(experiment, logic_output_low=..., logic_output_high=..., logic_output_x=..., logic_output_z=...)`
- `sample_and_write_back_to_pl(experiment, ...)`
- `write_back_and_read_logic_outputs(experiment, by="label_or_identifier", ...)`
- `sample_layout()`
- `sample()`

说明：

- `ExperimentCircuit.sample()` 现在走 `pl_pe_circuit_sample_digital_state_u8()`，可保留完整 `L/H/X/Z`
- `write_back_to_pl(...)` 默认保持历史行为：`L->0.0`、`H->1.0`、`X->1.0`、`Z->1.0`
- `sample_and_write_back_to_pl(...)` 只是一个更直观的别名；它和 `write_back_to_pl(...)` 一样，都会先采样当前 PE 状态再写回 PhysicsLab
- `write_back_and_read_logic_outputs(...)` 会先回写，再自动扫描所有 `Logic Output`，直接返回 Python `dict`
- 如果你希望把配置集中成一个对象，可以写：

```python
from phy_engine import WriteBackOptions

pe.write_back_to_pl(
    ex,
    options=WriteBackOptions(
        logic_output_x=0.25,
        logic_output_z=-1.0,
    ),
)
```

- 如果你希望把未知态单独映射到别的数值，可以显式传参：

```python
pe.write_back_to_pl(
    ex,
    logic_output_x=0.25,
    logic_output_z=-1.0,
)
```

也可以一步拿回批量输出：

```python
outputs = pe.write_back_and_read_logic_outputs(ex, by="label")
print(outputs)
```

- 这条链路特别适合“读取 `.sav` → 仿真 → 回写 `.sav`”的自动化脚本

### 8.3 自动布局

```python
from phy_engine import AutoLayoutMode, Experiment

with Experiment.load_from_file("and_gate.sav") as ex:
    stats = ex.auto_layout(
        (-2.0, -1.0, 0.0),
        (2.0, 1.0, 0.0),
        mode=AutoLayoutMode.FAST,
    )
    print(stats)
```

返回值 `AutoLayoutStats` 包含：

- `grid_w`
- `grid_h`
- `fixed_obstacles`
- `placed`
- `skipped`

---

## 9) PE → PhysicsLab 导出

你可以把已有 PE 电路导出成 PhysicsLab `Experiment`：

```python
from phy_engine import Circuit, Element, ElementCode, DigitalState, Wire

with Circuit(
    elements=[
        Element(ElementCode.DIGITAL_OUTPUT),
        Element(ElementCode.DIGITAL_AND),
        Element(ElementCode.DIGITAL_INPUT, [DigitalState.H]),
        Element(ElementCode.DIGITAL_INPUT, [DigitalState.H]),
    ],
    wires=[
        Wire(0, 0, 1, 2),
        Wire(3, 0, 1, 1),
        Wire(2, 0, 1, 0),
    ],
) as circuit:
    with circuit.to_experiment(
        keep_pl_macros=True,
        generate_wires=True,
    ) as ex:
        ex.save("exported_from_pe.sav")
```

如果你更喜欢函数式入口，也可以用：

```python
ex = circuit_to_experiment(circuit, keep_pl_macros=True)
```

常用导出选项：

- `fixed_pos=(x, y, z)`
- `element_xyz_coords`
- `keep_pl_macros`
- `include_linear`
- `include_ground`
- `generate_wires`
- `keep_unknown_as_placeholders`
- `drop_dangling_logic_inputs`

---

## 10) 错误处理

Python 包装层遇到失败时会抛 `PhyEngineError`。

底层错误字符串来自共享库的线程局部 `phy_engine_last_error()`。

例如 Verilog 编译失败时，异常信息会尽量带上：

- `line:column`
- 原始源码行
- `^` caret 指示位置

---

## 11) 代码位置

- Python FFI 装载：`python/phy_engine/_ffi.py`
- 电路接口：`python/phy_engine/circuit.py`
- 网表构图器：`python/phy_engine/builder.py`
- Verilog 接口：`python/phy_engine/verilog.py`
- PhysicsLab / PL↔PE：`python/phy_engine/physicslab.py`
- 新增 C ABI：`include/phy_engine/dll_api.h`
- 默认实现：`src/dll_main.cpp`
