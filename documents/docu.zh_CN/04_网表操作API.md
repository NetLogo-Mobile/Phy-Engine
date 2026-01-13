# 04 网表操作 API（`phy_engine::netlist`）

本章覆盖 `include/phy_engine/netlist/operation.h`（以及 `impl.h`）中对外可用的网表构建/修改接口。

## 1) 添加器件：`add_model`

头文件：`include/phy_engine/netlist/operation.h`

```cpp
auto [m, pos] = phy_engine::netlist::add_model(nl, phy_engine::model::resistance{.r = 10.0});
```

- 返回 `add_model_retstr{ model_base* mod, model_pos mod_pos }`
- `model_pos` 是稳定定位（`vec_pos`/`chunk_pos`），用于跨 FFI 或存档
- `model_base*` 可直接用（如 `m->ptr->generate_pin_view()`）

注意：

- `add_model` 对模型类型有约束（需要能生成 pin_view，且能参与 MNA 或是合法数字模型）

## 2) 创建节点：`create_node` 与地节点

```cpp
auto& n1 = phy_engine::netlist::create_node(nl);
auto& gnd = phy_engine::netlist::get_ground_node(nl); // 或 nl.ground_node
```

每个 `node_t` 表示一个“电气网络”（net）。

## 3) 连线：`add_to_node` / `remove_from_node` / `merge_node`

最常用的连线方式是把“器件引脚”挂到节点上：

```cpp
phy_engine::netlist::add_to_node(nl, *m, /*pin*/0, n1);
```

`add_to_node` 有两套重载：

- `add_to_node(nl, model_pos, pin, node)`：适合外部只存 `model_pos` 的场景
- `add_to_node(nl, model_base&, pin, node)`：适合 C++ 直接操作（测试代码多用这一种）

`merge_node(nl, a, b)` 会把 `b` 上的所有 pins 转移到 `a`，并 `destroy()` 旧节点。

## 4) 删除与查询：`delete_model/get_model/get_num_of_model`

常用接口：

- `delete_model(nl, model_pos)`：标记/清除某个模型槽位
- `get_model(nl, model_pos)`：取回 `model_base*`
- `get_num_of_model(nl)`：统计模型数量（可选 `check=true` 走慢路径精确计数）

这些接口主要用于编辑器/交互式场景或 FFI。

## 5) 常见坑：pin 的 `model` 指针

部分算法需要 `pin.model` 已被填好（例如浮空子网检测、数字更新遍历等）。通常 `circult::prepare()` 会在分析前为每个 pin 填充 `pin.model = model_base*`。

如果你在“纯网表层”调用某些算法（不经过 `circult::analyze()`），可能需要手动确保：

- 参见 `include/phy_engine/circuits/floating_subnet/detect.h` 的 `ensure_pin_model_ptr()`

## 6) 示例：检测浮空子网（floating subnet）

接口：`phy_engine::floating_subnet::detect(netlist&)`（`include/phy_engine/circuits/floating_subnet/detect.h`）

示例：`test/0010.floating_subnet/detect.cpp`。

