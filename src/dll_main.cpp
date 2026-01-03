#include <phy_engine/phy_engine.h>
#include <iostream>

// 并查集查找函数
static int uf_find(int x, int* parent, int* visited) {
    if (parent[x] != x) {
        parent[x] = uf_find(parent[x], parent, visited);
    }
    visited[x] = 1;
    return parent[x];
}

// 从导线构建网表
void build_netlist_from_wires(phy_engine::netlist::netlist& nl,
                             int* elements, int ele_size,
                             int* wires, int wire_count,
                             ::phy_engine::netlist::model_pos* model_pos_arr) {
    // TODO 可以把MAX_PINS优化掉
    const int MAX_PINS = 10;
    int total_nodes = ele_size * MAX_PINS;
    
    // 扩展数组：增加一个地节点槽位
    int GROUND_NODE_ID = total_nodes;  // 地节点的索引
    
    int* parent = (int*)malloc((total_nodes + 1) * sizeof(int));  // +1 给地节点
    int* visited = (int*)calloc(total_nodes + 1, sizeof(int));

    // 初始化并查集
    for (int i = 0; i <= total_nodes; i++) {
        parent[i] = i;
    }
    
    // 处理所有导线连接
    for (int i = 0; i < wire_count; i++) {
        int ele1 = wires[i*4];
        int pin1 = wires[i*4+1];
        int ele2 = wires[i*4+2];
        int pin2 = wires[i*4+3];
        
        // 检查是否是接地元件
        int node1, node2;
        
        if (!elements[ele1]) {
            node1 = GROUND_NODE_ID;  // 连接到地节点
        } else {
            node1 = ele1 * MAX_PINS + pin1;
        }
        
        if (!elements[ele2]) {
            node2 = GROUND_NODE_ID;  // 连接到地节点
        } else {
            node2 = ele2 * MAX_PINS + pin2;
        }
        
        int root1 = uf_find(node1, parent, visited);
        int root2 = uf_find(node2, parent, visited);
        if (root1 != root2) {
            // 确保地节点总是根节点
            if (root1 == GROUND_NODE_ID) {
                parent[root2] = root1;  // 其他节点连接到地
            } else if (root2 == GROUND_NODE_ID) {
                parent[root1] = root2;  // 其他节点连接到地
            } else {
                parent[root2] = root1;  // 普通节点合并
            }
        }
    }
    
    // 创建节点映射表
    ::std::unordered_map<int, ::phy_engine::model::node_t*> node_map;
    
    // 为每个连通分量创建node_t（排除地节点）
    for (int i = 0; i < total_nodes; i++) {  // 注意：不包括GROUND_NODE_ID
        if (parent[i] == i && visited[i]) {
            // 检查是否连接到地节点
            int root = uf_find(i, parent, visited);
            if (root == GROUND_NODE_ID) {
                // 连接到地的节点直接使用地节点
                node_map[i] = &get_ground_node(nl);
            } else {
                // 创建新的普通节点
                auto& node = create_node(nl);
                node_map[i] = &node;
            }
        }
    }
    
    // 添加地节点到映射表
    node_map[GROUND_NODE_ID] = &get_ground_node(nl);
    
    // 连接所有引脚到对应的节点
    int comp_id = 0;
    for (int ele_id = 0; ele_id < ele_size; ++ele_id) {
        if (!elements[ele_id]) continue;
        auto model = get_model(nl, model_pos_arr[comp_id]);

        auto pin_view = model->ptr->generate_pin_view();
        for (::std::size_t pin_id{}; pin_id < pin_view.size; pin_id++) {
            int node_id;

            node_id = ele_id * MAX_PINS + pin_id;
            
            // 只处理实际被使用的引脚
            if (visited[node_id]) {
                int root = uf_find(node_id, parent, visited);
                auto& node = *node_map[root];
                add_to_node(nl, model_pos_arr[comp_id], pin_id, node);
            }
        }
        ++comp_id;
    }
    
    free(parent);
    free(visited);
}

::phy_engine::netlist::add_model_retstr add_model_via_code(phy_engine::netlist::netlist& nl, int element_code, double** curr_prop_ptr) {
    // element_code是元件的code
    switch (element_code) {
        // 对每个case，语法应该是
        // add_model(nl, ::phy_engine::model::[model名]{.[属性1] = *((*curr_prop_ptr)++), .[属性2] = *((*curr_prop_ptr)++), ...});
        // 后面那一坨会自动++
        case 1:
            // Resistor
            return add_model(nl, ::phy_engine::model::resistance{.r = *((*curr_prop_ptr)++)});
        case 2:
            // Capacitor
            return add_model(nl, ::phy_engine::model::capacitor{.m_kZimag = *((*curr_prop_ptr)++)});
        case 3:
            // Inductor
            return add_model(nl, ::phy_engine::model::inductor{.m_kZimag = *((*curr_prop_ptr)++)});
        case 4:
            // VDC
            return add_model(nl, ::phy_engine::model::VDC{.V = *((*curr_prop_ptr)++)});
        default:
            // ……待后续补充
            return {};
    }
}

extern "C"
void* create_circuit(int* elements, ::std::size_t ele_size,
                     int* wires, ::std::size_t wires_size,
                     double* properties,
                     ::std::size_t** vec_pos, ::std::size_t** chunk_pos, ::std::size_t* comp_size) {
    // TODO 在以后的版本中，或许应该在elements里面就不允许出现0（接地元件）
    *vec_pos = (::std::size_t*)malloc(ele_size * sizeof(::std::size_t));
    *chunk_pos = (::std::size_t*)malloc(ele_size * sizeof(::std::size_t));
    ::phy_engine::circult* c = reinterpret_cast<::phy_engine::circult*>(std::malloc(sizeof(::phy_engine::circult)));
    if(c == nullptr) [[unlikely]] {
        ::fast_io::fast_terminate();
    }
    ::std::construct_at(c);

    c->set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c->get_analyze_setting()};

    // 步长设置
    setting.tr.t_step = 0.000001;
    setting.tr.t_stop = 0.00001;

    auto& nl{c->get_netlist()};

    phy_engine::netlist::model_pos* model_pos_arr = (phy_engine::netlist::model_pos*)malloc(ele_size * sizeof(phy_engine::netlist::model_pos));

    double* curr_prop = properties; // current 'properties', for we don't know the length of properties
    ::std::size_t curr_i{}; // curr_i以和i区分，前者说明的是第几个非接地元件 // TODO 如果要在elements里禁止接地元件，需要去掉curr_i
    for (::std::size_t i{}; i < ele_size; ++i) {
        // vec_pos和chunk_pos保序，这很重要
        if (elements[i]) {
            auto [ele, ele_pos]{add_model_via_code(nl, elements[i], &curr_prop)};
            (*vec_pos)[curr_i] = ele_pos.vec_pos;
            (*chunk_pos)[curr_i] = ele_pos.chunk_pos;
            model_pos_arr[curr_i] = ele_pos;
            ++curr_i;
        }

        /* TEMP
        if (elements[i]) { // 非接地元件
            auto [ele, ele_pos]{add_model_via_code(nl, elements[i], &curr_prop)};
            (*vec_pos)[i] = ele_pos.vec_pos;
            (*chunk_pos)[i] = ele_pos.chunk_pos;
            model_pos_arr[i] = ele_pos;
        } else { // 接地元件
            (*vec_pos)[i] = 0;
            (*chunk_pos)[i] = 0;
            model_pos_arr[i] = {};
        }
        */
    }
    *comp_size = curr_i;

    // 默认wires元素取值不会超过ele_size，且默认wires_size为4的倍数

    // 在这里调用build_netlist_from_wires，使用model_pos_arr获取model_pos后构建元件
    // TODO comp_size应该是去掉接地元件的数目
    // TODO 要深入考虑一下到底应不应该在pos里加上接地元件
    // TODO 再检查一下build_netlist_from_wires
    build_netlist_from_wires(nl, elements, ele_size, wires, wires_size, model_pos_arr);

    return c;
}

extern "C"
void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos) { // 这里pos是否需要是void*？
    if (circuit_ptr) {
        ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(circuit_ptr);
        ::std::destroy_at(c);
        ::std::free(circuit_ptr);
        circuit_ptr = NULL;
    }
    if (vec_pos) {
        ::std::free(vec_pos);
        vec_pos = NULL;
    }
    if (chunk_pos) {
        ::std::free(chunk_pos);
        chunk_pos = NULL;
    }
}

void set_property(phy_engine::model::model_base* model, ::std::size_t index, double property) {
    // 是否有必要用elements（code数组）重写
    ::fast_io::u8string_view name = model->ptr->get_model_name();
    // 写这里的时候需要查看每个元件的set_attribute_define，如果有多个可能的index则用switch
    // 对于类型不是d的情况，是否需要显式转换？
    if (name == ::fast_io::u8string_view{u8"Resistance"}) {
        model->ptr->set_attribute(index, {.d{property}, .type{::phy_engine::model::variant_type::d}});
    } else if (name == ::fast_io::u8string_view{u8"VDC"}) {
        model->ptr->set_attribute(index, {.d{property}, .type{::phy_engine::model::variant_type::d}});
    }
}

extern "C"
int analyze_circuit(void* circuit_ptr,
                    ::std::size_t* vec_pos, ::std::size_t* chunk_pos, ::std::size_t comp_size,
                    int* changed_ele, ::std::size_t* changed_ind, double* changed_prop, ::std::size_t prop_size,
                    double* voltage, ::std::size_t* voltage_ord,
                    double* current, ::std::size_t* current_ord,
                    bool* digital, ::std::size_t* digital_ord) {
    // prop相关不需要判断，因为确实可能为空
    // TODO 是否需要获取运行时间
    if (circuit_ptr && vec_pos && chunk_pos && voltage && voltage_ord && current && current_ord && digital && digital_ord) {
        ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(circuit_ptr);
        auto& nl{c->get_netlist()};
        // 按需修改properties
        for (::std::size_t i{}; i < prop_size; ++i) {
            phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[changed_ele[i]], chunk_pos[changed_ele[i]]});
            set_property(model, changed_ind[i], changed_prop[i]);
        }

        // 分析和读取
        if (!c->analyze()) { return 1; }
        else {
            voltage_ord[0] = current_ord[0] = digital_ord[0] = 0;
            for (::std::size_t i{}; i < comp_size; ++i) {
                // 输出里不包含接地元件的数据，也没有算入_ord中。物实端读取的时候应注意
                phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
                auto const model_pin_view{model->ptr->generate_pin_view()};
                auto const model_branch_view{model->ptr->generate_branch_view()};

                // 获取电压，存入voltage并将其起始位置存入voltage_ord。后续程序编写中应注意保持各引脚的顺序（位置）。电压数据和pin的数量一样多。
                // voltage_ord似乎需要比ele_size多一位来存储voltage数列的大小
                for (::std::size_t j{}; j < model_pin_view.size; ++j) {
                    voltage[j + voltage_ord[i]] = model_pin_view.pins[j].nodes->node_information.an.voltage.real(); 
                }
                voltage_ord[i + 1] = voltage_ord[i] + model_pin_view.size;

                // 电流。数据量因元件而异，大多为1，也有0或2
                for (::std::size_t j{}; j < model_branch_view.size; ++j) {
                    current[j + current_ord[i]] = model_branch_view.branches[j].current.real(); 
                }
                current_ord[i + 1] = current_ord[i] + model_branch_view.size;

                // TODO: 逻辑状态获取（是否需要实现、实现方式取决于物实）
                digital_ord[i + 1] = digital_ord[i];
            }
        }
    }
    return 0;
}
