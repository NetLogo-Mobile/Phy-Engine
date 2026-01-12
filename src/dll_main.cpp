#include <phy_engine/phy_engine.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <numbers>
#include <unordered_map>

#include <phy_engine/model/models/linear/transformer.h>
#include <phy_engine/model/models/linear/transformer_center_tap.h>
#include <phy_engine/model/models/linear/coupled_inductors.h>
#include <phy_engine/model/models/linear/op_amp.h>

#include <phy_engine/model/models/controller/relay.h>
#include <phy_engine/model/models/controller/comparator.h>

#include <phy_engine/model/models/generator/sawtooth.h>
#include <phy_engine/model/models/generator/square.h>
#include <phy_engine/model/models/generator/pulse.h>
#include <phy_engine/model/models/generator/triangle.h>

#include <phy_engine/model/models/non-linear/BJT_NPN.h>
#include <phy_engine/model/models/non-linear/BJT_PNP.h>
#include <phy_engine/model/models/non-linear/nmosfet.h>
#include <phy_engine/model/models/non-linear/pmosfet.h>
#include <phy_engine/model/models/non-linear/full_bridge_rectifier.h>

#include <phy_engine/model/models/digital/logical/yes.h>
#include <phy_engine/model/models/digital/logical/tri_state.h>
#include <phy_engine/model/models/digital/verilog_ports.h>

#include <phy_engine/verilog/digital/pe_synth.h>

// 并查集查找函数
static int uf_find(int x, int* parent, int* visited)
{
    if(parent[x] != x) { parent[x] = uf_find(parent[x], parent, visited); }
    visited[x] = 1;
    return parent[x];
}

// 从导线构建网表
void build_netlist_from_wires(phy_engine::netlist::netlist& nl,
                              int* elements,
                              int ele_size,
                              int* wires,
                              int wire_count,
                              ::phy_engine::netlist::model_pos* model_pos_arr)
{
    if(elements == nullptr || wires == nullptr || model_pos_arr == nullptr) { return; }
    if(ele_size <= 0 || wire_count <= 0) { return; }

    // Compute per-element pin counts from the actual model pin views. This avoids relying on a fixed MAX_PINS.
    ::std::size_t* base_offset = static_cast<::std::size_t*>(::std::calloc(static_cast<::std::size_t>(ele_size) + 1, sizeof(::std::size_t)));
    ::std::size_t* pin_count = static_cast<::std::size_t*>(::std::calloc(static_cast<::std::size_t>(ele_size), sizeof(::std::size_t)));
    if(base_offset == nullptr || pin_count == nullptr)
    {
        ::std::free(base_offset);
        ::std::free(pin_count);
        return;
    }

    int comp_id_for_pin_scan = 0;
    for(int ele_id = 0; ele_id < ele_size; ++ele_id)
    {
        if(!elements[ele_id])
        {
            pin_count[ele_id] = 0;
            continue;
        }
        auto model = get_model(nl, model_pos_arr[comp_id_for_pin_scan]);
        if(model == nullptr || model->ptr == nullptr)
        {
            pin_count[ele_id] = 0;
            ++comp_id_for_pin_scan;
            continue;
        }
        auto const pv = model->ptr->generate_pin_view();
        pin_count[ele_id] = pv.size;
        ++comp_id_for_pin_scan;
    }

    for(int ele_id = 0; ele_id < ele_size; ++ele_id) { base_offset[ele_id + 1] = base_offset[ele_id] + pin_count[ele_id]; }

    int const total_nodes = static_cast<int>(base_offset[ele_size]);

    // 扩展数组：增加一个地节点槽位
    int const GROUND_NODE_ID = total_nodes;  // 地节点的索引

    int* parent = (int*)malloc((static_cast<::std::size_t>(total_nodes) + 1) * sizeof(int));  // +1 给地节点
    int* visited = (int*)calloc(static_cast<::std::size_t>(total_nodes) + 1, sizeof(int));
    if(parent == nullptr || visited == nullptr)
    {
        ::std::free(base_offset);
        ::std::free(pin_count);
        ::std::free(parent);
        ::std::free(visited);
        return;
    }

    // 初始化并查集
    for(int i = 0; i <= total_nodes; i++) { parent[i] = i; }

    // 处理所有导线连接
    for(int i = 0; i < wire_count; i++)
    {
        int ele1 = wires[i * 4];
        int pin1 = wires[i * 4 + 1];
        int ele2 = wires[i * 4 + 2];
        int pin2 = wires[i * 4 + 3];

        if(ele1 < 0 || ele2 < 0 || ele1 >= ele_size || ele2 >= ele_size) { continue; }

        // 检查是否是接地元件
        int node1, node2;

        if(!elements[ele1])
        {
            node1 = GROUND_NODE_ID;  // 连接到地节点
        }
        else
        {
            if(pin1 < 0 || static_cast<::std::size_t>(pin1) >= pin_count[ele1]) { continue; }
            node1 = static_cast<int>(base_offset[ele1] + static_cast<::std::size_t>(pin1));
        }

        if(!elements[ele2])
        {
            node2 = GROUND_NODE_ID;  // 连接到地节点
        }
        else
        {
            if(pin2 < 0 || static_cast<::std::size_t>(pin2) >= pin_count[ele2]) { continue; }
            node2 = static_cast<int>(base_offset[ele2] + static_cast<::std::size_t>(pin2));
        }

        int root1 = uf_find(node1, parent, visited);
        int root2 = uf_find(node2, parent, visited);
        if(root1 != root2)
        {
            // 确保地节点总是根节点
            if(root1 == GROUND_NODE_ID)
            {
                parent[root2] = root1;  // 其他节点连接到地
            }
            else if(root2 == GROUND_NODE_ID)
            {
                parent[root1] = root2;  // 其他节点连接到地
            }
            else
            {
                parent[root2] = root1;  // 普通节点合并
            }
        }
    }

    // 创建节点映射表
    ::std::unordered_map<int, ::phy_engine::model::node_t*> node_map;

    // 为每个连通分量创建node_t（排除地节点）
    for(int i = 0; i < total_nodes; i++)
    {  // 注意：不包括GROUND_NODE_ID
        if(parent[i] == i && visited[i])
        {
            // 检查是否连接到地节点
            int root = uf_find(i, parent, visited);
            if(root == GROUND_NODE_ID)
            {
                // 连接到地的节点直接使用地节点
                node_map[i] = &get_ground_node(nl);
            }
            else
            {
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
    for(int ele_id = 0; ele_id < ele_size; ++ele_id)
    {
        if(!elements[ele_id]) { continue; }
        auto model = get_model(nl, model_pos_arr[comp_id]);
        if(model == nullptr || model->ptr == nullptr)
        {
            ++comp_id;
            continue;
        }

        auto pin_view = model->ptr->generate_pin_view();
        for(::std::size_t pin_id{}; pin_id < pin_view.size; pin_id++)
        {
            int node_id;

            node_id = static_cast<int>(base_offset[ele_id] + pin_id);

            // 只处理实际被使用的引脚
            if(visited[node_id])
            {
                int root = uf_find(node_id, parent, visited);
                auto& node = *node_map[root];
                add_to_node(nl, model_pos_arr[comp_id], pin_id, node);
            }
        }
        ++comp_id;
    }

    ::std::free(base_offset);
    ::std::free(pin_count);
    free(parent);
    free(visited);
}

::phy_engine::netlist::add_model_retstr add_model_via_code(phy_engine::netlist::netlist& nl, int element_code, double** curr_prop_ptr)
{
    // element_code是元件的code
    if(curr_prop_ptr == nullptr || *curr_prop_ptr == nullptr) { return {}; }
    switch(element_code)
    {
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
        case 5:
        {
            // VAC: Vp, freq(Hz), phase(deg)
            double const vp = *((*curr_prop_ptr)++);
            double const freq = *((*curr_prop_ptr)++);
            double const phase_deg = *((*curr_prop_ptr)++);
            return add_model(nl,
                             ::phy_engine::model::VAC{
                                 .m_Vp = vp,
                                 .m_omega = freq * (2.0 * ::std::numbers::pi),
                                 .m_phase = phase_deg * (::std::numbers::pi / 180.0),
                             });
        }
        case 6:
            // IDC
            return add_model(nl, ::phy_engine::model::IDC{.I = *((*curr_prop_ptr)++)});
        case 7:
        {
            // IAC: Ip, freq(Hz), phase(deg)
            double const ip = *((*curr_prop_ptr)++);
            double const freq = *((*curr_prop_ptr)++);
            double const phase_deg = *((*curr_prop_ptr)++);
            return add_model(nl,
                             ::phy_engine::model::IAC{
                                 .m_Ip = ip,
                                 .m_omega = freq * (2.0 * ::std::numbers::pi),
                                 .m_phase = phase_deg * (::std::numbers::pi / 180.0),
                             });
        }
        case 8:
            // VCCS
            return add_model(nl, ::phy_engine::model::VCCS{.m_g = *((*curr_prop_ptr)++)});
        case 9:
            // VCVS
            return add_model(nl, ::phy_engine::model::VCVS{.m_mu = *((*curr_prop_ptr)++)});
        case 10:
            // CCCS
            return add_model(nl, ::phy_engine::model::CCCS{.m_alpha = *((*curr_prop_ptr)++)});
        case 11:
            // CCVS
            return add_model(nl, ::phy_engine::model::CCVS{.m_r = *((*curr_prop_ptr)++)});
        case 12:
        {
            // single pole switch: 0/1
            double const v = *((*curr_prop_ptr)++);
            return add_model(nl, ::phy_engine::model::single_pole_switch{.cut_through = (v != 0.0)});
        }
        case 13:
        {
            // PN junction: Is, N, Isr, Nr, Temp, Ibv, Bv, Bv_set(0/1), Area
            ::phy_engine::model::PN_junction pn{};
            pn.Is = *((*curr_prop_ptr)++);
            pn.N = *((*curr_prop_ptr)++);
            pn.Isr = *((*curr_prop_ptr)++);
            pn.Nr = *((*curr_prop_ptr)++);
            pn.Temp = *((*curr_prop_ptr)++);
            pn.Ibv = *((*curr_prop_ptr)++);
            pn.Bv = *((*curr_prop_ptr)++);
            pn.Bv_set = (*((*curr_prop_ptr)++) != 0.0);
            pn.Area = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(pn));
        }
        case 14:
        {
            // Transformer: n
            return add_model(nl, ::phy_engine::model::transformer{.n = *((*curr_prop_ptr)++)});
        }
        case 15:
        {
            // Coupled inductors: L1, L2, k
            ::phy_engine::model::coupled_inductors kl{};
            kl.L1 = *((*curr_prop_ptr)++);
            kl.L2 = *((*curr_prop_ptr)++);
            kl.k = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(kl));
        }
        case 16:
        {
            // Transformer center tap: n_total
            return add_model(nl, ::phy_engine::model::transformer_center_tap{.n_total = *((*curr_prop_ptr)++)});
        }
        case 17:
        {
            // OpAmp: mu
            return add_model(nl, ::phy_engine::model::op_amp{.mu = *((*curr_prop_ptr)++)});
        }
        case 18:
        {
            // Relay: Von, Voff
            ::phy_engine::model::relay r{};
            r.Von = *((*curr_prop_ptr)++);
            r.Voff = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(r));
        }
        case 19:
        {
            // Comparator: Ll, Hl
            ::phy_engine::model::comparator c{};
            c.Ll = *((*curr_prop_ptr)++);
            c.Hl = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(c));
        }
        case 20:
        {
            // Sawtooth generator: Vh, Vl, freq(Hz), phase(rad)
            ::phy_engine::model::sawtooth_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 21:
        {
            // Square generator: Vh, Vl, freq(Hz), duty, phase(rad)
            ::phy_engine::model::square_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.duty = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 22:
        {
            // Pulse generator: Vh, Vl, freq(Hz), duty, phase(rad), tr, tf
            ::phy_engine::model::pulse_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.duty = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            g.tr = *((*curr_prop_ptr)++);
            g.tf = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 23:
        {
            // Triangle generator: Vh, Vl, freq(Hz), phase(rad)
            ::phy_engine::model::triangle_gen g{};
            g.Vh = *((*curr_prop_ptr)++);
            g.Vl = *((*curr_prop_ptr)++);
            g.freq = *((*curr_prop_ptr)++);
            g.phase = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(g));
        }
        case 50:
        {
            // NPN BJT: Is, N, BetaF, Temp, Area
            ::phy_engine::model::BJT_NPN q{};
            q.Is = *((*curr_prop_ptr)++);
            q.N = *((*curr_prop_ptr)++);
            q.BetaF = *((*curr_prop_ptr)++);
            q.Temp = *((*curr_prop_ptr)++);
            q.Area = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(q));
        }
        case 51:
        {
            // PNP BJT: Is, N, BetaF, Temp, Area
            ::phy_engine::model::BJT_PNP q{};
            q.Is = *((*curr_prop_ptr)++);
            q.N = *((*curr_prop_ptr)++);
            q.BetaF = *((*curr_prop_ptr)++);
            q.Temp = *((*curr_prop_ptr)++);
            q.Area = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(q));
        }
        case 52:
        {
            // NMOSFET: Kp, lambda, Vth
            ::phy_engine::model::nmosfet m{};
            m.Kp = *((*curr_prop_ptr)++);
            m.lambda = *((*curr_prop_ptr)++);
            m.Vth = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(m));
        }
        case 53:
        {
            // PMOSFET: Kp, lambda, Vth
            ::phy_engine::model::pmosfet m{};
            m.Kp = *((*curr_prop_ptr)++);
            m.lambda = *((*curr_prop_ptr)++);
            m.Vth = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(m));
        }
        case 54:
        {
            // Full bridge rectifier (uses internal default diode params)
            return add_model(nl, ::phy_engine::model::full_bridge_rectifier{});
        }
        case 200:
        {
            // Digital INPUT: state (0=L,1=H,2=X,3=Z)
            int const st = static_cast<int>(*(((*curr_prop_ptr)++)));
            ::phy_engine::model::digital_node_statement_t dv{::phy_engine::model::digital_node_statement_t::X};
            switch(st)
            {
                case 0: dv = ::phy_engine::model::digital_node_statement_t::L; break;
                case 1: dv = ::phy_engine::model::digital_node_statement_t::H; break;
                case 2: dv = ::phy_engine::model::digital_node_statement_t::X; break;
                case 3: dv = ::phy_engine::model::digital_node_statement_t::Z; break;
                default: dv = ::phy_engine::model::digital_node_statement_t::X; break;
            }
            return add_model(nl, ::phy_engine::model::INPUT{.outputA = dv});
        }
        case 201:
            // Digital OUTPUT
            return add_model(nl, ::phy_engine::model::OUTPUT{});
        case 202:
            // Digital OR gate
            return add_model(nl, ::phy_engine::model::OR{});
        case 203:
            // Digital YES (buffer)
            return add_model(nl, ::phy_engine::model::YES{});
        case 204:
            // Digital AND gate
            return add_model(nl, ::phy_engine::model::AND{});
        case 205:
            // Digital NOT gate
            return add_model(nl, ::phy_engine::model::NOT{});
        case 206:
            // Digital XOR gate
            return add_model(nl, ::phy_engine::model::XOR{});
        case 207:
            // Digital XNOR gate
            return add_model(nl, ::phy_engine::model::XNOR{});
        case 208:
            // Digital NAND gate
            return add_model(nl, ::phy_engine::model::NAND{});
        case 209:
            // Digital NOR gate
            return add_model(nl, ::phy_engine::model::NOR{});
        case 210:
            // Digital TRI (tri-state buffer)
            return add_model(nl, ::phy_engine::model::TRI{});
        case 211:
            // Digital IMP (implication)
            return add_model(nl, ::phy_engine::model::IMP{});
        case 212:
            // Digital NIMP (non-implication)
            return add_model(nl, ::phy_engine::model::NIMP{});
        case 220:
            // Digital HALF_ADDER
            return add_model(nl, ::phy_engine::model::HALF_ADDER{});
        case 221:
            // Digital FULL_ADDER
            return add_model(nl, ::phy_engine::model::FULL_ADDER{});
        case 222:
            // Digital HALF_SUBTRACTOR
            return add_model(nl, ::phy_engine::model::HALF_SUB{});
        case 223:
            // Digital FULL_SUBTRACTOR
            return add_model(nl, ::phy_engine::model::FULL_SUB{});
        case 224:
            // Digital MUL2
            return add_model(nl, ::phy_engine::model::MUL2{});
        case 225:
            // Digital DFF
            return add_model(nl, ::phy_engine::model::DFF{});
        case 226:
            // Digital TFF
            return add_model(nl, ::phy_engine::model::TFF{});
        case 227:
            // Digital T_BAR_FF
            return add_model(nl, ::phy_engine::model::T_BAR_FF{});
        case 228:
            // Digital JKFF
            return add_model(nl, ::phy_engine::model::JKFF{});
        case 229:
        {
            // Digital COUNTER4: init_value (0..15)
            double const v = *((*curr_prop_ptr)++);
            auto const init = (v < 0.0) ? 0u : (v > 15.0 ? 15u : static_cast<unsigned>(v));
            return add_model(nl, ::phy_engine::model::COUNTER4{.value = static_cast<::std::uint8_t>(init)});
        }
        case 230:
        {
            // Digital RANDOM_GENERATOR4: init_state (0..15)
            double const v = *((*curr_prop_ptr)++);
            auto const init = (v < 0.0) ? 0u : (v > 15.0 ? 15u : static_cast<unsigned>(v));
            return add_model(nl, ::phy_engine::model::RANDOM_GENERATOR4{.state = static_cast<::std::uint8_t>(init)});
        }
        case 231:
        {
            // Digital EIGHT_BIT_INPUT: value (0..255)
            double const v = *((*curr_prop_ptr)++);
            auto const init = (v < 0.0) ? 0u : (v > 255.0 ? 255u : static_cast<unsigned>(v));
            return add_model(nl, ::phy_engine::model::EIGHT_BIT_INPUT{.value = static_cast<::std::uint8_t>(init)});
        }
        case 232:
            // Digital EIGHT_BIT_DISPLAY
            return add_model(nl, ::phy_engine::model::EIGHT_BIT_DISPLAY{});
        case 233:
        {
            // Digital SCHMITT_TRIGGER: Vth_low,Vth_high,inverted(0/1),Ll,Hl
            ::phy_engine::model::SCHMITT_TRIGGER s{};
            s.Vth_low = *((*curr_prop_ptr)++);
            s.Vth_high = *((*curr_prop_ptr)++);
            s.inverted = (*((*curr_prop_ptr)++) != 0.0);
            s.Ll = *((*curr_prop_ptr)++);
            s.Hl = *((*curr_prop_ptr)++);
            return add_model(nl, ::std::move(s));
        }
        default:
            // ……待后续补充
            return {};
    }
}

namespace
{
    // Extended element code for Verilog module creation through create_circuit_ex().
    inline constexpr int PE_ELEMENT_VERILOG_MODULE = 300;
    inline constexpr int PE_ELEMENT_VERILOG_NETLIST = 301;

    struct verilog_text_tables
    {
        char const* const* texts{};
        ::std::size_t const* sizes{};
        ::std::size_t text_count{};
        ::std::size_t const* element_src_index{};  // length ele_size; out-of-range => no verilog
        ::std::size_t const* element_top_index{};  // length ele_size; out-of-range => default "top"
        ::std::size_t ele_size{};
    };

    inline ::fast_io::u8string_view get_u8_text(verilog_text_tables const& vt, ::std::size_t idx) noexcept
    {
        if(vt.texts == nullptr || vt.sizes == nullptr) { return {}; }
        if(idx >= vt.text_count) { return {}; }
        auto const* p = vt.texts[idx];
        auto const n = vt.sizes[idx];
        if(p == nullptr || n == 0) { return {}; }
        return ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(p), n};
    }

    inline ::fast_io::u8string_view get_verilog_src(verilog_text_tables const& vt, ::std::size_t ele_id) noexcept
    {
        if(vt.element_src_index == nullptr || ele_id >= vt.ele_size) { return {}; }
        return get_u8_text(vt, vt.element_src_index[ele_id]);
    }

    inline ::fast_io::u8string_view get_verilog_top(verilog_text_tables const& vt, ::std::size_t ele_id) noexcept
    {
        if(vt.element_top_index == nullptr || ele_id >= vt.ele_size) { return {}; }
        return get_u8_text(vt, vt.element_top_index[ele_id]);
    }

    inline ::phy_engine::netlist::add_model_retstr add_model_via_code_ex(::phy_engine::netlist::netlist& nl,
                                                                         int element_code,
                                                                         double** curr_prop_ptr,
                                                                         verilog_text_tables const* vt,
                                                                         ::std::size_t ele_id) noexcept
    {
        if(element_code == PE_ELEMENT_VERILOG_MODULE)
        {
            if(vt == nullptr) { return {}; }
            auto const src = get_verilog_src(*vt, ele_id);
            if(src.empty()) { return {}; }
            auto top = get_verilog_top(*vt, ele_id);
            if(top.empty())
            {
                static constexpr char8_t fallback_top[] = u8"top";
                top = ::fast_io::u8string_view{fallback_top, sizeof(fallback_top) - 1};
            }
            return add_model(nl, ::phy_engine::model::make_verilog_module(src, top));
        }

        // Otherwise, fall back to the numeric-property based creator.
        return add_model_via_code(nl, element_code, curr_prop_ptr);
    }
}  // namespace

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos);

extern "C" int circuit_set_analyze_type(void* circuit_ptr, ::std::uint32_t analyze_type_value)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->set_analyze_type(static_cast<::phy_engine::analyze_type>(analyze_type_value));
    return 0;
}

extern "C" int circuit_set_tr(void* circuit_ptr, double t_step, double t_stop)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->get_analyze_setting().tr.t_step = t_step;
    c->get_analyze_setting().tr.t_stop = t_stop;
    return 0;
}

extern "C" int circuit_set_ac_omega(void* circuit_ptr, double omega)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->get_analyze_setting().ac.sweep = ::phy_engine::analyzer::AC::sweep_type::single;
    c->get_analyze_setting().ac.omega = omega;
    return 0;
}

extern "C" int circuit_analyze(void* circuit_ptr)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    return c->analyze() ? 0 : 1;
}

extern "C" int circuit_digital_clk(void* circuit_ptr)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    c->digital_clk();
    return 0;
}

extern "C" int circuit_sample(void* circuit_ptr,
                              ::std::size_t* vec_pos,
                              ::std::size_t* chunk_pos,
                              ::std::size_t comp_size,
                              double* voltage,
                              ::std::size_t* voltage_ord,
                              double* current,
                              ::std::size_t* current_ord,
                              bool* digital,
                              ::std::size_t* digital_ord)
{
    if(circuit_ptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr ||
       current_ord == nullptr || digital == nullptr || digital_ord == nullptr)
    {
        return 1;
    }

    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    voltage_ord[0] = current_ord[0] = digital_ord[0] = 0;
    for(::std::size_t i{}; i < comp_size; ++i)
    {
        phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
        if(model == nullptr || model->ptr == nullptr)
        {
            voltage_ord[i + 1] = voltage_ord[i];
            current_ord[i + 1] = current_ord[i];
            digital_ord[i + 1] = digital_ord[i];
            continue;
        }

        auto const model_pin_view{model->ptr->generate_pin_view()};
        auto const model_branch_view{model->ptr->generate_branch_view()};

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            voltage[j + voltage_ord[i]] = (node != nullptr) ? node->node_information.an.voltage.real() : 0.0;
        }
        voltage_ord[i + 1] = voltage_ord[i] + model_pin_view.size;

        for(::std::size_t j{}; j < model_branch_view.size; ++j) { current[j + current_ord[i]] = model_branch_view.branches[j].current.real(); }
        current_ord[i + 1] = current_ord[i] + model_branch_view.size;

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            bool v{};
            if(node != nullptr && node->num_of_analog_node == 0)
            {
                v = (node->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state);
            }
            digital[j + digital_ord[i]] = v;
        }
        digital_ord[i + 1] = digital_ord[i] + model_pin_view.size;
    }

    return 0;
}

extern "C" int circuit_sample_u8(void* circuit_ptr,
                                 ::std::size_t* vec_pos,
                                 ::std::size_t* chunk_pos,
                                 ::std::size_t comp_size,
                                 double* voltage,
                                 ::std::size_t* voltage_ord,
                                 double* current,
                                 ::std::size_t* current_ord,
                                 ::std::uint8_t* digital,
                                 ::std::size_t* digital_ord)
{
    if(circuit_ptr == nullptr || vec_pos == nullptr || chunk_pos == nullptr || voltage == nullptr || voltage_ord == nullptr || current == nullptr ||
       current_ord == nullptr || digital == nullptr || digital_ord == nullptr)
    {
        return 1;
    }

    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    voltage_ord[0] = current_ord[0] = digital_ord[0] = 0;
    for(::std::size_t i{}; i < comp_size; ++i)
    {
        phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[i], chunk_pos[i]});
        if(model == nullptr || model->ptr == nullptr)
        {
            voltage_ord[i + 1] = voltage_ord[i];
            current_ord[i + 1] = current_ord[i];
            digital_ord[i + 1] = digital_ord[i];
            continue;
        }

        auto const model_pin_view{model->ptr->generate_pin_view()};
        auto const model_branch_view{model->ptr->generate_branch_view()};

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            voltage[j + voltage_ord[i]] = (node != nullptr) ? node->node_information.an.voltage.real() : 0.0;
        }
        voltage_ord[i + 1] = voltage_ord[i] + model_pin_view.size;

        for(::std::size_t j{}; j < model_branch_view.size; ++j) { current[j + current_ord[i]] = model_branch_view.branches[j].current.real(); }
        current_ord[i + 1] = current_ord[i] + model_branch_view.size;

        for(::std::size_t j{}; j < model_pin_view.size; ++j)
        {
            auto const* node{model_pin_view.pins[j].nodes};
            ::std::uint8_t v{};
            if(node != nullptr && node->num_of_analog_node == 0)
            {
                v = (node->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state) ? 1 : 0;
            }
            digital[j + digital_ord[i]] = v;
        }
        digital_ord[i + 1] = digital_ord[i] + model_pin_view.size;
    }

    return 0;
}

extern "C" int circuit_set_model_digital(void* circuit_ptr, ::std::size_t vec_pos, ::std::size_t chunk_pos, ::std::size_t attribute_index, ::std::uint8_t state)
{
    if(circuit_ptr == nullptr) { return 1; }
    auto* c = static_cast<::phy_engine::circult*>(circuit_ptr);
    auto& nl{c->get_netlist()};

    auto* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos, chunk_pos});
    if(model == nullptr || model->ptr == nullptr) { return 2; }

    ::phy_engine::model::digital_node_statement_t dv{::phy_engine::model::digital_node_statement_t::X};
    switch(state)
    {
        case 0: dv = ::phy_engine::model::digital_node_statement_t::L; break;
        case 1: dv = ::phy_engine::model::digital_node_statement_t::H; break;
        case 2: dv = ::phy_engine::model::digital_node_statement_t::X; break;
        case 3: dv = ::phy_engine::model::digital_node_statement_t::Z; break;
        default: dv = ::phy_engine::model::digital_node_statement_t::X; break;
    }

    ::phy_engine::model::variant vi{};
    vi.digital = dv;
    vi.type = ::phy_engine::model::variant_type::digital;
    return model->ptr->set_attribute(attribute_index, vi) ? 0 : 3;
}

extern "C" void* create_circuit(int* elements,
                                ::std::size_t ele_size,
                                int* wires,
                                ::std::size_t wires_size,
                                double* properties,
                                ::std::size_t** vec_pos,
                                ::std::size_t** chunk_pos,
                                ::std::size_t* comp_size)
{
    // TODO 在以后的版本中，或许应该在elements里面就不允许出现0（接地元件）
    if(vec_pos == nullptr || chunk_pos == nullptr || comp_size == nullptr) { return nullptr; }
    *vec_pos = nullptr;
    *chunk_pos = nullptr;
    *comp_size = 0;
    if(elements == nullptr || properties == nullptr) { return nullptr; }
    *vec_pos = (::std::size_t*)malloc(ele_size * sizeof(::std::size_t));
    *chunk_pos = (::std::size_t*)malloc(ele_size * sizeof(::std::size_t));
    if(*vec_pos == nullptr || *chunk_pos == nullptr)
    {
        ::std::free(*vec_pos);
        ::std::free(*chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }
    ::phy_engine::circult* c = reinterpret_cast<::phy_engine::circult*>(std::malloc(sizeof(::phy_engine::circult)));
    if(c == nullptr) [[unlikely]] { ::fast_io::fast_terminate(); }
    ::std::construct_at(c);

    c->set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c->get_analyze_setting()};

    // 步长设置
    setting.tr.t_step = 1e-6;
    setting.tr.t_stop = 1e-6;

    auto& nl{c->get_netlist()};

    phy_engine::netlist::model_pos* model_pos_arr = (phy_engine::netlist::model_pos*)malloc(ele_size * sizeof(phy_engine::netlist::model_pos));
    if(model_pos_arr == nullptr)
    {
        destroy_circuit(c, *vec_pos, *chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }

    double* curr_prop = properties;  // current 'properties', for we don't know the length of properties
    ::std::size_t curr_i{};          // curr_i以和i区分，前者说明的是第几个非接地元件 // TODO 如果要在elements里禁止接地元件，需要去掉curr_i
    for(::std::size_t i{}; i < ele_size; ++i)
    {
        // vec_pos和chunk_pos保序，这很重要
        if(elements[i])
        {
            auto [ele, ele_pos]{add_model_via_code(nl, elements[i], &curr_prop)};
            if(ele == nullptr)
            {
                ::std::free(model_pos_arr);
                destroy_circuit(c, *vec_pos, *chunk_pos);
                *vec_pos = nullptr;
                *chunk_pos = nullptr;
                return nullptr;
            }
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
    int const wire_count = static_cast<int>(wires_size / 4);
    if(wires != nullptr && wire_count > 0) { build_netlist_from_wires(nl, elements, static_cast<int>(ele_size), wires, wire_count, model_pos_arr); }

    ::std::free(model_pos_arr);

    return c;
}

extern "C" void* create_circuit_ex(int* elements,
                                   ::std::size_t ele_size,
                                   int* wires,
                                   ::std::size_t wires_size,
                                   double* properties,
                                   char const* const* texts,
                                   ::std::size_t const* text_sizes,
                                   ::std::size_t text_count,
                                   ::std::size_t const* element_src_index,
                                   ::std::size_t const* element_top_index,
                                   ::std::size_t** vec_pos,
                                   ::std::size_t** chunk_pos,
                                   ::std::size_t* comp_size)
{
    if(vec_pos == nullptr || chunk_pos == nullptr || comp_size == nullptr) { return nullptr; }
    *vec_pos = nullptr;
    *chunk_pos = nullptr;
    *comp_size = 0;
    if(elements == nullptr || properties == nullptr) { return nullptr; }

    verilog_text_tables vt{
        .texts = texts,
        .sizes = text_sizes,
        .text_count = text_count,
        .element_src_index = element_src_index,
        .element_top_index = element_top_index,
        .ele_size = ele_size,
    };

    *vec_pos = (::std::size_t*)malloc(ele_size * sizeof(::std::size_t));
    *chunk_pos = (::std::size_t*)malloc(ele_size * sizeof(::std::size_t));
    if(*vec_pos == nullptr || *chunk_pos == nullptr)
    {
        ::std::free(*vec_pos);
        ::std::free(*chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }

    ::phy_engine::circult* c = reinterpret_cast<::phy_engine::circult*>(std::malloc(sizeof(::phy_engine::circult)));
    if(c == nullptr) [[unlikely]] { ::fast_io::fast_terminate(); }
    ::std::construct_at(c);

    c->set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting{c->get_analyze_setting()};
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;

    auto& nl{c->get_netlist()};

    struct verilog_netlist_job
    {
        ::phy_engine::netlist::model_pos stub_pos{};
        ::std::shared_ptr<::phy_engine::verilog::digital::compiled_design> design{};
        ::phy_engine::verilog::digital::instance_state top{};
    };

    ::std::vector<verilog_netlist_job> verilog_jobs{};

    phy_engine::netlist::model_pos* model_pos_arr = (phy_engine::netlist::model_pos*)malloc(ele_size * sizeof(phy_engine::netlist::model_pos));
    if(model_pos_arr == nullptr)
    {
        destroy_circuit(c, *vec_pos, *chunk_pos);
        *vec_pos = nullptr;
        *chunk_pos = nullptr;
        return nullptr;
    }

    double* curr_prop = properties;
    ::std::size_t curr_i{};
    for(::std::size_t i{}; i < ele_size; ++i)
    {
        if(elements[i])
        {
            ::phy_engine::netlist::add_model_retstr ret{};

            if(elements[i] == PE_ELEMENT_VERILOG_NETLIST)
            {
                auto const src = get_verilog_src(vt, i);
                if(src.empty())
                {
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                auto top = get_verilog_top(vt, i);
                if(top.empty())
                {
                    static constexpr char8_t fallback_top[] = u8"top";
                    top = ::fast_io::u8string_view{fallback_top, sizeof(fallback_top) - 1};
                }

                ::phy_engine::verilog::digital::compile_options opt{};
                auto cr = ::phy_engine::verilog::digital::compile(src, opt);
                if(!cr.errors.empty() || cr.modules.empty())
                {
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                auto design = ::std::make_shared<::phy_engine::verilog::digital::compiled_design>(::phy_engine::verilog::digital::build_design(::std::move(cr)));
                if(design->modules.empty())
                {
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                auto const* chosen = ::phy_engine::verilog::digital::find_module(*design, top);
                if(chosen == nullptr) { chosen = __builtin_addressof(design->modules.front_unchecked()); }

                auto top_inst = ::phy_engine::verilog::digital::elaborate(*design, *chosen);

                ::fast_io::vector<::fast_io::u8string> pin_names{};
                if(top_inst.mod != nullptr)
                {
                    pin_names.reserve(top_inst.mod->ports.size());
                    for(auto const& p: top_inst.mod->ports) { pin_names.push_back(p.name); }
                }

                ret = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::VERILOG_PORTS{::std::move(pin_names)});
                if(ret.mod == nullptr)
                {
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }

                verilog_jobs.push_back(verilog_netlist_job{
                    .stub_pos = ret.mod_pos,
                    .design = ::std::move(design),
                    .top = ::std::move(top_inst),
                });
            }
            else
            {
                ret = add_model_via_code_ex(nl, elements[i], &curr_prop, &vt, i);
            }

            auto [ele, ele_pos]{ret};
            if(ele == nullptr)
            {
                ::std::free(model_pos_arr);
                destroy_circuit(c, *vec_pos, *chunk_pos);
                *vec_pos = nullptr;
                *chunk_pos = nullptr;
                return nullptr;
            }
            (*vec_pos)[curr_i] = ele_pos.vec_pos;
            (*chunk_pos)[curr_i] = ele_pos.chunk_pos;
            model_pos_arr[curr_i] = ele_pos;
            ++curr_i;
        }
    }
    *comp_size = curr_i;

    int const wire_count = static_cast<int>(wires_size / 4);
    if(wires != nullptr && wire_count > 0) { build_netlist_from_wires(nl, elements, static_cast<int>(ele_size), wires, wire_count, model_pos_arr); }

    // Expand synthesized Verilog modules into PE digital primitives and wire them to the already-connected port stub pins.
    for(auto& job: verilog_jobs)
    {
        auto* stub = ::phy_engine::netlist::get_model(nl, job.stub_pos);
        if(stub == nullptr || stub->ptr == nullptr)
        {
            ::std::free(model_pos_arr);
            destroy_circuit(c, *vec_pos, *chunk_pos);
            *vec_pos = nullptr;
            *chunk_pos = nullptr;
            return nullptr;
        }

        auto pv = stub->ptr->generate_pin_view();
        ::std::vector<::phy_engine::model::node_t*> port_nodes{};
        port_nodes.reserve(pv.size);
        for(::std::size_t pi{}; pi < pv.size; ++pi)
        {
            auto* n = pv.pins[pi].nodes;
            if(n == nullptr)
            {
                auto& created = ::phy_engine::netlist::create_node(nl);
                if(!::phy_engine::netlist::add_to_node(nl, *stub, pi, created))
                {
                    ::std::free(model_pos_arr);
                    destroy_circuit(c, *vec_pos, *chunk_pos);
                    *vec_pos = nullptr;
                    *chunk_pos = nullptr;
                    return nullptr;
                }
                n = __builtin_addressof(created);
            }
            port_nodes.push_back(n);
        }

        ::phy_engine::verilog::digital::pe_synth_error syn_err{};
        ::phy_engine::verilog::digital::pe_synth_options syn_opt{
            .allow_inout = true,
            .allow_multi_driver = true,
        };
        if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, job.top, port_nodes, &syn_err, syn_opt))
        {
            ::std::free(model_pos_arr);
            destroy_circuit(c, *vec_pos, *chunk_pos);
            *vec_pos = nullptr;
            *chunk_pos = nullptr;
            return nullptr;
        }
    }

    ::std::free(model_pos_arr);
    return c;
}

extern "C" void destroy_circuit(void* circuit_ptr, ::std::size_t* vec_pos, ::std::size_t* chunk_pos)
{  // 这里pos是否需要是void*？
    if(circuit_ptr)
    {
        ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(circuit_ptr);
        ::std::destroy_at(c);
        ::std::free(circuit_ptr);
        circuit_ptr = NULL;
    }
    if(vec_pos)
    {
        ::std::free(vec_pos);
        vec_pos = NULL;
    }
    if(chunk_pos)
    {
        ::std::free(chunk_pos);
        chunk_pos = NULL;
    }
}

void set_property(phy_engine::model::model_base* model, ::std::size_t index, double property)
{
    if(model == nullptr || model->ptr == nullptr) { return; }

    // First try numeric attribute.
    if(model->ptr->set_attribute(index, {.d{property}, .type{::phy_engine::model::variant_type::d}})) { return; }

    // Then try boolean (treat non-zero as true).
    if(model->ptr->set_attribute(index, {.boolean{property != 0.0}, .type{::phy_engine::model::variant_type::boolean}})) { return; }

    // Finally try digital (0 -> 0, 1 -> 1, others -> X).
    ::phy_engine::model::digital_node_statement_t dv{::phy_engine::model::digital_node_statement_t::indeterminate_state};
    if(property == 0.0) { dv = ::phy_engine::model::digital_node_statement_t::false_state; }
    else if(property == 1.0) { dv = ::phy_engine::model::digital_node_statement_t::true_state; }
    (void)model->ptr->set_attribute(index, {.digital{dv}, .type{::phy_engine::model::variant_type::digital}});
}

extern "C" int analyze_circuit(void* circuit_ptr,
                               ::std::size_t* vec_pos,
                               ::std::size_t* chunk_pos,
                               ::std::size_t comp_size,
                               int* changed_ele,
                               ::std::size_t* changed_ind,
                               double* changed_prop,
                               ::std::size_t prop_size,
                               double* voltage,
                               ::std::size_t* voltage_ord,
                               double* current,
                               ::std::size_t* current_ord,
                               bool* digital,
                               ::std::size_t* digital_ord)
{
    // prop相关不需要判断，因为确实可能为空
    // TODO 是否需要获取运行时间
    if(circuit_ptr && vec_pos && chunk_pos && voltage && voltage_ord && current && current_ord && digital && digital_ord)
    {
        ::phy_engine::circult* c = static_cast<::phy_engine::circult*>(circuit_ptr);
        auto& nl{c->get_netlist()};
        // 按需修改properties
        for(::std::size_t i{}; i < prop_size; ++i)
        {
            phy_engine::model::model_base* model = get_model(nl, ::phy_engine::netlist::model_pos{vec_pos[changed_ele[i]], chunk_pos[changed_ele[i]]});
            set_property(model, changed_ind[i], changed_prop[i]);
        }

        // 分析
        if(!c->analyze()) { return 1; }

        // 读取（不触发 digital_clk；需要的话调用方通过 circuit_digital_clk() 单独触发）
        return circuit_sample(circuit_ptr, vec_pos, chunk_pos, comp_size, voltage, voltage_ord, current, current_ord, digital, digital_ord);
    }
    return 0;
}
