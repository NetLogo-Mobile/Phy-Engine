#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/netlist/operation.h>

#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace
{
using u8sv = ::fast_io::u8string_view;

::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}

std::string read_file_text(std::filesystem::path const& path)
{
    std::ifstream ifs(path, std::ios::binary);
    if(!ifs.is_open()) { throw std::runtime_error("failed to open: " + path.string()); }
    std::string s;
    ifs.seekg(0, std::ios::end);
    auto const n = static_cast<std::size_t>(ifs.tellg());
    ifs.seekg(0, std::ios::beg);
    s.resize(n);
    if(n != 0) { ifs.read(s.data(), static_cast<std::streamsize>(n)); }
    return s;
}

struct include_ctx
{
    std::filesystem::path base_dir;
};

bool include_resolver_fs(void* user, ::fast_io::u8string_view path, ::fast_io::u8string& out_text) noexcept
{
    try
    {
        auto* ctx = static_cast<include_ctx*>(user);
        std::string rel(reinterpret_cast<char const*>(path.data()), path.size());
        auto p = ctx->base_dir / rel;
        auto s = read_file_text(p);
        out_text.assign(::fast_io::u8string_view{reinterpret_cast<char8_t const*>(s.data()), s.size()});
        return true;
    }
    catch(...)
    {
        return false;
    }
}

std::optional<std::size_t> parse_bit_index(std::string_view s, std::string_view base)
{
    if(!s.starts_with(base)) { return std::nullopt; }
    if(s.size() < base.size() + 3) { return std::nullopt; }  // "a[0]"
    if(s[base.size()] != '[') { return std::nullopt; }
    if(s.back() != ']') { return std::nullopt; }
    auto inner = s.substr(base.size() + 1, s.size() - (base.size() + 2));
    if(inner.empty()) { return std::nullopt; }
    std::size_t v{};
    for(char ch : inner)
    {
        if(ch < '0' || ch > '9') { return std::nullopt; }
        v = v * 10 + static_cast<std::size_t>(ch - '0');
    }
    return v;
}

inline bool set_uN_vec_value(::phy_engine::verilog::digital::compiled_module const& m,
                             ::phy_engine::verilog::digital::instance_state& st,
                             u8sv base,
                             ::std::uint64_t value,
                             ::std::size_t nbits)
{
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end()) { return false; }
    auto const& vd{it->second};
    if(vd.bits.size() != nbits) { return false; }
    for(::std::size_t pos{}; pos < nbits; ++pos)
    {
        ::std::size_t const bit_from_lsb{nbits - 1 - pos};
        bool const b{bit_from_lsb < 64 ? (((value >> bit_from_lsb) & 1u) != 0u) : false};
        auto const sig{vd.bits.index_unchecked(pos)};
        if(sig >= st.state.values.size()) { return false; }
        st.state.values.index_unchecked(sig) = b ? ::phy_engine::verilog::digital::logic_t::true_state
                                                 : ::phy_engine::verilog::digital::logic_t::false_state;
    }
    return true;
}

inline bool read_uN_vec(::phy_engine::verilog::digital::compiled_module const& m,
                        ::phy_engine::verilog::digital::instance_state const& st,
                        u8sv base,
                        ::std::uint64_t& out,
                        ::std::size_t nbits)
{
    auto it{m.vectors.find(::fast_io::u8string{base})};
    if(it == m.vectors.end()) { return false; }
    auto const& vd{it->second};
    if(vd.bits.size() != nbits) { return false; }

    ::std::uint64_t v{};
    for(::std::size_t pos{}; pos < nbits; ++pos)
    {
        auto const sig{vd.bits.index_unchecked(pos)};
        if(sig >= st.state.values.size()) { return false; }
        auto const bit_from_lsb{nbits - 1 - pos};
        auto const bit = st.state.values.index_unchecked(sig);
        if(bit != ::phy_engine::verilog::digital::logic_t::false_state && bit != ::phy_engine::verilog::digital::logic_t::true_state)
        {
            std::fprintf(stderr, "read_uN_vec non-binary base=%.*s pos=%zu\n", static_cast<int>(base.size()), reinterpret_cast<char const*>(base.data()), pos);
            return false;
        }
        if(bit == ::phy_engine::verilog::digital::logic_t::true_state && bit_from_lsb < 64) { v |= (1ull << bit_from_lsb); }
    }
    out = v;
    return true;
}

inline void commit_prev(::phy_engine::verilog::digital::instance_state& st)
{
    st.state.prev_values = st.state.values;
    st.state.comb_prev_values = st.state.values;
}

struct fp16_case
{
    std::uint16_t a;
    std::uint16_t b;
    std::uint8_t op;  // 0:add 1:sub 2:mul 3:div
    std::uint16_t expected;
};

constexpr fp16_case kCases[] = {
    {0x3E00u, 0x4000u, 0u, 0x4300u},  // 1.5 + 2.0 = 3.5
    {0x4000u, 0x3E00u, 1u, 0x3800u},  // 2.0 - 1.5 = 0.5
    {0x3E00u, 0x4000u, 2u, 0x4200u},  // 1.5 * 2.0 = 3.0
    {0x4200u, 0x4000u, 3u, 0x3E00u},  // 3.0 / 2.0 = 1.5

    {0xBC00u, 0x3800u, 0u, 0xB800u},  // -1.0 + 0.5 = -0.5
    {0x3800u, 0x3C00u, 1u, 0xB800u},  // 0.5 - 1.0 = -0.5
    {0xBC00u, 0xC000u, 2u, 0x4000u},  // -1.0 * -2.0 = 2.0
    {0xC200u, 0x4000u, 3u, 0xBE00u},  // -3.0 / 2.0 = -1.5
};

constexpr double x_for_bit(std::size_t idx, std::size_t nbits)
{
    if(nbits <= 1) { return 0.0; }
    return -1.0 + 2.0 * (static_cast<double>(idx) / static_cast<double>(nbits - 1));
}
}  // namespace

int main()
{
    using namespace phy_engine;
    using namespace phy_engine::phy_lab_wrapper;
    using namespace phy_engine::verilog::digital;

    static constexpr std::size_t kA = 16;
    static constexpr std::size_t kB = 16;
    static constexpr std::size_t kOP = 2;
    static constexpr std::size_t kY = 16;

    auto const src_path = std::filesystem::path(__FILE__).parent_path() / "fp16_calc.v";
    auto const src_s = read_file_text(src_path);
    auto const src = ::fast_io::u8string_view{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

    // Compile once (used for both verilog-module simulation and PE synthesis).
    ::phy_engine::verilog::digital::compile_options copt{};
    include_ctx ictx{.base_dir = src_path.parent_path()};
    copt.preprocess.user = __builtin_addressof(ictx);
    copt.preprocess.include_resolver = include_resolver_fs;
    auto cr = ::phy_engine::verilog::digital::compile(src, copt);
    if(!cr.errors.empty() || cr.modules.empty())
    {
        if(!cr.errors.empty())
        {
            auto const& e = cr.errors.front_unchecked();
            throw std::runtime_error("verilog compile error at line " + std::to_string(e.line) + ": " +
                                     std::string(reinterpret_cast<char const*>(e.message.data()), e.message.size()));
        }
        return 1;
    }

    auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
    auto const* top_mod = ::phy_engine::verilog::digital::find_module(design, u8"fp16_calc_top");
    if(top_mod == nullptr) { return 2; }

    // 1) Verilog-module simulation (verilog::digital).
    {
        auto st = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
        if(st.mod == nullptr) { return 3; }

        std::size_t case_idx{};
        for(auto const& tc : kCases)
        {
            // Force input changes to trigger combinational evaluation.
            if(!set_uN_vec_value(*top_mod, st, u8sv{u8"a"}, static_cast<std::uint64_t>(tc.a ^ 0xFFFFu), kA)) { return 4; }
            if(!set_uN_vec_value(*top_mod, st, u8sv{u8"b"}, static_cast<std::uint64_t>(tc.b ^ 0xFFFFu), kB)) { return 5; }
            if(!set_uN_vec_value(*top_mod, st, u8sv{u8"op"}, static_cast<std::uint64_t>(tc.op ^ 0x3u), kOP)) { return 6; }
            commit_prev(st);

            if(!set_uN_vec_value(*top_mod, st, u8sv{u8"a"}, static_cast<std::uint64_t>(tc.a), kA)) { return 7; }
            if(!set_uN_vec_value(*top_mod, st, u8sv{u8"b"}, static_cast<std::uint64_t>(tc.b), kB)) { return 8; }
            if(!set_uN_vec_value(*top_mod, st, u8sv{u8"op"}, static_cast<std::uint64_t>(tc.op), kOP)) { return 9; }

            simulate(st, 10, false);

            std::uint64_t ra{};
            std::uint64_t rb{};
            std::uint64_t rop{};
            if(!read_uN_vec(*top_mod, st, u8sv{u8"a"}, ra, kA)) { return 10; }
            if(!read_uN_vec(*top_mod, st, u8sv{u8"b"}, rb, kB)) { return 10; }
            if(!read_uN_vec(*top_mod, st, u8sv{u8"op"}, rop, kOP)) { return 10; }
            if(static_cast<std::uint16_t>(ra) != tc.a || static_cast<std::uint16_t>(rb) != tc.b ||
               static_cast<std::uint8_t>(rop) != tc.op)
            {
                std::fprintf(stderr,
                             "verilog sim input readback mismatch case=%zu a=0x%04x b=0x%04x op=%u read_a=0x%04x read_b=0x%04x read_op=%u\n",
                             case_idx,
                             tc.a,
                             tc.b,
                             static_cast<unsigned>(tc.op),
                             static_cast<unsigned>(static_cast<std::uint16_t>(ra)),
                             static_cast<unsigned>(static_cast<std::uint16_t>(rb)),
                             static_cast<unsigned>(static_cast<std::uint8_t>(rop)));
                return 10;
            }

            std::uint64_t y{};
            if(!read_uN_vec(*top_mod, st, u8sv{u8"y"}, y, kY)) { return 10; }
            auto const got = static_cast<std::uint16_t>(y);
            if(got != tc.expected)
            {
                std::uint64_t y_add{};
                std::uint64_t y_sub{};
                std::uint64_t y_mul{};
                std::uint64_t y_div{};
                bool const has_add = read_uN_vec(*top_mod, st, u8sv{u8"y_add"}, y_add, kY);
                bool const has_sub = read_uN_vec(*top_mod, st, u8sv{u8"y_sub"}, y_sub, kY);
                bool const has_mul = read_uN_vec(*top_mod, st, u8sv{u8"y_mul"}, y_mul, kY);
                bool const has_div = read_uN_vec(*top_mod, st, u8sv{u8"y_div"}, y_div, kY);
                std::fprintf(stderr,
                             "verilog sim mismatch case=%zu a=0x%04x b=0x%04x op=%u got=0x%04x exp=0x%04x add=%s0x%04x sub=%s0x%04x mul=%s0x%04x div=%s0x%04x\n",
                             case_idx,
                             tc.a,
                             tc.b,
                             static_cast<unsigned>(tc.op),
                             got,
                             tc.expected,
                             has_add ? "" : "?",
                             has_add ? static_cast<unsigned>(static_cast<std::uint16_t>(y_add)) : 0u,
                             has_sub ? "" : "?",
                             has_sub ? static_cast<unsigned>(static_cast<std::uint16_t>(y_sub)) : 0u,
                             has_mul ? "" : "?",
                             has_mul ? static_cast<unsigned>(static_cast<std::uint16_t>(y_mul)) : 0u,
                             has_div ? "" : "?",
                             has_div ? static_cast<unsigned>(static_cast<std::uint16_t>(y_div)) : 0u);
                return 11;
            }
            ++case_idx;
        }
    }

    // 2) Verilog -> PE netlist compilation + 3) PE simulation + 4) PL (.sav) export with fixed IO placement.
    ::phy_engine::circult c{};
    c.set_analyze_type(::phy_engine::analyze_type::TR);
    auto& setting = c.get_analyze_setting();
    setting.tr.t_step = 1e-9;
    setting.tr.t_stop = 1e-9;
    auto& nl = c.get_netlist();

    auto top_inst = ::phy_engine::verilog::digital::elaborate(design, *top_mod);
    if(top_inst.mod == nullptr) { return 12; }

    // Port nodes in module port order.
    std::vector<::phy_engine::model::node_t*> ports{};
    ports.reserve(top_inst.mod->ports.size());
    for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
    {
        auto& n = ::phy_engine::netlist::create_node(nl);
        ports.push_back(__builtin_addressof(n));
    }

    std::unordered_map<std::string, ::phy_engine::model::model_base*> input_by_name{};
    input_by_name.reserve(top_inst.mod->ports.size());

    std::array<::phy_engine::model::model_base*, kA> in_a{};
    std::array<::phy_engine::model::model_base*, kB> in_b{};
    std::array<::phy_engine::model::model_base*, kOP> in_op{};
    std::array<std::size_t, kY> out_y{};
    in_a.fill(nullptr);
    in_b.fill(nullptr);
    in_op.fill(nullptr);
    out_y.fill(static_cast<std::size_t>(-1));

    auto set_if_match_input = [&](std::string_view port_name, ::phy_engine::model::model_base* m) -> bool {
        if(auto idx = parse_bit_index(port_name, "a"); idx && *idx < kA) { in_a[*idx] = m; return true; }
        if(auto idx = parse_bit_index(port_name, "b"); idx && *idx < kB) { in_b[*idx] = m; return true; }
        if(auto idx = parse_bit_index(port_name, "op"); idx && *idx < kOP) { in_op[*idx] = m; return true; }
        return false;
    };
    auto set_if_match_output = [&](std::string_view port_name, std::size_t pi) -> bool {
        if(auto idx = parse_bit_index(port_name, "y"); idx && *idx < kY) { out_y[*idx] = pi; return true; }
        return false;
    };

    // External IO models.
    for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
    {
        auto const& p = top_inst.mod->ports.index_unchecked(pi);
        std::string port_name(reinterpret_cast<char const*>(p.name.data()), p.name.size());

        if(p.dir == ::phy_engine::verilog::digital::port_dir::input)
        {
            auto [m, pos] =
                ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 13; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 14; }

            (void)set_if_match_input(port_name, m);
            input_by_name.emplace(std::move(port_name), m);
            continue;
        }

        if(p.dir == ::phy_engine::verilog::digital::port_dir::output)
        {
            auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
            (void)pos;
            if(m == nullptr || m->ptr == nullptr) { return 15; }
            m->name = p.name;
            if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *ports[pi])) { return 16; }

            (void)set_if_match_output(port_name, pi);
            continue;
        }

        // No inout at top-level for this test.
        return 17;
    }

    for(auto* m : in_a)
    {
        if(m == nullptr) { return 18; }
    }
    for(auto* m : in_b)
    {
        if(m == nullptr) { return 19; }
    }
    for(auto* m : in_op)
    {
        if(m == nullptr) { return 20; }
    }
    for(auto pi : out_y)
    {
        if(pi == static_cast<std::size_t>(-1)) { return 21; }
    }

    // Synthesize to PE netlist (digital primitives).
    ::phy_engine::verilog::digital::pe_synth_error err{};
    ::phy_engine::verilog::digital::pe_synth_options opt{
        .allow_inout = false,
        .allow_multi_driver = false,
    };
    if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, ports, &err, opt))
    {
        throw std::runtime_error("pe_synth failed: " + std::string(reinterpret_cast<char const*>(err.message.data()), err.message.size()));
    }

    // Export PE->PL (.sav) with deterministic IO placement (no auto placement).
    {
        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;

        static constexpr double kRowA = 0.75;
        static constexpr double kRowB = 0.25;
        static constexpr double kRowOP = -0.25;
        static constexpr double kRowY = -0.75;

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            if(ctx.pl_model_id == pl_model_id::logic_input)
            {
                auto idx_a = parse_bit_index(ctx.pe_instance_name, "a");
                if(idx_a && *idx_a < kA) { return position{x_for_bit(*idx_a, kA), kRowA, 0.0}; }

                auto idx_b = parse_bit_index(ctx.pe_instance_name, "b");
                if(idx_b && *idx_b < kB) { return position{x_for_bit(*idx_b, kB), kRowB, 0.0}; }

                auto idx_op = parse_bit_index(ctx.pe_instance_name, "op");
                if(idx_op && *idx_op < kOP) { return position{x_for_bit(*idx_op, kOP), kRowOP, 0.0}; }
            }

            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                auto idx_y = parse_bit_index(ctx.pe_instance_name, "y");
                if(idx_y && *idx_y < kY) { return position{x_for_bit(*idx_y, kY), kRowY, 0.0}; }
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
        assert(!r.ex.wires().empty());

        auto const out_path = std::filesystem::path("fp16_calc_pe_to_pl.sav");
        r.ex.save(out_path, 2);
        if(!std::filesystem::exists(out_path)) { return 22; }
        if(std::filesystem::file_size(out_path) < 128) { return 23; }

        // Validate deterministic IO placement.
        struct key
        {
            std::string model_id;
            std::string name;
        };
        struct key_hash
        {
            std::size_t operator()(key const& k) const noexcept
            {
                return std::hash<std::string>{}(k.model_id) ^ (std::hash<std::string>{}(k.name) << 1);
            }
        };
        struct key_eq
        {
            bool operator()(key const& a, key const& b) const noexcept { return a.model_id == b.model_id && a.name == b.name; }
        };

        std::unordered_map<key, position, key_hash, key_eq> pos_by_kind_and_name{};
        pos_by_kind_and_name.reserve(r.ex.elements().size());
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            std::string name;
            if(auto it = e.data().find("Label"); it != e.data().end() && it->is_string())
            {
                name = it->get<std::string>();
            }
            else
            {
                name.clear();
            }
            if(!mid.empty() && !name.empty())
            {
                pos_by_kind_and_name.emplace(key{mid, name}, e.element_position());
            }
        }

        auto assert_pos = [&](std::string_view mid, std::string name, position expected) -> bool {
            auto it = pos_by_kind_and_name.find(key{std::string(mid), std::move(name)});
            if(it == pos_by_kind_and_name.end()) { return false; }
            auto const p = it->second;
            assert(p.x == expected.x && p.y == expected.y && p.z == expected.z);
            return true;
        };

        for(std::size_t i{}; i < kA; ++i)
        {
            if(!assert_pos(pl_model_id::logic_input, "a[" + std::to_string(i) + "]", position{x_for_bit(i, kA), kRowA, 0.0})) { return 24; }
        }
        for(std::size_t i{}; i < kB; ++i)
        {
            if(!assert_pos(pl_model_id::logic_input, "b[" + std::to_string(i) + "]", position{x_for_bit(i, kB), kRowB, 0.0})) { return 25; }
        }
        for(std::size_t i{}; i < kOP; ++i)
        {
            if(!assert_pos(pl_model_id::logic_input, "op[" + std::to_string(i) + "]", position{x_for_bit(i, kOP), kRowOP, 0.0})) { return 26; }
        }
        for(std::size_t i{}; i < kY; ++i)
        {
            if(!assert_pos(pl_model_id::logic_output, "y[" + std::to_string(i) + "]", position{x_for_bit(i, kY), kRowY, 0.0})) { return 27; }
        }
    }

    // Run PE simulation.
    if(!c.analyze()) { return 28; }

    auto set_in = [&](::phy_engine::model::model_base* m, bool v) {
        (void)m->ptr->set_attribute(0, dv(v ? ::phy_engine::model::digital_node_statement_t::true_state
                                            : ::phy_engine::model::digital_node_statement_t::false_state));
    };
    auto compute_y_if_binary = [&]() -> std::optional<std::uint16_t> {
        std::uint16_t y{};
        for(std::size_t i{}; i < kY; ++i)
        {
            auto const s = ports[out_y[i]]->node_information.dn.state;
            if(s != ::phy_engine::model::digital_node_statement_t::false_state &&
               s != ::phy_engine::model::digital_node_statement_t::true_state)
            {
                return std::nullopt;
            }
            if(s == ::phy_engine::model::digital_node_statement_t::true_state) { y |= static_cast<std::uint16_t>(1u << i); }
        }
        return y;
    };

    auto force_and_set_inputs = [&](fp16_case const& tc) {
        // The PE update-table scheduler is edge-driven; if an input bit never changes from its
        // initial value, parts of the combinational netlist can remain X/Z forever.
        // So, force a toggle on every input bit first, then set the real value.
        for(std::size_t i{}; i < kA; ++i) { set_in(in_a[i], (((tc.a ^ 0xFFFFu) >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kB; ++i) { set_in(in_b[i], (((tc.b ^ 0xFFFFu) >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kOP; ++i) { set_in(in_op[i], (((tc.op ^ 0x3u) >> i) & 1u) != 0); }
        c.digital_clk();
        c.digital_clk();

        for(std::size_t i{}; i < kA; ++i) { set_in(in_a[i], ((tc.a >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kB; ++i) { set_in(in_b[i], ((tc.b >> i) & 1u) != 0); }
        for(std::size_t i{}; i < kOP; ++i) { set_in(in_op[i], ((tc.op >> i) & 1u) != 0); }
    };

    auto settle = [&]() noexcept
    {
        c.digital_clk();
        c.digital_clk();
        c.digital_clk();
        c.digital_clk();
    };

    for(auto const& tc : kCases)
    {
        force_and_set_inputs(tc);
        settle();
        auto const oy = compute_y_if_binary();
        if(!oy) { return 29; }
        if(*oy != tc.expected)
        {
            std::fprintf(stderr,
                         "pe sim mismatch a=0x%04x b=0x%04x op=%u got=0x%04x exp=0x%04x\n",
                         tc.a,
                         tc.b,
                         static_cast<unsigned>(tc.op),
                         static_cast<unsigned>(*oy),
                         tc.expected);
            return 29;
        }
    }

    return 0;
}
