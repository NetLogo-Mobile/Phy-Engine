#include <cassert>
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <fast_io/fast_io_dsal/string.h>

#include <phy_engine/circuits/circuit.h>
#include <phy_engine/model/models/controller/comparator.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/model/models/digital/verilog_module.h>
#include <phy_engine/model/models/linear/VDC.h>
#include <phy_engine/model/models/linear/resistance.h>
#include <phy_engine/netlist/operation.h>
#include <phy_engine/phy_lab_wrapper/auto_layout/auto_layout.h>
#include <phy_engine/phy_lab_wrapper/pe_model_id.h>
#include <phy_engine/phy_lab_wrapper/pe_to_pl.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

namespace
{
using ::phy_engine::phy_lab_wrapper::position;
namespace pl_model_id = ::phy_engine::phy_lab_wrapper::pl_model_id;

constexpr std::size_t kLevels = 16;                // 16 buckets
constexpr std::size_t kThresholds = kLevels - 1;   // 15 thresholds
constexpr double kVref = 5.0;
constexpr double kRladder = 1000.0;
constexpr double kRin = 10000.0;

[[noreturn]] void die(char const* msg)
{
    std::fprintf(stderr, "adc16_onehot test fatal: %s\n", msg);
    std::abort();
}

inline void require(bool ok, char const* msg)
{
    if(!ok) { die(msg); }
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

inline ::phy_engine::model::variant dv(::phy_engine::model::digital_node_statement_t v) noexcept
{
    ::phy_engine::model::variant vi{};
    vi.digital = v;
    vi.type = ::phy_engine::model::variant_type::digital;
    return vi;
}

void append_u8_decimal(::fast_io::u8string& s, std::size_t v)
{
    auto tmp = std::to_string(v);
    s.reserve(s.size() + tmp.size());
    for(char ch : tmp) { s.push_back(static_cast<char8_t>(ch)); }
}

void append_u8(::fast_io::u8string& s, std::u8string_view v)
{
    s.reserve(s.size() + v.size());
    for(char8_t ch : v) { s.push_back(ch); }
}

std::size_t expected_bin(double vin) noexcept
{
    // Thresholds at (i+1)/16 * Vref.
    // bin 0: vin < 1/16 Vref; bin 15: vin >= 15/16 Vref.
    if(!(vin >= 0.0)) { return 0; }
    for(std::size_t i{}; i < kThresholds; ++i)
    {
        double const vth = (static_cast<double>(i + 1) / static_cast<double>(kLevels)) * kVref;
        if(vin < vth) { return i; }
    }
    return kLevels - 1;
}

struct adc_net
{
    ::phy_engine::netlist::netlist* nl{};
    ::phy_engine::model::model_base* vin_src{};  // optional
    std::vector<::phy_engine::model::model_base*> out_probes{};  // OUTPUT models, size=16
    std::vector<::phy_engine::model::node_t*> cmp_nodes{};       // size=15
    std::vector<::phy_engine::model::node_t*> out_nodes{};       // size=16
};

adc_net build_adc(::phy_engine::netlist::netlist& nl,
                  bool with_vin_source,
                  bool use_verilog_module_runtime,
                  bool use_verilog_synth)
{
    require(!(use_verilog_module_runtime && use_verilog_synth), "invalid build_adc flags");

    adc_net r{};
    r.nl = __builtin_addressof(nl);

    // Analog nodes:
    // - vin is the exposed input node, with input impedance Rin to ground.
    // - n_div[i] are the resistor ladder nodes (0..16), with n_div[0]=gnd, n_div[16]=Vref.
    auto& vin = ::phy_engine::netlist::create_node(nl);

    std::vector<::phy_engine::model::node_t*> n_div{};
    n_div.resize(kLevels + 1);
    n_div[0] = __builtin_addressof(nl.ground_node);
    for(std::size_t i{1}; i <= kLevels; ++i)
    {
        n_div[i] = __builtin_addressof(::phy_engine::netlist::create_node(nl));
    }

    // Input impedance: a single resistor to ground (this is the only external analog "port").
    {
        auto [rin, rin_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::resistance{.r = kRin});
        (void)rin_pos;
        require(rin != nullptr, "add_model(resistance) failed");
        rin->name = ::fast_io::u8string{u8"vin"};
        require(::phy_engine::netlist::add_to_node(nl, *rin, 0, vin), "add_to_node(vin,r) failed");
        require(::phy_engine::netlist::add_to_node(nl, *rin, 1, nl.ground_node), "add_to_node(gnd,r) failed");
    }

    // Reference source for the divider ladder.
    {
        auto [vref, vref_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::VDC{.V = kVref});
        (void)vref_pos;
        require(vref != nullptr, "add_model(VDC vref) failed");
        vref->name = ::fast_io::u8string{u8"vref"};
        require(::phy_engine::netlist::add_to_node(nl, *vref, 0, *n_div[kLevels]), "add_to_node(vref,+) failed");
        require(::phy_engine::netlist::add_to_node(nl, *vref, 1, nl.ground_node), "add_to_node(vref,-) failed");
    }

    // Resistor string (Vref -> ... -> gnd), equally spaced thresholds.
    for(std::size_t i{1}; i <= kLevels; ++i)
    {
        auto [rr, rr_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::resistance{.r = kRladder});
        (void)rr_pos;
        require(rr != nullptr, "add_model(ladder resistance) failed");
        rr->name = ::fast_io::u8string{u8"ladder_r["};
        append_u8_decimal(rr->name, i - 1);
        append_u8(rr->name, u8"]");
        require(::phy_engine::netlist::add_to_node(nl, *rr, 0, *n_div[i]), "add_to_node(ladder r,0) failed");
        require(::phy_engine::netlist::add_to_node(nl, *rr, 1, *n_div[i - 1]), "add_to_node(ladder r,1) failed");
    }

    // Optional: an internal VIN source used only for PE-side simulation.
    if(with_vin_source)
    {
        auto [vsrc, vsrc_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::VDC{.V = 0.0});
        (void)vsrc_pos;
        require(vsrc != nullptr, "add_model(VDC vin_src) failed");
        vsrc->name = ::fast_io::u8string{u8"vin_src"};
        require(::phy_engine::netlist::add_to_node(nl, *vsrc, 0, vin), "add_to_node(vin_src,+) failed");
        require(::phy_engine::netlist::add_to_node(nl, *vsrc, 1, nl.ground_node), "add_to_node(vin_src,-) failed");
        r.vin_src = vsrc;
    }

    // Comparators: cmp[i] = (vin >= Vth[i]), where Vth[i] = (i+1)/16 * Vref.
    r.cmp_nodes.resize(kThresholds);
    for(std::size_t i{}; i < kThresholds; ++i)
    {
        auto& n_cmp = ::phy_engine::netlist::create_node(nl);
        r.cmp_nodes[i] = __builtin_addressof(n_cmp);

        ::phy_engine::model::comparator cmp{};
        cmp.Ll = 0.0;
        cmp.Hl = 5.0;
        auto [u, u_pos] = ::phy_engine::netlist::add_model(nl, ::std::move(cmp));
        (void)u_pos;
        require(u != nullptr, "add_model(comparator) failed");
        u->name = ::fast_io::u8string{u8"cmp["};
        append_u8_decimal(u->name, i);
        append_u8(u->name, u8"]");

        require(::phy_engine::netlist::add_to_node(nl, *u, 0, vin), "add_to_node(cmp.A) failed");           // A
        require(::phy_engine::netlist::add_to_node(nl, *u, 1, *n_div[i + 1]), "add_to_node(cmp.B) failed");  // B
        require(::phy_engine::netlist::add_to_node(nl, *u, 2, n_cmp), "add_to_node(cmp.o) failed");          // o
    }

    // Output nodes + probes (Logic Outputs in PL export).
    r.out_nodes.resize(kLevels);
    r.out_probes.resize(kLevels);
    for(std::size_t i{}; i < kLevels; ++i)
    {
        auto& n_out = ::phy_engine::netlist::create_node(nl);
        r.out_nodes[i] = __builtin_addressof(n_out);

        auto [outp, outp_pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
        (void)outp_pos;
        require(outp != nullptr, "add_model(OUTPUT probe) failed");
        outp->name = ::fast_io::u8string{u8"out["};
        append_u8_decimal(outp->name, i);
        append_u8(outp->name, u8"]");
        r.out_probes[i] = outp;

        require(::phy_engine::netlist::add_to_node(nl, *outp, 0, n_out), "add_to_node(OUTPUT) failed");
    }

    // Verilog: encode comparator thermometer bits to a 16-bit one-hot output.
    decltype(auto) verilog_src = u8R"(
module adc16_onehot(
  input  [14:0] cmp,
  output [15:0] out
);
  assign out[0] = ~cmp[0];
  assign out[1]  = cmp[0]  & ~cmp[1];
  assign out[2]  = cmp[1]  & ~cmp[2];
  assign out[3]  = cmp[2]  & ~cmp[3];
  assign out[4]  = cmp[3]  & ~cmp[4];
  assign out[5]  = cmp[4]  & ~cmp[5];
  assign out[6]  = cmp[5]  & ~cmp[6];
  assign out[7]  = cmp[6]  & ~cmp[7];
  assign out[8]  = cmp[7]  & ~cmp[8];
  assign out[9]  = cmp[8]  & ~cmp[9];
  assign out[10] = cmp[9]  & ~cmp[10];
  assign out[11] = cmp[10] & ~cmp[11];
  assign out[12] = cmp[11] & ~cmp[12];
  assign out[13] = cmp[12] & ~cmp[13];
  assign out[14] = cmp[13] & ~cmp[14];
  assign out[15] = cmp[14];
endmodule
)";

    if(use_verilog_module_runtime)
    {
        // Runtime VERILOG_MODULE model (for quick mixed-signal simulation sanity check).
        auto vm = ::phy_engine::model::make_verilog_module(verilog_src, u8"adc16_onehot");
        require(vm.design != nullptr && vm.top_instance.mod != nullptr, "make_verilog_module failed");

        auto [m, mpos] = ::phy_engine::netlist::add_model(nl, ::std::move(vm));
        (void)mpos;
        require(m != nullptr, "add_model(VERILOG_MODULE) failed");
        m->name = ::fast_io::u8string{u8"adc16_onehot"};

        // Bind pins by port name (do not assume order).
        auto pv = m->ptr->generate_pin_view();
        require(pv.size == (kThresholds + kLevels), "unexpected pin count for adc16_onehot");
        for(std::size_t pi{}; pi < pv.size; ++pi)
        {
            auto const pn_u8 = pv.pins[pi].name;
            std::string const pn(reinterpret_cast<char const*>(pn_u8.data()), pn_u8.size());
            if(auto idx = parse_bit_index(pn, "cmp"); idx && *idx < kThresholds)
            {
                require(::phy_engine::netlist::add_to_node(nl, *m, pi, *r.cmp_nodes[*idx]), "add_to_node(VERILOG_MODULE.cmp) failed");
                continue;
            }
            if(auto idx = parse_bit_index(pn, "out"); idx && *idx < kLevels)
            {
                require(::phy_engine::netlist::add_to_node(nl, *m, pi, *r.out_nodes[*idx]), "add_to_node(VERILOG_MODULE.out) failed");
                continue;
            }
            die("unexpected port name in VERILOG_MODULE pin view");
        }
    }

    if(use_verilog_synth)
    {
        // Compile + elaborate.
        auto cr = ::phy_engine::verilog::digital::compile(verilog_src);
        require(cr.errors.empty(), "verilog compile failed");
        auto design = ::phy_engine::verilog::digital::build_design(::std::move(cr));
        auto const* mod = ::phy_engine::verilog::digital::find_module(design, u8"adc16_onehot");
        require(mod != nullptr, "find_module(adc16_onehot) failed");
        auto inst = ::phy_engine::verilog::digital::elaborate(design, *mod);
        require(inst.mod != nullptr, "elaborate(adc16_onehot) failed");

        // Provide the external port node list in module port order.
        std::vector<::phy_engine::model::node_t*> ports{};
        ports.reserve(inst.mod->ports.size());
        for(auto const& p : inst.mod->ports)
        {
            std::string const pn(reinterpret_cast<char const*>(p.name.data()), p.name.size());
            if(auto idx = parse_bit_index(pn, "cmp"); idx && *idx < kThresholds)
            {
                ports.push_back(r.cmp_nodes[*idx]);
                continue;
            }
            if(auto idx = parse_bit_index(pn, "out"); idx && *idx < kLevels)
            {
                ports.push_back(r.out_nodes[*idx]);
                continue;
            }
            die("unexpected port name in synthesized module port list");
        }
        require(ports.size() == (kThresholds + kLevels), "unexpected port vector size");

        ::phy_engine::verilog::digital::pe_synth_error err{};
        ::phy_engine::verilog::digital::pe_synth_options opt{
            .allow_inout = true,
            .allow_multi_driver = true,
        };
        require(::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, inst, ports, &err, opt), "synthesize_to_pe_netlist failed");
    }

    return r;
}

void set_vdc(::phy_engine::model::model_base& mb, double v)
{
    ::phy_engine::model::variant vi{};
    vi.d = v;
    vi.type = ::phy_engine::model::variant_type::d;
    require(mb.ptr != nullptr, "set_vdc: null ptr");
    require(mb.ptr->set_attribute(0, vi), "set_vdc: set_attribute(V) failed");
}

char dn_char(::phy_engine::model::digital_node_statement_t st) noexcept
{
    using s = ::phy_engine::model::digital_node_statement_t;
    switch(st)
    {
        case s::false_state: return '0';
        case s::true_state: return '1';
        case s::indeterminate_state: return 'X';
        case s::high_impedence_state: return 'Z';
        default: return '?';
    }
}

bool check_onehot_outputs(adc_net const& adc, double vin, std::size_t expected_idx)
{
    for(std::size_t i{}; i < kLevels; ++i)
    {
        auto* n = adc.out_nodes[i];
        assert(n != nullptr);
        bool const is1 = (n->node_information.dn.state == ::phy_engine::model::digital_node_statement_t::true_state);
        bool const exp = (i == expected_idx);
        if(is1 != exp)
        {
            std::fprintf(stderr, "adc16_onehot mismatch: vin=%.9f expected_bin=%zu\n", vin, expected_idx);
            std::fprintf(stderr, "  out:");
            for(std::size_t k{}; k < kLevels; ++k)
            {
                std::fprintf(stderr, " %c", dn_char(adc.out_nodes[k]->node_information.dn.state));
            }
            std::fprintf(stderr, "\n");
            std::fprintf(stderr, "  cmp:");
            for(std::size_t k{}; k < kThresholds; ++k)
            {
                auto const st = adc.cmp_nodes[k]->node_information.dn.state;
                std::fprintf(stderr, " %c", dn_char(st));
            }
            std::fprintf(stderr, "\n");
            return false;
        }
    }
    return true;
}

int run_pe_sanity(bool use_verilog_module_runtime)
{
    ::phy_engine::circult c{};
    // DC is sufficient here: the analog ladder is static; digital updates are driven by digital_clk().
    c.set_analyze_type(::phy_engine::analyze_type::DC);
    auto& nl = c.get_netlist();

    auto adc = build_adc(nl,
                         /*with_vin_source=*/true,
                         /*use_verilog_module_runtime=*/use_verilog_module_runtime,
                         /*use_verilog_synth=*/!use_verilog_module_runtime);
    require(adc.vin_src != nullptr, "vin_src missing");

    // Sweep a few representative points (including edges).
    std::vector<double> samples{
        0.0,
        (1.0 / 16.0) * kVref - 1e-6,
        (1.0 / 16.0) * kVref + 1e-6,
        (8.0 / 16.0) * kVref,
        (15.0 / 16.0) * kVref - 1e-6,
        (15.0 / 16.0) * kVref + 1e-6,
        kVref,
    };

    for(double vin : samples)
    {
        set_vdc(*adc.vin_src, vin);

        if(!c.analyze()) { return 10; }

        // Note: VERILOG_MODULE is `before_all_clk`, but comparators are `update_table`.
        // Run two ticks so the Verilog logic sees the updated comparator outputs.
        c.digital_clk();
        c.digital_clk();

        auto const bin = expected_bin(vin);
        if(!check_onehot_outputs(adc, vin, bin)) { return 11; }
    }

    return 0;
}

}  // namespace

int main()
{
    // 1) Sanity check with runtime Verilog module.
    if(auto rc = run_pe_sanity(/*use_verilog_module_runtime=*/true); rc != 0) { return rc; }

    // 2) Synthesize Verilog to PE netlist and re-run the same checks.
    if(auto rc = run_pe_sanity(/*use_verilog_module_runtime=*/false); rc != 0) { return rc; }

    // 3) Export to PhysicsLab .sav (no internal VIN source; only one exposed resistor pin for input).
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl = c.get_netlist();

        (void)build_adc(nl,
                        /*with_vin_source=*/false,
                        /*use_verilog_module_runtime=*/false,
                        /*use_verilog_synth=*/true);

        ::phy_engine::phy_lab_wrapper::pe_to_pl::options popt{};
        popt.fixed_pos = {0.0, 0.0, 0.0};
        popt.generate_wires = true;
        popt.keep_pl_macros = true;
        popt.include_linear = true;
        popt.include_ground = true;

        popt.element_placer = [&](::phy_engine::phy_lab_wrapper::pe_to_pl::options::placement_context const& ctx) -> std::optional<position> {
            // Top: a single resistor as input interface (label "vin").
            if(ctx.pl_model_id == pl_model_id::resistor && ctx.pe_instance_name == "vin")
            {
                return position{0.0, 1.0, 0.0};
            }

            // Bottom: 16 output pins (Logic Output) in a row.
            if(ctx.pl_model_id == pl_model_id::logic_output)
            {
                auto idx = parse_bit_index(ctx.pe_instance_name, "out");
                if(!idx || *idx >= kLevels) { return std::nullopt; }
                double const x = (kLevels <= 1) ? 0.0 : (-1.0 + 2.0 * (static_cast<double>(*idx) / static_cast<double>(kLevels - 1)));
                return position{x, -1.0, 0.0};
            }

            return std::nullopt;
        };

        auto r = ::phy_engine::phy_lab_wrapper::pe_to_pl::convert(nl, popt);
        assert(!r.ex.elements().empty());

        // Keep port IO fixed (do not let auto-layout move them).
        for(auto const& e : r.ex.elements())
        {
            auto const mid = e.data().value("ModelID", "");
            if(mid == pl_model_id::resistor)
            {
                auto it = e.data().find("Label");
                if(it != e.data().end() && it->is_string() && it->get<std::string>() == "vin")
                {
                    r.ex.get_element(e.identifier()).set_participate_in_layout(false);
                }
                continue;
            }
            if(mid == pl_model_id::logic_output)
            {
                auto it = e.data().find("Label");
                if(it != e.data().end() && it->is_string())
                {
                    std::string_view const name{it->get<std::string>()};
                    if(name.starts_with("out[")) { r.ex.get_element(e.identifier()).set_participate_in_layout(false); }
                }
                continue;
            }
        }

        // Middle: hierarchical layout for the ADC core.
        {
            ::phy_engine::phy_lab_wrapper::auto_layout::options aopt{};
            aopt.layout_mode = ::phy_engine::phy_lab_wrapper::auto_layout::mode::hierarchical;
            aopt.respect_fixed_elements = true;
            aopt.small_element = {1, 1};
            aopt.big_element = {2, 2};
            aopt.step_x = 0.05;
            aopt.step_y = 0.05;
            aopt.margin_x = 1e-6;
            aopt.margin_y = 1e-6;

            auto const corner0 = position{-1.0, -0.6, 0.0};
            auto const corner1 = position{1.0, 0.6, 0.0};
            (void)::phy_engine::phy_lab_wrapper::auto_layout::layout(r.ex, corner0, corner1, 0.0, aopt);
        }

        auto const out_path = std::filesystem::path(__FILE__).parent_path() / "adc16_pe_to_pl.sav";
        r.ex.save(out_path, 2);
        assert(std::filesystem::exists(out_path));
        assert(std::filesystem::file_size(out_path) > 512);
    }

    return 0;
}
