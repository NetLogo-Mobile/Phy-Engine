#include <phy_engine/phy_engine.h>
#include <phy_engine/verilog/digital/digital.h>
#include <phy_engine/verilog/digital/pe_synth.h>

#include <phy_engine/model/models/digital/logical/input.h>
#include <phy_engine/model/models/digital/logical/output.h>
#include <phy_engine/model/models/digital/verilog_module.h>

#include <phy_engine/netlist/operation.h>

#include <phy_engine/pe_nl_fileformat/pe_nl_fileformat.h>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_set>
#include <utility>
#include <vector>

namespace
{
    using u8sv = ::fast_io::u8string_view;

    struct include_ctx
    {
        std::filesystem::path base_dir;
    };

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

    bool include_resolver_fs(void* user, u8sv path, ::fast_io::u8string& out_text) noexcept
    {
        try
        {
            auto* ctx = static_cast<include_ctx*>(user);
            std::string rel(reinterpret_cast<char const*>(path.data()), path.size());
            auto p = ctx->base_dir / rel;
            auto s = read_file_text(p);
            out_text.assign(u8sv{reinterpret_cast<char8_t const*>(s.data()), s.size()});
            return true;
        }
        catch(...)
        {
            return false;
        }
    }

    static void usage(char const* argv0)
    {
        ::fast_io::io::perr(::fast_io::err(),
                        "usage: ",
                        ::fast_io::mnp::os_c_str(argv0),
                        " OUT.penl IN.v [--top TOP_MODULE]\n"
                        "  - Stores Verilog as a VERILOG_MODULE in PE netlist, with optional IO wrappers,\n"
                        "    then exports using PE-NL (LevelDB-backed) format.\n"
                        "options:\n"
                        "  --layout file|dir                         Output layout (default: file)\n"
                        "  --mode full|structure|checkpoint          Export mode (default: full)\n"
                        "  --no-io                                  Do not generate INPUT/OUTPUT models\n"
                        "  --synth                                  Synthesize to PE primitives (no VERILOG_MODULE)\n"
                        "  -O0|-O1|-O2|-O3|-O4|-O5|-Omax|-Ocuda       PE synth optimization level (default: O0)\n"
                        "  --opt-level N                             PE synth optimization level (0..5)\n"
                        "  --opt-timeout-ms MS                        Omax: wall-clock budget (0 disables; default: 0)\n"
                        "  --opt-max-iter N                           Omax: max restarts/tries (default: 32)\n"
                        "  --opt-randomize                            Omax: enable randomized search variants (default: off)\n"
                        "  --opt-rand-seed SEED                       Omax: RNG seed (default: 1)\n"
                        "  --opt-allow-regress                        Omax: allow non-improving tries to be kept (default: off)\n"
                        "  --opt-verify                               Omax: verify candidate netlists (comb-only; default: off)\n"
                        "  --opt-verify-exact-max-inputs N            Omax: exhaustive verify threshold (default: 12)\n"
                        "  --opt-verify-rand-vectors N                Omax: random vectors when not exhaustive (default: 256)\n"
                        "  --opt-verify-seed SEED                     Omax: verify RNG seed (default: 1)\n"
                        "  --cuda-opt                                 Enable CUDA acceleration for some O3/O4/Omax passes (default: off)\n"
                        "  --cuda-device-mask MASK                    CUDA device bitmask (0 = all; e.g. 3 uses GPU0+GPU1)\n"
                        "  --cuda-min-batch N                         Minimum cone batch size before offloading (default: 1024)\n"
                        "  --opt-cost gate|weighted                   Omax: objective cost model (default: gate)\n"
                        "  --opt-weight-NOT N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-AND N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-OR N                          Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-XOR N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-XNOR N                        Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-NAND N                        Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-NOR N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-IMP N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-NIMP N                        Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-YES N                         Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-CASE_EQ N                     Omax: weighted cost (default: 1)\n"
                        "  --opt-weight-IS_UNKNOWN N                  Omax: weighted cost (default: 1)\n"
                        "  --report                                  Print PE synth report (passes/iterations/Omax summary)\n"
                        "  --qm-max-vars N                            Two-level minimize budget (0 disables; default: 10)\n"
                        "  --qm-max-gates N                           Two-level minimize budget (default: 64)\n"
                        "  --qm-max-primes N                          Two-level minimize budget (default: 4096)\n"
                        "  --qm-max-minterms N                        Two-level minimize budget (0 disables; default: 0)\n"
                        "  --resub-max-vars N                         Resub budget (0 disables; default: 6)\n"
                        "  --resub-max-gates N                        Resub budget (default: 64)\n"
                        "  --sweep-max-vars N                         Sweep budget (0 disables; default: 6)\n"
                        "  --sweep-max-gates N                        Sweep budget (default: 64)\n"
                        "  --rewrite-max-candidates N                 Rewrite budget (0 disables; default: 0)\n"
                        "  --max-total-nodes N                        Global growth guard (0 disables)\n"
                        "  --max-total-models N                       Global growth guard (0 disables)\n"
                        "  --max-total-logic-gates N                  Global growth guard (0 disables)\n"
                        "  --assume-binary-inputs                    Treat X/Z as absent in synth (default: off)\n"
                        "  --no-assume-binary-inputs                 Preserve X-propagation logic\n"
                        "  --opt-wires|--no-opt-wires                Enable/disable YES buffer elimination (default: on)\n"
                        "  --opt-mul2|--no-opt-mul2                  Enable/disable MUL2 macro recognition (default: on)\n"
                        "  --opt-adders|--no-opt-adders              Enable/disable adder macro recognition (default: on)\n"
                        "  --allow-inout                             Allow inout ports (requires --no-io)\n"
                        "  --allow-multi-driver                      Allow multi-driver digital nets\n"
                        "  --overwrite                              Overwrite existing output\n");
    }

    static std::optional<std::string> arg_after(int argc, char** argv, std::string_view flag)
    {
        for(int i = 1; i + 1 < argc; ++i)
        {
            if(std::string_view(argv[i]) == flag) { return std::string(argv[i + 1]); }
        }
        return std::nullopt;
    }

    static bool has_flag(int argc, char** argv, std::string_view flag)
    {
        for(int i = 1; i < argc; ++i)
        {
            if(std::string_view(argv[i]) == flag) { return true; }
        }
        return false;
    }

    static std::optional<std::size_t> parse_size(std::string const& s)
    {
        try
        {
            std::size_t idx{};
            auto v = std::stoull(s, &idx, 10);
            if(idx != s.size()) { return std::nullopt; }
            return static_cast<std::size_t>(v);
        }
        catch(...)
        {
            return std::nullopt;
        }
    }

    static std::optional<bool> parse_toggle(int argc, char** argv, std::string_view on_flag, std::string_view off_flag)
    {
        std::optional<bool> v{};
        for(int i = 1; i < argc; ++i)
        {
            auto const a = std::string_view(argv[i]);
            if(a == on_flag) { v = true; }
            else if(a == off_flag) { v = false; }
        }
        return v;
    }

    static std::optional<std::uint8_t> parse_opt_level(int argc, char** argv)
    {
        std::optional<std::uint8_t> lvl{};
        for(int i = 1; i < argc; ++i)
        {
            auto const a = std::string_view(argv[i]);
            if(a == "-Omax" || a == "--Omax") { lvl = 5; }
            if(a == "-Ocuda" || a == "--Ocuda") { lvl = 5; }
            if(a.size() == 3 && a[0] == '-' && a[1] == 'O')
            {
                char const d = a[2];
                if(d >= '0' && d <= '5') { lvl = static_cast<std::uint8_t>(d - '0'); }
            }
        }
        if(auto s = arg_after(argc, argv, "--opt-level"))
        {
            if(auto n = parse_size(*s); n && *n <= 5u) { lvl = static_cast<std::uint8_t>(*n); }
            else
            {
                return std::nullopt;
            }
        }
        return lvl ? lvl : std::optional<std::uint8_t>{static_cast<std::uint8_t>(0)};
    }

    static std::optional<::phy_engine::pe_nl_fileformat::storage_layout> parse_layout(std::optional<std::string> const& s)
    {
        using ::phy_engine::pe_nl_fileformat::storage_layout;
        if(!s) { return storage_layout::single_file; }
        auto const& v = *s;
        if(v == "file" || v == "single" || v == "single_file" || v == "0") { return storage_layout::single_file; }
        if(v == "dir" || v == "directory" || v == "1") { return storage_layout::directory; }
        return std::nullopt;
    }

    static std::optional<::phy_engine::pe_nl_fileformat::export_mode> parse_mode(std::optional<std::string> const& s)
    {
        using ::phy_engine::pe_nl_fileformat::export_mode;
        if(!s) { return export_mode::full; }
        auto const& v = *s;
        if(v == "full" || v == "0") { return export_mode::full; }
        if(v == "structure" || v == "structure_only" || v == "1") { return export_mode::structure_only; }
        if(v == "checkpoint" || v == "runtime" || v == "runtime_only" || v == "2") { return export_mode::runtime_only; }
        return std::nullopt;
    }

    static ::phy_engine::verilog::digital::compiled_module const* find_top_module(::phy_engine::verilog::digital::compiled_design const& d,
                                                                                  std::optional<std::string> const& top_override)
    {
        using ::phy_engine::verilog::digital::find_module;

        if(top_override)
        {
            auto u8 = ::fast_io::u8string{
                u8sv{reinterpret_cast<char8_t const*>(top_override->data()), top_override->size()}
            };
            return find_module(d, u8);
        }

        // Heuristic: choose a module that is not instantiated by any other module.
        std::unordered_set<std::string> all{};
        std::unordered_set<std::string> used{};
        all.reserve(d.modules.size());
        used.reserve(d.modules.size());

        for(auto const& m: d.modules)
        {
            all.emplace(std::string(reinterpret_cast<char const*>(m.name.data()), m.name.size()));
            for(auto const& inst: m.instances) { used.emplace(std::string(reinterpret_cast<char const*>(inst.module_name.data()), inst.module_name.size())); }
        }

        std::vector<::phy_engine::verilog::digital::compiled_module const*> candidates{};
        for(auto const& m: d.modules)
        {
            std::string nm(reinterpret_cast<char const*>(m.name.data()), m.name.size());
            if(used.find(nm) == used.end()) { candidates.push_back(&m); }
        }

        if(candidates.size() == 1) { return candidates[0]; }
        if(candidates.empty())
        {
            if(d.modules.empty()) { return nullptr; }
            return &d.modules.back();
        }

        auto* best = candidates[0];
        for(auto* m: candidates)
        {
            if(m->ports.size() > best->ports.size()) { best = m; }
        }
        return best;
    }

}  // namespace

int main(int argc, char** argv)
{
    if(argc < 3 || has_flag(argc, argv, "--help") || has_flag(argc, argv, "-h"))
    {
        usage(argv[0]);
        return (argc < 3) ? 2 : 0;
    }

    try
    {
        auto out_path = std::filesystem::path(argv[1]);
        auto in_path = std::filesystem::path(argv[2]);

        auto top_override = arg_after(argc, argv, "--top");
        auto layout = parse_layout(arg_after(argc, argv, "--layout"));
        if(!layout)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --layout value\n");
            return 10;
        }

        auto mode = parse_mode(arg_after(argc, argv, "--mode"));
        if(!mode)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --mode value\n");
            return 11;
        }

        bool const gen_io = !has_flag(argc, argv, "--no-io");
        bool const overwrite = has_flag(argc, argv, "--overwrite");
        bool const synth = has_flag(argc, argv, "--synth") || has_flag(argc, argv, "--synthesize");

        auto const opt_level = parse_opt_level(argc, argv);
        if(!opt_level)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid -O* / --opt-level\n");
            return 12;
        }

        bool assume_binary_inputs = false;
        bool opt_wires = true;
        bool opt_mul2 = true;
        bool opt_adders = true;
        if(auto v = parse_toggle(argc, argv, "--assume-binary-inputs", "--no-assume-binary-inputs")) { assume_binary_inputs = *v; }
        if(auto v = parse_toggle(argc, argv, "--opt-wires", "--no-opt-wires")) { opt_wires = *v; }
        if(auto v = parse_toggle(argc, argv, "--opt-mul2", "--no-opt-mul2")) { opt_mul2 = *v; }
        if(auto v = parse_toggle(argc, argv, "--opt-adders", "--no-opt-adders")) { opt_adders = *v; }
        bool const show_report = has_flag(argc, argv, "--report");

        // Omax / budget knobs (also affect O3 since they are per-pass/global budgets).
        std::size_t omax_timeout_ms = 0;
        if(auto s = arg_after(argc, argv, "--opt-timeout-ms"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-timeout-ms\n");
                return 12;
            }
            omax_timeout_ms = *v;
        }

        std::size_t omax_max_iter = 32;
        if(auto s = arg_after(argc, argv, "--opt-max-iter"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-max-iter\n");
                return 12;
            }
            omax_max_iter = *v;
        }

        bool const omax_randomize = has_flag(argc, argv, "--opt-randomize");
        std::uint64_t omax_rand_seed = 1;
        if(auto s = arg_after(argc, argv, "--opt-rand-seed"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-rand-seed\n");
                return 12;
            }
            omax_rand_seed = static_cast<std::uint64_t>(*v);
        }
        bool const omax_allow_regress = has_flag(argc, argv, "--opt-allow-regress");

        bool const ocuda = has_flag(argc, argv, "-Ocuda") || has_flag(argc, argv, "--Ocuda");
        bool const cuda_opt = has_flag(argc, argv, "--cuda-opt") || ocuda;
        std::uint32_t cuda_device_mask = 0;
        if(auto s = arg_after(argc, argv, "--cuda-device-mask"))
        {
            auto v = parse_size(*s);
            if(!v || *v > 0xFFFFFFFFull)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --cuda-device-mask\n");
                return 12;
            }
            cuda_device_mask = static_cast<std::uint32_t>(*v);
        }
        std::size_t cuda_min_batch = 1024;
        if(auto s = arg_after(argc, argv, "--cuda-min-batch"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --cuda-min-batch\n");
                return 12;
            }
            cuda_min_batch = *v;
        }
        else if(ocuda)
        {
            cuda_min_batch = 64;
        }

        bool const omax_verify = has_flag(argc, argv, "--opt-verify");
        std::size_t omax_verify_exact_max_inputs = 12;
        if(auto s = arg_after(argc, argv, "--opt-verify-exact-max-inputs"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-verify-exact-max-inputs\n");
                return 12;
            }
            omax_verify_exact_max_inputs = *v;
        }
        std::size_t omax_verify_random_vectors = 256;
        if(auto s = arg_after(argc, argv, "--opt-verify-rand-vectors"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-verify-rand-vectors\n");
                return 12;
            }
            omax_verify_random_vectors = *v;
        }
        std::uint64_t omax_verify_seed = 1;
        if(auto s = arg_after(argc, argv, "--opt-verify-seed"))
        {
            auto v = parse_size(*s);
            if(!v)
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-verify-seed\n");
                return 12;
            }
            omax_verify_seed = static_cast<std::uint64_t>(*v);
        }

        auto omax_cost = ::phy_engine::verilog::digital::pe_synth_options::omax_cost_model::gate_count;
        if(auto s = arg_after(argc, argv, "--opt-cost"))
        {
            if(*s == "gate" || *s == "gate_count" || *s == "0") { omax_cost = ::phy_engine::verilog::digital::pe_synth_options::omax_cost_model::gate_count; }
            else if(*s == "weighted" || *s == "weighted_gate_count" || *s == "1")
            {
                omax_cost = ::phy_engine::verilog::digital::pe_synth_options::omax_cost_model::weighted_gate_count;
            }
            else
            {
                ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-cost (use gate|weighted)\n");
                return 12;
            }
        }

        auto w_not = std::uint16_t{1};
        auto w_and = std::uint16_t{1};
        auto w_or = std::uint16_t{1};
        auto w_xor = std::uint16_t{1};
        auto w_xnor = std::uint16_t{1};
        auto w_nand = std::uint16_t{1};
        auto w_nor = std::uint16_t{1};
        auto w_imp = std::uint16_t{1};
        auto w_nimp = std::uint16_t{1};
        auto w_yes = std::uint16_t{1};
        auto w_case_eq = std::uint16_t{1};
        auto w_is_unknown = std::uint16_t{1};
        auto parse_w16 = [&](char const* flag, std::uint16_t& out) -> bool
        {
            if(auto s = arg_after(argc, argv, flag))
            {
                auto v = parse_size(*s);
                if(!v || *v > 65535u) { return false; }
                out = static_cast<std::uint16_t>(*v);
            }
            return true;
        };
        if(!parse_w16("--opt-weight-NOT", w_not) || !parse_w16("--opt-weight-AND", w_and) || !parse_w16("--opt-weight-OR", w_or) ||
           !parse_w16("--opt-weight-XOR", w_xor) || !parse_w16("--opt-weight-XNOR", w_xnor) || !parse_w16("--opt-weight-NAND", w_nand) ||
           !parse_w16("--opt-weight-NOR", w_nor) || !parse_w16("--opt-weight-IMP", w_imp) || !parse_w16("--opt-weight-NIMP", w_nimp) ||
           !parse_w16("--opt-weight-YES", w_yes) || !parse_w16("--opt-weight-CASE_EQ", w_case_eq) ||
           !parse_w16("--opt-weight-IS_UNKNOWN", w_is_unknown))
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid --opt-weight-*\n");
            return 12;
        }

        // Per-pass/global budgets.
        auto parse_budget = [&](char const* flag, std::size_t& out) -> bool
        {
            if(auto s = arg_after(argc, argv, flag))
            {
                auto v = parse_size(*s);
                if(!v) { return false; }
                out = *v;
            }
            return true;
        };
        std::size_t qm_max_vars = 10, qm_max_gates = 64, qm_max_primes = 4096, qm_max_minterms = 0;
        std::size_t resub_max_vars = 6, resub_max_gates = 64;
        std::size_t sweep_max_vars = 6, sweep_max_gates = 64;
        std::size_t rewrite_max_candidates = 0;
        std::size_t max_total_nodes = 0, max_total_models = 0, max_total_logic_gates = 0;
        if(!parse_budget("--qm-max-vars", qm_max_vars) || !parse_budget("--qm-max-gates", qm_max_gates) || !parse_budget("--qm-max-primes", qm_max_primes) ||
           !parse_budget("--qm-max-minterms", qm_max_minterms) || !parse_budget("--resub-max-vars", resub_max_vars) ||
           !parse_budget("--resub-max-gates", resub_max_gates) || !parse_budget("--sweep-max-vars", sweep_max_vars) ||
           !parse_budget("--sweep-max-gates", sweep_max_gates) || !parse_budget("--rewrite-max-candidates", rewrite_max_candidates) ||
           !parse_budget("--max-total-nodes", max_total_nodes) || !parse_budget("--max-total-models", max_total_models) ||
           !parse_budget("--max-total-logic-gates", max_total_logic_gates))
        {
            ::fast_io::io::perr(::fast_io::err(), "error: invalid budget option (expects an integer)\n");
            return 12;
        }

        if(ocuda)
        {
            // Improve optimization quality under -Ocuda by expanding bounded TT windows by default.
            if(!arg_after(argc, argv, "--resub-max-vars")) { resub_max_vars = std::max<std::size_t>(resub_max_vars, 10u); }
            if(!arg_after(argc, argv, "--sweep-max-vars")) { sweep_max_vars = std::max<std::size_t>(sweep_max_vars, 10u); }
        }
        bool const allow_inout = has_flag(argc, argv, "--allow-inout");
        bool const allow_multi_driver = has_flag(argc, argv, "--allow-multi-driver");
        if(allow_inout && gen_io)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: --allow-inout requires --no-io\n");
            return 13;
        }

        using namespace ::phy_engine;
        using namespace ::phy_engine::verilog::digital;

        auto const src_s = read_file_text(in_path);
        auto const src = u8sv{reinterpret_cast<char8_t const*>(src_s.data()), src_s.size()};

        compile_options copt{};
        include_ctx ictx{.base_dir = in_path.parent_path()};
        copt.preprocess.user = __builtin_addressof(ictx);
        copt.preprocess.include_resolver = include_resolver_fs;

        auto const in_path_s = in_path.string();
        ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] compile ", ::fast_io::mnp::os_c_str(in_path_s.c_str()), "\n");
        auto cr = ::phy_engine::verilog::digital::compile(src, copt);
        if(!cr.errors.empty())
        {
            diagnostic_options dop{};
            dop.filename = u8sv{reinterpret_cast<char8_t const*>(in_path_s.data()), in_path_s.size()};

            auto const diag = format_compile_errors(cr, src, dop);
            if(!diag.empty()) { ::fast_io::io::perr(::fast_io::u8err(), u8sv{diag.data(), diag.size()}); }
            if(cr.modules.empty())
            {
                ::fast_io::io::perr(::fast_io::err(), "note: no Verilog `module` was successfully parsed; check that the input is a Verilog source file\n");
            }
            return 1;
        }
        if(cr.modules.empty())
        {
            ::fast_io::io::perr(::fast_io::err(), "error: no Verilog `module` found in input: ", ::fast_io::mnp::os_c_str(in_path_s.c_str()), "\n");
            return 1;
        }

        auto design_v = ::phy_engine::verilog::digital::build_design(::std::move(cr));
        auto design = ::std::make_shared<::phy_engine::verilog::digital::compiled_design>(::std::move(design_v));

        auto const* top_mod = find_top_module(*design, top_override);
        if(top_mod == nullptr) { return 3; }

        ::fast_io::io::perr(::fast_io::u8err(),
                            u8"[verilog2penl] top=",
                            u8sv{top_mod->name.data(), top_mod->name.size()},
                            u8" (ports=",
                            top_mod->ports.size(),
                            u8")\n");

        auto top_inst = ::phy_engine::verilog::digital::elaborate(*design, *top_mod);
        if(top_inst.mod == nullptr) { return 4; }

        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::TR);
        c.get_analyze_setting().tr.t_step = 1e-9;
        c.get_analyze_setting().tr.t_stop = 1e-9;
        auto& nl = c.get_netlist();

        // Create one node per port (in port-list order).
        std::vector<::phy_engine::model::node_t*> port_nodes{};
        port_nodes.reserve(top_inst.mod->ports.size());
        for(std::size_t i{}; i < top_inst.mod->ports.size(); ++i)
        {
            auto& n = ::phy_engine::netlist::create_node(nl);
            port_nodes.push_back(__builtin_addressof(n));
        }

        // Optional IO wrappers for interactive simulation.
        if(gen_io)
        {
            for(std::size_t pi{}; pi < top_inst.mod->ports.size(); ++pi)
            {
                auto const& p = top_inst.mod->ports.index_unchecked(pi);
                if(p.dir == port_dir::input)
                {
                    auto [m, pos] =
                        ::phy_engine::netlist::add_model(nl, ::phy_engine::model::INPUT{.outputA = ::phy_engine::model::digital_node_statement_t::false_state});
                    (void)pos;
                    if(m == nullptr || m->ptr == nullptr) { return 5; }
                    m->name = p.name;
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *port_nodes[pi])) { return 6; }
                }
                else if(p.dir == port_dir::output)
                {
                    auto [m, pos] = ::phy_engine::netlist::add_model(nl, ::phy_engine::model::OUTPUT{});
                    (void)pos;
                    if(m == nullptr || m->ptr == nullptr) { return 7; }
                    m->name = p.name;
                    if(!::phy_engine::netlist::add_to_node(nl, *m, 0, *port_nodes[pi])) { return 8; }
                }
                else
                {
                    ::fast_io::io::perr(::fast_io::err(), "error: unsupported port dir (inout)\n");
                    return 9;
                }
            }
        }

        if(synth)
        {
            ::phy_engine::verilog::digital::pe_synth_error err{};
            ::phy_engine::verilog::digital::pe_synth_options opt{};
            ::phy_engine::verilog::digital::pe_synth_report rep{};
            opt.allow_inout = allow_inout;
            opt.allow_multi_driver = allow_multi_driver;
            opt.assume_binary_inputs = assume_binary_inputs;
            opt.opt_level = *opt_level;
            opt.optimize_wires = opt_wires;
            opt.optimize_mul2 = opt_mul2;
            opt.optimize_adders = opt_adders;
            opt.cuda_enable = cuda_opt;
            opt.cuda_device_mask = cuda_device_mask;
            opt.cuda_min_batch = cuda_min_batch;
            if(ocuda)
            {
                opt.decomp_var_order_tries = std::max<std::size_t>(opt.decomp_var_order_tries, 16u);
            }

            opt.omax_timeout_ms = omax_timeout_ms;
            opt.omax_max_iter = omax_max_iter;
            opt.omax_randomize = omax_randomize;
            opt.omax_rand_seed = omax_rand_seed;
            opt.omax_allow_regress = omax_allow_regress;
            opt.omax_verify = omax_verify;
            opt.omax_verify_exact_max_inputs = omax_verify_exact_max_inputs;
            opt.omax_verify_random_vectors = omax_verify_random_vectors;
            opt.omax_verify_seed = omax_verify_seed;
            opt.omax_cost = omax_cost;
            opt.omax_gate_weights.not_w = w_not;
            opt.omax_gate_weights.and_w = w_and;
            opt.omax_gate_weights.or_w = w_or;
            opt.omax_gate_weights.xor_w = w_xor;
            opt.omax_gate_weights.xnor_w = w_xnor;
            opt.omax_gate_weights.nand_w = w_nand;
            opt.omax_gate_weights.nor_w = w_nor;
            opt.omax_gate_weights.imp_w = w_imp;
            opt.omax_gate_weights.nimp_w = w_nimp;
            opt.omax_gate_weights.yes_w = w_yes;
            opt.omax_gate_weights.case_eq_w = w_case_eq;
            opt.omax_gate_weights.is_unknown_w = w_is_unknown;

            opt.qm_max_vars = qm_max_vars;
            opt.qm_max_gates = qm_max_gates;
            opt.qm_max_primes = qm_max_primes;
            opt.qm_max_minterms = qm_max_minterms;
            opt.resub_max_vars = resub_max_vars;
            opt.resub_max_gates = resub_max_gates;
            opt.sweep_max_vars = sweep_max_vars;
            opt.sweep_max_gates = sweep_max_gates;
            opt.rewrite_max_candidates = rewrite_max_candidates;
            opt.max_total_nodes = max_total_nodes;
            opt.max_total_models = max_total_models;
            opt.max_total_logic_gates = max_total_logic_gates;

            opt.report_enable = show_report;
            opt.report = show_report ? __builtin_addressof(rep) : nullptr;

            ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] synthesize_to_pe_netlist\n");
            if(!::phy_engine::verilog::digital::synthesize_to_pe_netlist(nl, top_inst, port_nodes, &err, opt))
            {
                ::fast_io::io::perr(::fast_io::u8err(), u8"error: synthesize_to_pe_netlist failed: ", u8sv{err.message.data(), err.message.size()}, u8"\n");
                return 14;
            }

            if(show_report)
            {
                ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] pe_synth report:\n");
                if(!rep.omax_summary.empty())
                {
                    ::fast_io::io::perr(::fast_io::u8err(), u8"  ", u8sv{rep.omax_summary.data(), rep.omax_summary.size()}, u8"\n");
                }
                if(!rep.iter_gate_count.empty())
                {
                    ::fast_io::io::perr(::fast_io::err(), "  iter_gates:");
                    for(auto v: rep.iter_gate_count) { ::fast_io::io::perr(::fast_io::err(), " ", v); }
                    ::fast_io::io::perr(::fast_io::err(), "\n");
                }
                for(auto const& ps: rep.passes)
                {
                    ::fast_io::io::perr(::fast_io::u8err(),
                                        u8"  pass ",
                                        ps.pass,
                                        u8": ",
                                        ps.before,
                                        u8" -> ",
                                        ps.after,
                                        u8"\n");
                }
                if(!rep.omax_best_cost.empty())
                {
                    ::fast_io::io::perr(::fast_io::err(), "  omax_best_cost:");
                    for(auto v: rep.omax_best_cost) { ::fast_io::io::perr(::fast_io::err(), " ", v); }
                    ::fast_io::io::perr(::fast_io::err(), "\n");
                }
                if(!rep.omax_best_gate_count.empty())
                {
                    ::fast_io::io::perr(::fast_io::err(), "  omax_best_gates:");
                    for(auto v: rep.omax_best_gate_count) { ::fast_io::io::perr(::fast_io::err(), " ", v); }
                    ::fast_io::io::perr(::fast_io::err(), "\n");
                }
            }
        }
        else
        {
            // Add the VERILOG_MODULE model (preserves source + supports checkpointing).
            ::phy_engine::model::VERILOG_MODULE vm{};
            vm.source = ::fast_io::u8string{src};
            vm.top = top_mod->name;
            vm.design = std::move(design);
            vm.top_instance = std::move(top_inst);

            vm.pin_name_storage.clear();
            vm.pins.clear();
            vm.pin_name_storage.reserve(vm.top_instance.mod->ports.size());
            vm.pins.reserve(vm.top_instance.mod->ports.size());
            for(auto const& p: vm.top_instance.mod->ports)
            {
                vm.pin_name_storage.push_back(p.name);
                auto const& name = vm.pin_name_storage.back_unchecked();
                vm.pins.push_back({
                    ::fast_io::u8string_view{name.data(), name.size()},
                    nullptr,
                    nullptr
                });
            }

            auto added = ::phy_engine::netlist::add_model(nl, ::std::move(vm));
            if(added.mod == nullptr || added.mod->ptr == nullptr) { return 15; }
            added.mod->name = top_mod->name;
            {
                auto const p = in_path.filename().string();
                added.mod->describe = ::fast_io::u8string{
                    u8sv{reinterpret_cast<char8_t const*>(p.data()), p.size()}
                };
            }

            for(std::size_t pi{}; pi < port_nodes.size(); ++pi)
            {
                if(!::phy_engine::netlist::add_to_node(nl, added.mod_pos, pi, *port_nodes[pi])) { return 16; }
            }
        }

        ::phy_engine::pe_nl_fileformat::save_options sopt{};
        sopt.overwrite = overwrite;
        sopt.mode = *mode;
        sopt.layout = *layout;

        ::fast_io::io::perr(::fast_io::err(), "[verilog2penl] save\n");
        auto st = ::phy_engine::pe_nl_fileformat::save(out_path, c, sopt);
        if(!st)
        {
            ::fast_io::io::perr(::fast_io::err(), "error: save failed: ", ::fast_io::mnp::os_c_str(st.message.c_str()), "\n");
            return 20;
        }

        return 0;
    }
    catch(std::exception const& e)
    {
        ::fast_io::io::perr(::fast_io::err(), "fatal: ", ::fast_io::mnp::os_c_str(e.what()), "\n");
        return 100;
    }
}
