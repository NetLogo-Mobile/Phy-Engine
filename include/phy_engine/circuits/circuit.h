#pragma once
#include <cmath>
#include <complex>
#include <cstdint>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <utility>
#include <fast_io/fast_io_dsal/vector.h>

#ifdef PHY_ENGINE_USE_MKL
    #define EIGEN_USE_MKL_ALL
#endif
#include <Eigen/Dense>
#include <Eigen/Sparse>
#ifdef PHY_ENGINE_USE_MKL
    #include <Eigen/PardisoSupport>
#endif

#include "environment/environment.h"
#include "../netlist/netlist.h"
#include "MNA/mna.h"
#include "analyze.h"
#include "analyzer/impl.h"
#include "digital/update_table.h"

#if (defined(__CUDA__) || defined(__CUDACC__) || defined(__NVCC__)) && !defined(__CUDA_ARCH__)
    #include "solver/cuda_sparse_lu.h"
#endif

namespace phy_engine
{
    namespace details
    {
        inline bool profile_solve_enabled() noexcept
        {
            static bool const enabled = []() noexcept
            {
                auto const* v = ::std::getenv("PHY_ENGINE_PROFILE_SOLVE");
                return v != nullptr && (*v == '1' || *v == 'y' || *v == 'Y' || *v == 't' || *v == 'T');
            }();
            return enabled;
        }

        inline bool profile_solve_validate_enabled() noexcept
        {
            static bool const enabled = []() noexcept
            {
                auto const* v = ::std::getenv("PHY_ENGINE_PROFILE_SOLVE_VALIDATE");
                return v != nullptr && (*v == '1' || *v == 'y' || *v == 'Y' || *v == 't' || *v == 'T');
            }();
            return enabled;
        }

        template <typename Clock = ::std::chrono::steady_clock>
        inline double ms_since(typename Clock::time_point start, typename Clock::time_point end) noexcept
        { return static_cast<double>(::std::chrono::duration_cast<::std::chrono::duration<double, ::std::milli>>(end - start).count()); }
    }  // namespace details

    struct circult
    {
    public:
        enum class cuda_solve_policy : ::std::uint_fast8_t
        {
            auto_select,
            force_cpu,
            force_cuda
        };

        // setting
        ::phy_engine::environment env{};
        ::phy_engine::netlist::netlist nl{};
        ::phy_engine::analyze_type at{};
        ::phy_engine::analyzer::analyzer_storage_t analyzer_setting{};

        struct ac_sweep_point
        {
            double omega{};
            ::fast_io::vector<::std::complex<double>> x{};
        };

        ::fast_io::vector<ac_sweep_point> ac_sweep_results{};

        // storage

        bool has_prepare{};

        ::phy_engine::MNA::MNA mna{};

        ::std::size_t node_counter{};
        ::std::size_t branch_counter{};

        bool mna_keep_pattern_ready{};

        ::fast_io::vector<::phy_engine::model::node_t*> size_t_to_node_p{};
        ::fast_io::vector<::phy_engine::model::branch*> size_t_to_branch_p{};

        ::fast_io::vector<::std::complex<double>> newton_prev_node{};
        ::fast_io::vector<::std::complex<double>> newton_prev_branch{};

        ::phy_engine::digital::digital_node_update_table digital_update_tables{};            // digital
        ::fast_io::vector<::phy_engine::digital::need_operate_analog_node_t> digital_out{};  // digital
        ::fast_io::vector<::phy_engine::model::model_base*> before_all_clk_digital_model{};  // digital
        ::fast_io::vector<::phy_engine::model::model_base*> after_all_clk_digital_model{};   // digital

        // solver
        using smXcd = ::Eigen::SparseMatrix<::std::complex<double>>;
        using svXcd = ::Eigen::SparseVector<::std::complex<double>>;
#ifdef PHY_ENGINE_USE_MKL
        ::Eigen::PardisoLU<smXcd> solver{};  //  large-scale hybrid circuit
#else
        ::Eigen::SparseLU<smXcd> solver{};
#endif

        inline static constexpr ::std::size_t default_cuda_node_threshold{100000};
        cuda_solve_policy cuda_policy{cuda_solve_policy::auto_select};
        ::std::size_t cuda_node_threshold{default_cuda_node_threshold};

        constexpr void set_cuda_policy(cuda_solve_policy p) noexcept { cuda_policy = p; }

        constexpr void set_cuda_node_threshold(::std::size_t n) noexcept { cuda_node_threshold = n; }

#if (defined(__CUDA__) || defined(__CUDACC__) || defined(__NVCC__)) && !defined(__CUDA_ARCH__)
        ::phy_engine::solver::cuda_sparse_lu cuda_solver{};

        struct cuda_host_cache
        {
            ::std::size_t row_size{};
            ::std::size_t nnz_size{};

            ::fast_io::vector<int> csr_row_ptr{};
            ::fast_io::vector<int> csr_col_ind{};

            ::fast_io::vector<double> csr_values_real{};
            ::fast_io::vector<::std::complex<double>> csr_values{};

            ::fast_io::vector<double> b_real{};
            ::fast_io::vector<::std::complex<double>> b{};
            ::fast_io::vector<::std::size_t> b_touched_real{};
            ::fast_io::vector<::std::size_t> b_touched{};

            ::fast_io::vector<double> x_real{};
            ::fast_io::vector<::std::complex<double>> x{};

            void clear_b_real_touched() noexcept
            {
                for(auto const idx: b_touched_real) { b_real[idx] = 0.0; }
                b_touched_real.clear();
            }

            void clear_b_touched() noexcept
            {
                for(auto const idx: b_touched) { b[idx] = {}; }
                b_touched.clear();
            }
        };

        cuda_host_cache cuda_cache{};
#endif

        double tr_duration{};  // TR
        double last_step{};    // TR

        // func
        constexpr ::phy_engine::environment& get_environment() noexcept { return env; }

        constexpr ::phy_engine::netlist::netlist& get_netlist() noexcept { return nl; }

        constexpr void set_analyze_type(::phy_engine::analyze_type other) noexcept { at = other; }

        constexpr ::phy_engine::analyzer::analyzer_storage_t& get_analyze_setting() noexcept { return analyzer_setting; }

        constexpr auto& get_ac_sweep_results() noexcept { return ac_sweep_results; }

        constexpr auto const& get_ac_sweep_results() const noexcept { return ac_sweep_results; }

        constexpr void clear_ac_sweep_results() noexcept { ac_sweep_results.clear(); }

        constexpr bool analyze() noexcept
        {
            switch(at)
            {
                case ::phy_engine::analyze_type::OP: [[fallthrough]];
                case ::phy_engine::analyze_type::DC:
                {
                    prepare();

                    if(!solve()) [[unlikely]] { return false; }

                    break;
                }
                case ::phy_engine::analyze_type::AC:
                {
                    prepare();
                    clear_ac_sweep_results();
                    // SPICE-compatible behavior: small-signal AC requires a DC operating point for non-linear devices.
                    // If the circuit contains any non-linear devices, run an OP solve first to populate their linearization
                    // (gm/gds/...) and allow models to capture bias-dependent small-signal parameters via save_op().
                    if(has_nonlinear_device())
                    {
                        auto const saved_at{at};
                        at = ::phy_engine::analyze_type::OP;
                        if(!solve()) [[unlikely]]
                        {
                            at = saved_at;
                            return false;
                        }
                        at = saved_at;
                    }
                    if(!run_ac_analysis()) [[unlikely]] { return false; }
                    break;
                }
                case ::phy_engine::analyze_type::ACOP:
                {
                    prepare();
                    clear_ac_sweep_results();

                    auto const saved_at{at};
                    at = ::phy_engine::analyze_type::OP;
                    if(!solve()) [[unlikely]]
                    {
                        at = saved_at;
                        return false;
                    }

                    at = ::phy_engine::analyze_type::AC;
                    bool const ok{run_ac_analysis()};
                    at = saved_at;
                    if(!ok) [[unlikely]] { return false; }

                    break;
                }
                case ::phy_engine::analyze_type::TR:
                {
                    auto const dt{analyzer_setting.tr.t_step};
                    if(dt <= 0.0) [[unlikely]] { return false; }

                    auto const t_stop{analyzer_setting.tr.t_stop};

                    prepare();

                    auto const end_time{tr_duration + t_stop};
                    for(; tr_duration < end_time;)
                    {
                        update_tr_step(dt);

                        auto const prev_time{tr_duration};
                        tr_duration = prev_time + dt;
                        if(!solve()) [[unlikely]]
                        {
                            tr_duration = prev_time;
                            return false;
                        }
                    }
                    break;
                }
                case ::phy_engine::analyze_type::TROP:
                {
                    auto const dt{analyzer_setting.tr.t_step};
                    if(dt <= 0.0) [[unlikely]] { return false; }

                    auto const t_stop{analyzer_setting.tr.t_stop};

                    // 1) Transient operating point at current time (capacitor open, inductor short, etc.)
                    prepare();
                    if(!solve()) [[unlikely]] { return false; }

                    // 2) Continue with normal transient from that operating point
                    auto const saved_at{at};
                    at = ::phy_engine::analyze_type::TR;

                    auto const end_time{tr_duration + t_stop};
                    for(; tr_duration < end_time;)
                    {
                        update_tr_step(dt);

                        auto const prev_time{tr_duration};
                        tr_duration = prev_time + dt;
                        if(!solve()) [[unlikely]]
                        {
                            tr_duration = prev_time;
                            at = saved_at;
                            return false;
                        }
                    }

                    at = saved_at;
                    break;
                }
                default:
                {
                    ::fast_io::unreachable();
                }
            }
            return true;
        }

        void before_digital_clk() noexcept
        {
            for(auto i: before_all_clk_digital_model)
            {
                auto const rt{i->ptr->update_digital_clk(digital_update_tables, tr_duration, ::phy_engine::model::digital_update_method_t::before_all_clk)};
                if(rt.need_to_operate_analog_node) { digital_out.push_back(rt); }
            }
        }

        void update_table_digital_clk() noexcept
        {
            // Seed the queue with nodes that must always be processed (hybrid analog/digital nodes).
            if(!digital_update_tables.always_tables.empty())
            {
                digital_update_tables.tables.insert(digital_update_tables.always_tables.begin(), digital_update_tables.always_tables.end());
            }

            // Process pending nodes until the queue is empty (combinational settle in one tick).
            // Use an iteration budget to avoid hanging on oscillating combinational loops.
            std::size_t iter_budget{10'000'000};
            while(!digital_update_tables.tables.empty())
            {
                if(iter_budget-- == 0) { break; }

                auto it = digital_update_tables.tables.begin();
                auto* node = *it;
                digital_update_tables.tables.erase(it);

                for(auto p: node->pins)
                {
                    auto model{p->model};
                    if(model->ptr->get_device_type() == ::phy_engine::model::model_device_type::digital) [[likely]]
                    {
                        auto const rt{
                            model->ptr->update_digital_clk(digital_update_tables, tr_duration, ::phy_engine::model::digital_update_method_t::update_table)};
                        if(rt.need_to_operate_analog_node) { digital_out.push_back(rt); }
                    }
                }
            }
        }

        void after_table_digital_clk() noexcept
        {
            for(auto i: after_all_clk_digital_model)
            {
                auto const rt{i->ptr->update_digital_clk(digital_update_tables, tr_duration, ::phy_engine::model::digital_update_method_t::after_all_clk)};
                if(rt.need_to_operate_analog_node) { digital_out.push_back(rt); }
            }
        }

        void digital_clk() noexcept
        {
            digital_out.clear();
            before_digital_clk();
            update_table_digital_clk();
            after_table_digital_clk();
        }

        void update_digital() noexcept
        {
            if(at != ::phy_engine::analyze_type::TR && at != ::phy_engine::analyze_type::TROP) [[unlikely]] { ::fast_io::fast_terminate(); }

            digital_clk();
        }

        void update_tr_step(double dt) noexcept
        {
            for(auto& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                    if(!c->ptr->step_changed_tr(last_step, dt)) [[unlikely]] { ::fast_io::fast_terminate(); }
                }
            }
            last_step = dt;
        }

        [[nodiscard]] ::fast_io::vector<::std::complex<double>> capture_solution_vector() const noexcept
        {
            ::fast_io::vector<::std::complex<double>> x{};
            auto const row_size{node_counter + branch_counter};
            x.resize(row_size);

            for(auto const* const n: size_t_to_node_p) { x.index_unchecked(n->node_index) = n->node_information.an.voltage; }

            for(auto const* const b: size_t_to_branch_p) { x.index_unchecked(node_counter + b->index) = b->current; }

            return x;
        }

        [[nodiscard]] bool run_ac_analysis() noexcept
        {
            auto& ac_setting{analyzer_setting.ac};
            using sweep_t = ::phy_engine::analyzer::AC::sweep_type;

            if(ac_setting.sweep == sweep_t::single || ac_setting.points <= 1) { return solve(); }

            if(ac_setting.points == 0) [[unlikely]] { return false; }
            ac_sweep_results.clear();

            if(ac_setting.sweep == sweep_t::linear)
            {
                double const step{(ac_setting.points == 1) ? 0.0
                                                           : (ac_setting.omega_stop - ac_setting.omega_start) / static_cast<double>(ac_setting.points - 1)};
                for(::std::size_t idx{}; idx < ac_setting.points; ++idx)
                {
                    ac_setting.omega = ac_setting.omega_start + step * static_cast<double>(idx);
                    if(!solve()) [[unlikely]] { return false; }
                    ac_sweep_results.push_back({ac_setting.omega, capture_solution_vector()});
                }
                return true;
            }

            if(ac_setting.sweep == sweep_t::log)
            {
                if(ac_setting.omega_start <= 0.0 || ac_setting.omega_stop <= 0.0) [[unlikely]] { return false; }

                double const ratio{(ac_setting.points == 1)
                                       ? 1.0
                                       : ::std::pow(ac_setting.omega_stop / ac_setting.omega_start, 1.0 / static_cast<double>(ac_setting.points - 1))};
                double omega{ac_setting.omega_start};
                for(::std::size_t idx{}; idx < ac_setting.points; ++idx)
                {
                    ac_setting.omega = omega;
                    if(!solve()) [[unlikely]] { return false; }
                    ac_sweep_results.push_back({ac_setting.omega, capture_solution_vector()});
                    omega *= ratio;
                }
                return true;
            }

            return solve();
        }

        [[nodiscard]] bool has_nonlinear_device() const noexcept
        {
            for(auto const& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->type != ::phy_engine::model::model_type::normal || c->ptr == nullptr) [[unlikely]] { continue; }
                    if(c->ptr->get_device_type() == ::phy_engine::model::model_device_type::non_linear) { return true; }
                }
            }
            return false;
        }

        void reset() noexcept
        {
            tr_duration = 0.0;
            last_step = 0.0;

            digital_out.clear();
            digital_update_tables.always_tables.clear();
            digital_update_tables.tables.clear();

            for(auto i: size_t_to_node_p) { i->node_information.an.voltage = {}; }
            node_counter = 0;
            size_t_to_node_p.clear();

            for(auto i: size_t_to_branch_p) { i->current = {}; }
            branch_counter = 0;
            size_t_to_branch_p.clear();
            mna.clear();
            mna_keep_pattern_ready = false;
            has_prepare = false;
        }

        // private:
        void prepare() noexcept
        {
            digital_update_tables.always_tables.clear();
            digital_update_tables.tables.clear();

            // node
            node_counter = 0;

            size_t_to_node_p.clear();
            auto all_node_size{nl.nodes.size() * ::phy_engine::netlist::details::netlist_node_block::chunk_module_size};
            // if(nl.ground_node.num_of_analog_node != 0) { ++all_node_size; }
            size_t_to_node_p.reserve(all_node_size);

            for(auto& i: nl.nodes)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->num_of_analog_node == 0)
                    {
                        if(!c->pins.empty())  // digital
                        {
                            if(!has_prepare) [[unlikely]] { c->node_information.dn.state = ::phy_engine::model::digital_node_statement_t::X; }
                        }
                    }
                    else
                    {
                        if(c->num_of_analog_node != c->pins.size())  // hybrid
                        {
                            digital_update_tables.always_tables.emplace(c);
                        }

                        // analog
                        size_t_to_node_p.push_back_unchecked(c);
                        c->node_index = node_counter++;
                    }
                }
            }

            nl.ground_node.node_index = SIZE_MAX;

            // count branch and internal node
            branch_counter = digital_out.size();
            size_t_to_branch_p.clear();

            before_all_clk_digital_model.clear();
            after_all_clk_digital_model.clear();

            for(auto& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                    // pin
                    auto const pin_view{c->ptr->generate_pin_view()};
                    for(auto pin_c{pin_view.pins}; pin_c != pin_view.pins + pin_view.size; ++pin_c) { pin_c->model = c; }

                    // branch
                    auto const branch_view{c->ptr->generate_branch_view()};
                    for(auto branch_c{branch_view.branches}; branch_c != branch_view.branches + branch_view.size; ++branch_c)
                    {

                        size_t_to_branch_p.push_back(branch_c);
                        branch_c->index = branch_counter++;
                    }

                    // internal node
                    auto const internal_node_view{c->ptr->generate_internal_node_view()};
                    for(auto in_c{internal_node_view.nodes}; in_c != internal_node_view.nodes + internal_node_view.size; ++in_c)
                    {

                        size_t_to_node_p.push_back(in_c);
                        in_c->node_index = node_counter++;
                    }

                    if(c->ptr->get_device_type() == ::phy_engine::model::model_device_type::digital)
                    {
                        auto const method{c->ptr->get_digital_update_method()};
                        if(static_cast<::phy_engine::model::digital_update_method_t>(
                               static_cast<::std::uint_fast8_t>(method) &
                               static_cast<::std::uint_fast8_t>(::phy_engine::model::digital_update_method_t::before_all_clk)) ==
                           ::phy_engine::model::digital_update_method_t::before_all_clk)
                        {
                            before_all_clk_digital_model.push_back(c);
                        }
                        else if(static_cast<::phy_engine::model::digital_update_method_t>(
                                    static_cast<::std::uint_fast8_t>(method) &
                                    static_cast<::std::uint_fast8_t>(::phy_engine::model::digital_update_method_t::after_all_clk)) ==
                                ::phy_engine::model::digital_update_method_t::after_all_clk)
                        {
                            after_all_clk_digital_model.push_back(c);
                        }
                    }
                }
            }

            // clear mna (set zero)
            mna.clear();
            mna_keep_pattern_ready = false;

            mna.resize(node_counter
#if 0
                + static_cast<::std::size_t>(env.g_min != 0.0) // Gmin, to do
#endif
                       ,
                       branch_counter);

            // prepare MNA
            switch(at)
            {
                case ::phy_engine::analyze_type::ACOP: [[fallthrough]];
                case ::phy_engine::analyze_type::OP:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init) [[unlikely]]
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare) [[unlikely]]
                            {
                                if(!c->ptr->prepare_op()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            // Propagate global TNOM (nominal temperature) when the model exposes "tnom".
                            // This matches common SPICE semantics where .options TNOM applies across models.
                            if(env.norm_temperature != 27.0)
                            {
                                auto const ascii_ieq = [](char a, char b) constexpr noexcept
                                {
                                    auto const ua = static_cast<unsigned char>(a);
                                    auto const ub = static_cast<unsigned char>(b);
                                    auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
                                    auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
                                    return la == lb;
                                };
                                auto const name_is_tnom = [&](::fast_io::u8string_view n) constexpr noexcept
                                {
                                    if(n.size() != 4) { return false; }
                                    auto const* p = reinterpret_cast<char const*>(n.data());
                                    return ascii_ieq(p[0], 't') && ascii_ieq(p[1], 'n') && ascii_ieq(p[2], 'o') && ascii_ieq(p[3], 'm');
                                };
                                constexpr ::std::size_t kMaxScan{512};
                                constexpr ::std::size_t kMaxConsecutiveEmpty{64};
                                ::std::size_t empty_run{};
                                bool seen_any{};
                                for(::std::size_t idx{}; idx < kMaxScan; ++idx)
                                {
                                    auto const n = c->ptr->get_attribute_name(idx);
                                    if(n.empty())
                                    {
                                        if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                                        continue;
                                    }
                                    seen_any = true;
                                    empty_run = 0;
                                    if(!name_is_tnom(n)) { continue; }
                                    // Do not override a per-model TNOM if the user has explicitly set it away from 27C.
                                    auto const cur = c->ptr->get_attribute(idx);
                                    if(cur.type == ::phy_engine::model::variant_type::d && ::std::abs(cur.d - 27.0) > 1e-12) { break; }
                                    (void)c->ptr->set_attribute(idx, {.d{env.norm_temperature}, .type{::phy_engine::model::variant_type::d}});
                                    break;
                                }
                            }
                            if(!c->ptr->load_temperature(env.temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::DC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init) [[unlikely]]
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare) [[unlikely]]
                            {
                                if(!c->ptr->prepare_dc()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(env.norm_temperature != 27.0)
                            {
                                auto const ascii_ieq = [](char a, char b) constexpr noexcept
                                {
                                    auto const ua = static_cast<unsigned char>(a);
                                    auto const ub = static_cast<unsigned char>(b);
                                    auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
                                    auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
                                    return la == lb;
                                };
                                auto const name_is_tnom = [&](::fast_io::u8string_view n) constexpr noexcept
                                {
                                    if(n.size() != 4) { return false; }
                                    auto const* p = reinterpret_cast<char const*>(n.data());
                                    return ascii_ieq(p[0], 't') && ascii_ieq(p[1], 'n') && ascii_ieq(p[2], 'o') && ascii_ieq(p[3], 'm');
                                };
                                constexpr ::std::size_t kMaxScan{512};
                                constexpr ::std::size_t kMaxConsecutiveEmpty{64};
                                ::std::size_t empty_run{};
                                bool seen_any{};
                                for(::std::size_t idx{}; idx < kMaxScan; ++idx)
                                {
                                    auto const n = c->ptr->get_attribute_name(idx);
                                    if(n.empty())
                                    {
                                        if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                                        continue;
                                    }
                                    seen_any = true;
                                    empty_run = 0;
                                    if(!name_is_tnom(n)) { continue; }
                                    auto const cur = c->ptr->get_attribute(idx);
                                    if(cur.type == ::phy_engine::model::variant_type::d && ::std::abs(cur.d - 27.0) > 1e-12) { break; }
                                    (void)c->ptr->set_attribute(idx, {.d{env.norm_temperature}, .type{::phy_engine::model::variant_type::d}});
                                    break;
                                }
                            }
                            if(!c->ptr->load_temperature(env.temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::AC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init) [[unlikely]]
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare) [[unlikely]]
                            {
                                if(!c->ptr->prepare_ac()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(env.norm_temperature != 27.0)
                            {
                                auto const ascii_ieq = [](char a, char b) constexpr noexcept
                                {
                                    auto const ua = static_cast<unsigned char>(a);
                                    auto const ub = static_cast<unsigned char>(b);
                                    auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
                                    auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
                                    return la == lb;
                                };
                                auto const name_is_tnom = [&](::fast_io::u8string_view n) constexpr noexcept
                                {
                                    if(n.size() != 4) { return false; }
                                    auto const* p = reinterpret_cast<char const*>(n.data());
                                    return ascii_ieq(p[0], 't') && ascii_ieq(p[1], 'n') && ascii_ieq(p[2], 'o') && ascii_ieq(p[3], 'm');
                                };
                                constexpr ::std::size_t kMaxScan{512};
                                constexpr ::std::size_t kMaxConsecutiveEmpty{64};
                                ::std::size_t empty_run{};
                                bool seen_any{};
                                for(::std::size_t idx{}; idx < kMaxScan; ++idx)
                                {
                                    auto const n = c->ptr->get_attribute_name(idx);
                                    if(n.empty())
                                    {
                                        if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                                        continue;
                                    }
                                    seen_any = true;
                                    empty_run = 0;
                                    if(!name_is_tnom(n)) { continue; }
                                    auto const cur = c->ptr->get_attribute(idx);
                                    if(cur.type == ::phy_engine::model::variant_type::d && ::std::abs(cur.d - 27.0) > 1e-12) { break; }
                                    (void)c->ptr->set_attribute(idx, {.d{env.norm_temperature}, .type{::phy_engine::model::variant_type::d}});
                                    break;
                                }
                            }
                            if(!c->ptr->load_temperature(env.temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::TR:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init) [[unlikely]]
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare) [[unlikely]]
                            {
                                if(!c->ptr->prepare_tr()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(env.norm_temperature != 27.0)
                            {
                                auto const ascii_ieq = [](char a, char b) constexpr noexcept
                                {
                                    auto const ua = static_cast<unsigned char>(a);
                                    auto const ub = static_cast<unsigned char>(b);
                                    auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
                                    auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
                                    return la == lb;
                                };
                                auto const name_is_tnom = [&](::fast_io::u8string_view n) constexpr noexcept
                                {
                                    if(n.size() != 4) { return false; }
                                    auto const* p = reinterpret_cast<char const*>(n.data());
                                    return ascii_ieq(p[0], 't') && ascii_ieq(p[1], 'n') && ascii_ieq(p[2], 'o') && ascii_ieq(p[3], 'm');
                                };
                                constexpr ::std::size_t kMaxScan{512};
                                constexpr ::std::size_t kMaxConsecutiveEmpty{64};
                                ::std::size_t empty_run{};
                                bool seen_any{};
                                for(::std::size_t idx{}; idx < kMaxScan; ++idx)
                                {
                                    auto const n = c->ptr->get_attribute_name(idx);
                                    if(n.empty())
                                    {
                                        if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                                        continue;
                                    }
                                    seen_any = true;
                                    empty_run = 0;
                                    if(!name_is_tnom(n)) { continue; }
                                    auto const cur = c->ptr->get_attribute(idx);
                                    if(cur.type == ::phy_engine::model::variant_type::d && ::std::abs(cur.d - 27.0) > 1e-12) { break; }
                                    (void)c->ptr->set_attribute(idx, {.d{env.norm_temperature}, .type{::phy_engine::model::variant_type::d}});
                                    break;
                                }
                            }
                            if(!c->ptr->load_temperature(env.temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::TROP:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init) [[unlikely]]
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare) [[unlikely]]
                            {
                                if(!c->ptr->prepare_trop()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(env.norm_temperature != 27.0)
                            {
                                auto const ascii_ieq = [](char a, char b) constexpr noexcept
                                {
                                    auto const ua = static_cast<unsigned char>(a);
                                    auto const ub = static_cast<unsigned char>(b);
                                    auto const la = (ua >= 'A' && ua <= 'Z') ? static_cast<unsigned char>(ua + ('a' - 'A')) : ua;
                                    auto const lb = (ub >= 'A' && ub <= 'Z') ? static_cast<unsigned char>(ub + ('a' - 'A')) : ub;
                                    return la == lb;
                                };
                                auto const name_is_tnom = [&](::fast_io::u8string_view n) constexpr noexcept
                                {
                                    if(n.size() != 4) { return false; }
                                    auto const* p = reinterpret_cast<char const*>(n.data());
                                    return ascii_ieq(p[0], 't') && ascii_ieq(p[1], 'n') && ascii_ieq(p[2], 'o') && ascii_ieq(p[3], 'm');
                                };
                                constexpr ::std::size_t kMaxScan{512};
                                constexpr ::std::size_t kMaxConsecutiveEmpty{64};
                                ::std::size_t empty_run{};
                                bool seen_any{};
                                for(::std::size_t idx{}; idx < kMaxScan; ++idx)
                                {
                                    auto const n = c->ptr->get_attribute_name(idx);
                                    if(n.empty())
                                    {
                                        if(seen_any && ++empty_run >= kMaxConsecutiveEmpty) { break; }
                                        continue;
                                    }
                                    seen_any = true;
                                    empty_run = 0;
                                    if(!name_is_tnom(n)) { continue; }
                                    auto const cur = c->ptr->get_attribute(idx);
                                    if(cur.type == ::phy_engine::model::variant_type::d && ::std::abs(cur.d - 27.0) > 1e-12) { break; }
                                    (void)c->ptr->set_attribute(idx, {.d{env.norm_temperature}, .type{::phy_engine::model::variant_type::d}});
                                    break;
                                }
                            }
                            if(!c->ptr->load_temperature(env.temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                default:
                {
                    ::fast_io ::unreachable();
                }
            }

            // Gmin
            // to do
            has_prepare = true;
        }

        [[nodiscard]] bool solve() noexcept
        {
            if(at == ::phy_engine::analyze_type::AC) { return solve_once(); }

            if(!has_nonlinear_device()) { return solve_once(); }

            constexpr ::std::size_t max_iter{64};

            double const v_abstol{env.V_eps_max > 0.0 ? env.V_eps_max : 1e-6};
            double const v_reltol{env.V_epsr_max > 0.0 ? env.V_epsr_max : 1e-3};
            double const i_abstol{env.I_eps_max > 0.0 ? env.I_eps_max : 1e-12};
            double const i_reltol{env.I_epsr_max > 0.0 ? env.I_epsr_max : v_reltol};

            newton_prev_node.resize(node_counter);
            newton_prev_branch.resize(size_t_to_branch_p.size());

            for(::std::size_t iter{}; iter < max_iter; ++iter)
            {
                for(::std::size_t i{}; i < node_counter; ++i)
                {
                    newton_prev_node.index_unchecked(i) = size_t_to_node_p.index_unchecked(i)->node_information.an.voltage;
                }
                for(::std::size_t i{}; i < size_t_to_branch_p.size(); ++i)
                {
                    newton_prev_branch.index_unchecked(i) = size_t_to_branch_p.index_unchecked(i)->current;
                }

                if(!solve_once()) [[unlikely]] { return false; }

                bool converged{true};

                for(::std::size_t i{}; i < node_counter; ++i)
                {
                    auto const v_new{size_t_to_node_p.index_unchecked(i)->node_information.an.voltage};
                    auto const v_old{newton_prev_node.index_unchecked(i)};
                    double const tol{v_abstol + v_reltol * ::std::max(::std::abs(v_new), ::std::abs(v_old))};
                    if(::std::abs(v_new - v_old) > tol)
                    {
                        converged = false;
                        break;
                    }
                }

                if(converged)
                {
                    for(::std::size_t i{}; i < size_t_to_branch_p.size(); ++i)
                    {
                        auto const i_new{size_t_to_branch_p.index_unchecked(i)->current};
                        auto const i_old{newton_prev_branch.index_unchecked(i)};
                        double const tol{i_abstol + i_reltol * ::std::max(::std::abs(i_new), ::std::abs(i_old))};
                        if(::std::abs(i_new - i_old) > tol)
                        {
                            converged = false;
                            break;
                        }
                    }
                }

                if(converged)
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal || c->ptr == nullptr) [[unlikely]] { continue; }
                            if(!c->ptr->check_convergence())
                            {
                                converged = false;
                                break;
                            }
                        }
                        if(!converged) { break; }
                    }
                }

                if(converged)
                {
                    if(at == ::phy_engine::analyze_type::OP || at == ::phy_engine::analyze_type::DC || at == ::phy_engine::analyze_type::TROP)
                    {
                        for(auto& i: nl.models)
                        {
                            for(auto c{i.begin}; c != i.curr; ++c)
                            {
                                if(c->type != ::phy_engine::model::model_type::normal || c->ptr == nullptr) [[unlikely]] { continue; }
                                if(!c->ptr->save_op()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                        }
                    }
                    return true;
                }
            }

            return false;
        }

        bool solve_once() noexcept
        {
            using clock = ::std::chrono::steady_clock;
            bool const prof{details::profile_solve_enabled()};
            auto const t_total0 = clock::now();

            static bool const mna_reuse_enabled = []() noexcept
            {
                auto const* v = ::std::getenv("PHY_ENGINE_MNA_REUSE");
                return v == nullptr || (*v != '0');
            }();
            bool const can_reuse_mna = mna_reuse_enabled && mna_keep_pattern_ready && mna.node_size == node_counter && mna.branch_size == branch_counter;
            if(can_reuse_mna) { mna.clear_values_keep_pattern(); }
            else
            {
                mna.clear();
            }

            mna.resize(node_counter
#if 0
                + static_cast<::std::size_t>(env.g_min != 0.0) // Gmin, to do
#endif
                       ,
                       branch_counter);

            mna.r_open = env.r_open > 0.0 ? env.r_open : 1e12;

            // setup digital
            ::std::size_t digital_branch_counter{};
            for(auto const [v, n]: digital_out)
            {
                auto const k{digital_branch_counter++};
                mna.B_ref(n->node_index, k) = 1.0;
                mna.C_ref(k, n->node_index) = 1.0;
                mna.E_ref(k) = v;
            }

            // iterate
            auto const t_stamp0 = clock::now();
            switch(at)
            {
                case ::phy_engine::analyze_type::OP:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_op(mna)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::DC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_dc(mna)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::ACOP: [[fallthrough]];
                case ::phy_engine::analyze_type::AC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_ac(mna, analyzer_setting.ac.omega)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::TR:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_tr(mna, tr_duration)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::analyze_type::TROP:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_trop(mna)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                default:
                {
                    ::fast_io::unreachable();
                }
            }
            auto const t_stamp1 = clock::now();
            mna_keep_pattern_ready = true;

            if(env.g_min != 0.0)
            {
                for(::std::size_t n{}; n < node_counter; ++n) { mna.G_ref(n, n) += env.g_min; }
            }

            // solve
            {
                auto const row_size{node_counter + branch_counter};

                if(row_size == 0)
                {
                    // no solution
                    return true;
                }

#if (defined(__CUDA__) || defined(__CUDACC__) || defined(__NVCC__)) && !defined(__CUDA_ARCH__)
                bool const want_cuda = [&]() noexcept
                {
                    switch(cuda_policy)
                    {
                        case cuda_solve_policy::force_cpu: return false;
                        case cuda_solve_policy::force_cuda: return true;
                        case cuda_solve_policy::auto_select: [[fallthrough]];
                        default: return node_counter >= cuda_node_threshold;
                    }
                }();

                if(want_cuda && cuda_solver.is_available())
                {
                    if(row_size > static_cast<::std::size_t>(::std::numeric_limits<int>::max())) [[unlikely]] { return false; }

                    ::std::size_t nnz_size{};
                    for(auto const& row: mna.A) { nnz_size += row.size(); }
                    if(nnz_size > static_cast<::std::size_t>(::std::numeric_limits<int>::max())) [[unlikely]] { return false; }

                    auto const t_csr0 = clock::now();
                    static bool const cache_enabled = []() noexcept
                    {
                        auto const* v = ::std::getenv("PHY_ENGINE_CUDA_CSR_CACHE");
                        return v == nullptr || (*v != '0');
                    }();

                    auto& cache = cuda_cache;
                    bool rebuilt_pattern{};
                    bool all_real{(at == ::phy_engine::analyze_type::DC || at == ::phy_engine::analyze_type::OP || at == ::phy_engine::analyze_type::TR ||
                                   at == ::phy_engine::analyze_type::TROP)};

                    if(!cache_enabled || cache.row_size != row_size || cache.nnz_size != nnz_size || cache.csr_row_ptr.size() != row_size + 1 ||
                       cache.csr_col_ind.size() != nnz_size)
                    {
                        rebuilt_pattern = true;
                        cache.row_size = row_size;
                        cache.nnz_size = nnz_size;
                        cache.csr_row_ptr.assign(row_size + 1, 0);
                        cache.csr_col_ind.resize(nnz_size);
                    }

                    // Ensure value buffers are allocated (no per-iter push_back/realloc).
                    if(all_real) { cache.csr_values_real.resize(nnz_size); }
                    else
                    {
                        cache.csr_values.resize(nnz_size);
                    }

                    auto const fill_values_and_validate = [&]() noexcept -> bool
                    {
                        ::std::size_t k{};
                        cache.csr_row_ptr[0] = 0;
                        for(::std::size_t r{}; r != row_size; ++r)
                        {
                            auto const& row_map = mna.A[r];
                            for(auto const& [col, v]: row_map)
                            {
                                if(k >= nnz_size) [[unlikely]] { return false; }

                                if(cache_enabled && !rebuilt_pattern)
                                {
                                    if(cache.csr_col_ind[k] != static_cast<int>(col)) { return false; }
                                }
                                else
                                {
                                    cache.csr_col_ind[k] = static_cast<int>(col);
                                }

                                if(all_real)
                                {
                                    if(v.imag() != 0.0)
                                    {
                                        all_real = false;
                                        cache.csr_values.resize(nnz_size);
                                        for(::std::size_t i{}; i < k; ++i) { cache.csr_values[i] = {cache.csr_values_real[i], 0.0}; }
                                        cache.csr_values_real.clear();
                                    }
                                    if(all_real) { cache.csr_values_real[k] = v.real(); }
                                    else
                                    {
                                        cache.csr_values[k] = v;
                                    }
                                }
                                else
                                {
                                    cache.csr_values[k] = v;
                                }

                                ++k;
                            }

                            // When caching is enabled and the pattern is reused, row nnz must match.
                            if(cache_enabled && !rebuilt_pattern)
                            {
                                if(static_cast<::std::size_t>(cache.csr_row_ptr[r + 1]) != k) { return false; }
                            }
                            else
                            {
                                cache.csr_row_ptr[r + 1] = static_cast<int>(k);
                            }
                        }

                        return k == nnz_size;
                    };

                    if(cache_enabled && !rebuilt_pattern)
                    {
                        if(!fill_values_and_validate())
                        {
                            rebuilt_pattern = true;
                            cache.csr_row_ptr.assign(row_size + 1, 0);
                            if(all_real) { cache.csr_values_real.resize(nnz_size); }
                            else
                            {
                                cache.csr_values.resize(nnz_size);
                            }
                            if(!fill_values_and_validate()) [[unlikely]] { return false; }
                        }
                    }
                    else
                    {
                        if(!fill_values_and_validate()) [[unlikely]] { return false; }
                    }

                    auto const t_rhs0 = clock::now();
                    if(all_real)
                    {
                        if(cache.b_real.size() != row_size)
                        {
                            cache.b_real.assign(row_size, 0.0);
                            cache.b_touched_real.clear();
                        }
                        else
                        {
                            cache.clear_b_real_touched();
                        }
                    }
                    else
                    {
                        if(cache.b.size() != row_size)
                        {
                            cache.b.assign(row_size, {});
                            cache.b_touched.clear();
                        }
                        else
                        {
                            cache.clear_b_touched();
                        }
                    }
                    for(auto const& [idx, v]: mna.Z)
                    {
                        if(idx >= row_size) [[unlikely]] { return false; }
                        if(all_real)
                        {
                            if(v.imag() != 0.0) { all_real = false; }
                            if(all_real)
                            {
                                cache.b_real[idx] = v.real();
                                cache.b_touched_real.push_back(idx);
                            }
                            else
                            {
                                if(cache.b.size() != row_size) { cache.b.assign(row_size, {}); }
                                cache.clear_b_touched();
                                cache.b_touched.reserve(cache.b_touched_real.size() + 8);
                                for(auto const j: cache.b_touched_real)
                                {
                                    cache.b[j] = {cache.b_real[j], 0.0};
                                    cache.b_touched.push_back(j);
                                }
                                cache.b_touched_real.clear();
                                cache.b[idx] = v;
                                cache.b_touched.push_back(idx);
                            }
                        }
                        else
                        {
                            cache.b[idx] = v;
                            cache.b_touched.push_back(idx);
                        }
                    }

                    if(all_real)
                    {
                        if(cache.x_real.size() != row_size) { cache.x_real.resize(row_size); }
                    }
                    else
                    {
                        if(cache.x.size() != row_size) { cache.x.resize(row_size); }
                    }
                    auto const t_rhs1 = clock::now();

                    ::phy_engine::solver::cuda_sparse_lu::timings cuda_t{};
                    auto const t_cuda0 = clock::now();
                    bool ok{};
                    if(all_real)
                    {
                        ok = cuda_solver.solve_csr_real(static_cast<int>(row_size),
                                                        static_cast<int>(nnz_size),
                                                        cache.csr_row_ptr.data(),
                                                        cache.csr_col_ind.data(),
                                                        cache.csr_values_real.data(),
                                                        cache.b_real.data(),
                                                        cache.x_real.data(),
                                                        cuda_t,
                                                        rebuilt_pattern);
                    }
                    else
                    {
                        ok = cuda_solver.solve_csr_timed(static_cast<int>(row_size),
                                                         static_cast<int>(nnz_size),
                                                         cache.csr_row_ptr.data(),
                                                         cache.csr_col_ind.data(),
                                                         cache.csr_values.data(),
                                                         cache.b.data(),
                                                         cache.x.data(),
                                                         cuda_t,
                                                         rebuilt_pattern);
                    }
                    auto const t_cuda1 = clock::now();

                    if(!ok) [[unlikely]] { return false; }

                    if(all_real)
                    {
                        for(auto* const n: size_t_to_node_p) { n->node_information.an.voltage = {cache.x_real[n->node_index], 0.0}; }
                        nl.ground_node.node_information.an.voltage = {};
                        for(auto* const bptr: size_t_to_branch_p) { bptr->current = {cache.x_real[mna.node_size + bptr->index], 0.0}; }
                    }
                    else
                    {
                        for(auto* const n: size_t_to_node_p) { n->node_information.an.voltage = cache.x[n->node_index]; }
                        nl.ground_node.node_information.an.voltage = {};
                        for(auto* const bptr: size_t_to_branch_p) { bptr->current = cache.x[mna.node_size + bptr->index]; }
                    }

                    if(prof)
                    {
                        double resid_max{};
                        bool has_resid{};
                        if(details::profile_solve_validate_enabled())
                        {
                            constexpr ::std::size_t samples{16};
                            auto const sample_row = [&](::std::size_t s) noexcept -> ::std::size_t
                            {
                                if(row_size <= 1 || samples <= 1) { return 0; }
                                return (row_size - 1) * s / (samples - 1);
                            };

                            if(all_real)
                            {
                                for(::std::size_t s{}; s < samples; ++s)
                                {
                                    auto const r{sample_row(s)};
                                    double acc{};
                                    for(int k{cache.csr_row_ptr[r]}; k < cache.csr_row_ptr[r + 1]; ++k)
                                    {
                                        acc += cache.csr_values_real[static_cast<::std::size_t>(k)] *
                                               cache.x_real[static_cast<::std::size_t>(cache.csr_col_ind[static_cast<::std::size_t>(k)])];
                                    }
                                    double const resid{acc - cache.b_real[r]};
                                    double const a{resid < 0.0 ? -resid : resid};
                                    if(!has_resid || a > resid_max)
                                    {
                                        resid_max = a;
                                        has_resid = true;
                                    }
                                }
                            }
                            else
                            {
                                for(::std::size_t s{}; s < samples; ++s)
                                {
                                    auto const r{sample_row(s)};
                                    ::std::complex<double> acc{};
                                    for(int k{cache.csr_row_ptr[r]}; k < cache.csr_row_ptr[r + 1]; ++k)
                                    {
                                        acc += cache.csr_values[static_cast<::std::size_t>(k)] *
                                               cache.x[static_cast<::std::size_t>(cache.csr_col_ind[static_cast<::std::size_t>(k)])];
                                    }
                                    auto const resid{acc - cache.b[r]};
                                    double const a{::std::abs(resid)};
                                    if(!has_resid || a > resid_max)
                                    {
                                        resid_max = a;
                                        has_resid = true;
                                    }
                                }
                            }
                        }

                        auto const t_total1 = clock::now();
                        if(has_resid)
                        {
                            ::fast_io::io::perr("[profile] stamp_ms=",
                                                details::ms_since<clock>(t_stamp0, t_stamp1),
                                                " n=",
                                                row_size,
                                                " nnz=",
                                                nnz_size,
                                                " csr_ms=",
                                                details::ms_since<clock>(t_csr0, t_rhs0),
                                                " rhs_ms=",
                                                details::ms_since<clock>(t_rhs0, t_rhs1),
                                                " cuda_total_ms=",
                                                details::ms_since<clock>(t_cuda0, t_cuda1),
                                                " (h2d_ms=",
                                                cuda_t.h2d_ms,
                                                " solve_ms=",
                                                cuda_t.solve_ms,
                                                " d2h_ms=",
                                                cuda_t.d2h_ms,
                                                " solve_host_ms=",
                                                cuda_t.solve_host_ms,
                                                " solve_total_host_ms=",
                                                cuda_t.solve_total_host_ms,
                                                ")",
                                                " real=",
                                                all_real ? "1" : "0",
                                                " resid_max=",
                                                resid_max,
                                                " total_ms=",
                                                details::ms_since<clock>(t_total0, t_total1),
                                                "\n");
                        }
                        else
                        {
                            ::fast_io::io::perr("[profile] stamp_ms=",
                                                details::ms_since<clock>(t_stamp0, t_stamp1),
                                                " n=",
                                                row_size,
                                                " nnz=",
                                                nnz_size,
                                                " csr_ms=",
                                                details::ms_since<clock>(t_csr0, t_rhs0),
                                                " rhs_ms=",
                                                details::ms_since<clock>(t_rhs0, t_rhs1),
                                                " cuda_total_ms=",
                                                details::ms_since<clock>(t_cuda0, t_cuda1),
                                                " (h2d_ms=",
                                                cuda_t.h2d_ms,
                                                " solve_ms=",
                                                cuda_t.solve_ms,
                                                " d2h_ms=",
                                                cuda_t.d2h_ms,
                                                " solve_host_ms=",
                                                cuda_t.solve_host_ms,
                                                " solve_total_host_ms=",
                                                cuda_t.solve_total_host_ms,
                                                ")",
                                                " real=",
                                                all_real ? "1" : "0",
                                                " total_ms=",
                                                details::ms_since<clock>(t_total0, t_total1),
                                                "\n");
                        }
                    }
                    return true;
                }
#endif

                // mna A matrix
                //
                // mna.A is stored row-indexed (row -> {col->value}). Eigen's default SparseMatrix is column-major,
                // and writing with (row,col) outer/inner would effectively transpose the system for non-symmetric A.
                // Build in row-major first, then convert to column-major for SparseLU.
                using smXcd_rm = ::Eigen::SparseMatrix<::std::complex<double>, ::Eigen::RowMajor>;
                smXcd_rm temp_A_rm{static_cast<::Eigen::Index>(row_size), static_cast<::Eigen::Index>(row_size)};

                smXcd_rm::IndexVector wi{temp_A_rm.outerSize()};
                for(::std::size_t r{}; r < row_size; ++r)
                {
                    wi(static_cast<::Eigen::Index>(r)) = static_cast<typename smXcd_rm::StorageIndex>(mna.A[r].size());
                }

                temp_A_rm.reserve(wi);

                for(::std::size_t r{}; r < row_size; ++r)
                {
                    for(auto const& [c, v]: mna.A[r]) { temp_A_rm.insertBackUncompressed(static_cast<::Eigen::Index>(r), static_cast<::Eigen::Index>(c)) = v; }
                }

                temp_A_rm.collapseDuplicates(::Eigen::internal::scalar_sum_op<smXcd_rm::Scalar, smXcd_rm::Scalar>());
                smXcd temp_A{temp_A_rm};
                temp_A.makeCompressed();

                // mna Z vector
                svXcd temp_Z{static_cast<::Eigen::Index>(row_size)};

                temp_Z.reserve(mna.Z.size());
                for(auto& i: mna.Z) { temp_Z.insertBackUnordered(i.first) = i.second; }

                // solve
                solver.compute(temp_A);
                if(!solver.factorizationIsOk()) [[unlikely]] { return false; }
                ::Eigen::VectorXcd X{solver.solve(temp_Z)};

                // storage
                for(auto* const n: size_t_to_node_p) { n->node_information.an.voltage = X[n->node_index]; }
                nl.ground_node.node_information.an.voltage = {};
                for(auto* const b: size_t_to_branch_p) { b->current = X[mna.node_size + b->index]; }
            }

            return true;
        }
    };

}  // namespace phy_engine
