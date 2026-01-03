#pragma once
#include <cmath>
#include <complex>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>
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

#if defined(__CUDA__) && !defined(__CUDA_ARCH__)
    #include "solver/cuda_sparse_lu.h"
#endif

namespace phy_engine
{

    struct circult
    {
    public:
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

        inline static constexpr ::std::size_t cuda_node_threshold{100000};
#if defined(__CUDA__) && !defined(__CUDA_ARCH__)
        ::phy_engine::solver::cuda_sparse_lu cuda_solver{};
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
            for(auto i: digital_update_tables.tables)
            {
                for(auto p: i->pins)
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

            for(auto const* const n: size_t_to_node_p)
            {
                x.index_unchecked(n->node_index) = n->node_information.an.voltage;
            }

            for(auto const* const b: size_t_to_branch_p)
            {
                x.index_unchecked(node_counter + b->index) = b->current;
            }

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
                                                          : (ac_setting.omega_stop - ac_setting.omega_start) /
                                                                static_cast<double>(ac_setting.points - 1)};
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

                double const ratio{(ac_setting.points == 1) ? 1.0
                                                            : ::std::pow(ac_setting.omega_stop / ac_setting.omega_start,
                                                                         1.0 / static_cast<double>(ac_setting.points - 1))};
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
            digital_update_tables.tables.clear();

            for(auto i: size_t_to_node_p) { i->node_information.an.voltage = {}; }
            node_counter = 0;
            size_t_to_node_p.clear();

            for(auto i: size_t_to_branch_p) { i->current = {}; }
            branch_counter = 0;
            size_t_to_branch_p.clear();
            mna.clear();
            has_prepare = false;
        }

        // private:
        void prepare() noexcept
        {
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
                            digital_update_tables.tables.emplace(c);
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
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
            mna.clear();

            mna.resize(node_counter
#if 0
                + static_cast<::std::size_t>(env.g_min != 0.0) // Gmin, to do
#endif
                       ,
                       branch_counter);

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

#if defined(__CUDA__) && !defined(__CUDA_ARCH__)
                if(node_counter >= cuda_node_threshold && cuda_solver.is_available())
                {
                    if(row_size > static_cast<::std::size_t>(::std::numeric_limits<int>::max())) [[unlikely]] { return false; }

                    ::std::size_t nnz_size{};
                    for(auto const& row: mna.A) { nnz_size += row.second.size(); }
                    if(nnz_size > static_cast<::std::size_t>(::std::numeric_limits<int>::max())) [[unlikely]] { return false; }

                    ::std::vector<int> csr_row_ptr(row_size + 1);
                    ::std::vector<int> csr_col_ind{};
                    ::std::vector<::std::complex<double>> csr_values{};
                    csr_col_ind.reserve(nnz_size);
                    csr_values.reserve(nnz_size);

                    ::std::size_t nnz_counter{};
                    csr_row_ptr[0] = 0;

                    auto it{mna.A.begin()};
                    auto const ed{mna.A.end()};
                    for(::std::size_t r{}; r != row_size; ++r)
                    {
                        if(it != ed && it->first == r)
                        {
                            for(auto const& [col, v]: it->second)
                            {
                                csr_col_ind.push_back(static_cast<int>(col));
                                csr_values.push_back(v);
                                ++nnz_counter;
                            }
                            ++it;
                        }
                        csr_row_ptr[r + 1] = static_cast<int>(nnz_counter);
                    }

                    ::std::vector<::std::complex<double>> b(row_size, {});
                    for(auto const& [idx, v]: mna.Z)
                    {
                        if(idx >= row_size) [[unlikely]] { return false; }
                        b[idx] = v;
                    }

                    ::std::vector<::std::complex<double>> x(row_size);

                    if(!cuda_solver.solve_csr(static_cast<int>(row_size),
                                              static_cast<int>(nnz_size),
                                              csr_row_ptr.data(),
                                              csr_col_ind.data(),
                                              csr_values.data(),
                                              b.data(),
                                              x.data())) [[unlikely]]
                    {
                        return false;
                    }

                    for(auto* const n: size_t_to_node_p) { n->node_information.an.voltage = x[n->node_index]; }
                    nl.ground_node.node_information.an.voltage = {};
                    for(auto* const b: size_t_to_branch_p)
                    {
                        b->current = x[mna.node_size + b->index];
                    }

                    return true;
                }
#endif

                // mna A matrix
                smXcd temp_A{static_cast<::Eigen::Index>(row_size), static_cast<::Eigen::Index>(row_size)};

                smXcd::IndexVector wi{temp_A.outerSize()};

                for(auto& i: mna.A) { wi(i.first) = static_cast<::std::remove_cvref_t<decltype(wi(i.first))>>(i.second.size()); }

                temp_A.reserve(wi);

                for(auto& i: mna.A)
                {
                    for(auto& j: i.second) { temp_A.insertBackUncompressed(i.first, j.first) = j.second; }
                }

                temp_A.collapseDuplicates(::Eigen::internal::scalar_sum_op<smXcd::Scalar, smXcd::Scalar>());

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
                for(auto* const b: size_t_to_branch_p)
                {
                    b->current = X[mna.node_size + b->index];
                }
            }

            return true;
        }
    };

}  // namespace phy_engine
