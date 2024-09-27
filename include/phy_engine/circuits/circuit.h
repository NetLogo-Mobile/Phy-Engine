#pragma once
#include <cstdint>
#include <utility>
#include <fast_io/fast_io_dsal/vector.h>
#ifdef PHY_ENGINE_PRINT_MARTIX
    #include <iostream>
#endif
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "environment/environment.h"
#include "../netlist/netlist.h"
#include "MNA/mna.h"
#include "analyze.h"
#include "analyzer/impl.h"
#include "digital/update_table.h"

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

        // storage

        bool has_prepare{};

        ::phy_engine::MNA::MNA mna{};

        ::std::size_t node_counter{};
        ::std::size_t branch_counter{};

        ::fast_io::vector<::phy_engine::model::node_t*> size_t_to_node_p{};
        ::fast_io::vector<::phy_engine::model::branch*> size_t_to_branch_p{};

        ::phy_engine::digital::digital_node_update_table digital_update_tables{};            // digital
        ::fast_io::vector<::phy_engine::digital::need_operate_analog_node_t> digital_out{};  // digital
        ::fast_io::vector<::phy_engine::model::model_base*> before_all_clk_digital_model{};  // digital
        ::fast_io::vector<::phy_engine::model::model_base*> after_all_clk_digital_model{};   // digital

        double tr_duration{};  // TR
        double last_step{};    // TR

        // func
        constexpr ::phy_engine::environment& get_environment() noexcept { return env; }

        constexpr ::phy_engine::netlist::netlist& get_netlist() noexcept { return nl; }

        constexpr void set_analyze_type(::phy_engine::analyze_type other) noexcept { at = other; }

        constexpr ::phy_engine::analyzer::analyzer_storage_t& get_analyze_setting() noexcept { return analyzer_setting; }

        constexpr bool analyze() noexcept
        {
            switch(at)
            {
                case ::phy_engine::OP: [[fallthrough]];
                case ::phy_engine::DC: [[fallthrough]];
                case ::phy_engine::AC: [[fallthrough]];
                case ::phy_engine::ACOP:
                {
                    prepare();

                    if(!solve()) [[unlikely]] { return false; }

                    ::std::size_t i{};
                    for(; i < mna.node_size; ++i) { size_t_to_node_p[i]->node_information.an.voltage = mna.X_ref(i); }
                    nl.ground_node.node_information.an.voltage = {};
                    ::std::size_t c{};
                    for(; i < mna.node_size + mna.branch_size; ++i) { size_t_to_branch_p[c++]->current = mna.X_ref(i); }

                    break;
                }
                case ::phy_engine::TR: [[fallthrough]];
                case ::phy_engine::TROP:
                {
                    double t_time{};
                    auto const t_step{analyzer_setting.tr.t_step};
                    auto const t_stop{analyzer_setting.tr.t_stop};

                    prepare();

                    if(last_step != t_step) [[unlikely]]
                    {
                        for(auto& i: nl.models)
                        {
                            for(auto c{i.begin}; c != i.curr; ++c)
                            {
                                if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                                if(!c->ptr->step_changed_tr(last_step, t_step)) [[unlikely]] { ::fast_io::fast_terminate(); };
                            }
                        }

                        last_step = t_step;
                    }

                    for(; t_time < t_stop; t_time += t_step)
                    {
                        if(!solve()) [[unlikely]] { return false; }

                        tr_duration += t_step;

                        ::std::size_t i{};
                        for(; i < mna.node_size; ++i) { size_t_to_node_p[i]->node_information.an.voltage = mna.X_ref(i); }
                        nl.ground_node.node_information.an.voltage = {};
                        ::std::size_t c{};
                        for(; i < mna.node_size + mna.branch_size; ++i) { size_t_to_branch_p[c++]->current = mna.X_ref(i); }
                    }
                    break;
                }
                default:
                {
                    ::fast_io::unreachable();
                }
            }
            return true;
        }

        void digital_clk() noexcept
        {
            digital_out.clear();
            for(auto i: before_all_clk_digital_model)
            {
                auto const rt{i->ptr->update_digital_clk(digital_update_tables, tr_duration, ::phy_engine::model::digital_update_method_t::before_all_clk)};
                if(rt.need_to_operate_analog_node) { digital_out.push_back(rt); }
            }

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

            for(auto i: after_all_clk_digital_model)
            {
                auto const rt{i->ptr->update_digital_clk(digital_update_tables, tr_duration, ::phy_engine::model::digital_update_method_t::after_all_clk)};
                if(rt.need_to_operate_analog_node) { digital_out.push_back(rt); }
            }
        }

        void update_digital() noexcept
        {
            if(at != ::phy_engine::analyze_type::TR && at != ::phy_engine::analyze_type::TROP) [[unlikely]] { ::fast_io::fast_terminate(); }

            digital_clk();
        }

        void reset() noexcept
        {
            tr_duration = 0.0;

            digital_out.clear();

            for(auto i: size_t_to_node_p) { i->node_information.an.voltage = {}; }
            node_counter = 0;
            size_t_to_node_p.clear();

            for(auto i: size_t_to_branch_p) { i->current = {}; }
            branch_counter = 0;
            size_t_to_branch_p.clear();
            mna.clear();
            has_prepare = false;
        }

    private:
        void prepare() noexcept
        {
            if(!has_prepare && (at == ::phy_engine::analyze_type::TR || at == ::phy_engine::analyze_type::TROP)) [[unlikely]]
            {
                last_step = analyzer_setting.tr.t_step;
            }

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
                        else  // analog
                        {
                            size_t_to_node_p.push_back_unchecked(c);
                            c->node_index = node_counter++;
                        }
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
                case ::phy_engine::ACOP: [[fallthrough]];
                case ::phy_engine::OP:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init)
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare)
                            {
                                if(!c->ptr->prepare_op()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::DC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init)
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare)
                            {
                                if(!c->ptr->prepare_dc()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::AC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init)
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare)
                            {
                                if(!c->ptr->prepare_ac()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::TR:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init)
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare)
                            {
                                if(!c->ptr->prepare_tr()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::TROP:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            if(!c->has_init)
                            {
                                if(!c->ptr->init_model()) [[unlikely]] { ::fast_io::fast_terminate(); }

                                c->has_init = true;
                            }
                            if(!has_prepare)
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

        bool solve() noexcept
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
                case ::phy_engine::OP:
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
                case ::phy_engine::DC:
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
                case ::phy_engine::ACOP: [[fallthrough]];
                case ::phy_engine::AC:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_ac(mna, analyzer_setting.dc.m_currentOmega)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::TR:
                {
                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }

                            if(!c->ptr->iterate_tr(mna, analyzer_setting.tr.t_step)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                case ::phy_engine::TROP:
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

#ifdef PHY_ENGINE_PRINT_MARTIX
            ::std::cout << mna.A << "\n";
            ::std::cout << mna.Z << "\n";
#endif  // _DEBUG

            // solve
            {
                using smXcd = ::Eigen::SparseMatrix<::std::complex<double>>;
                using svXcd = ::Eigen::SparseVector<::std::complex<double>>;

                auto const row_size{node_counter + branch_counter};

                // mna A matrix
                smXcd temp_A{static_cast<::Eigen::Index>(row_size), static_cast<::Eigen::Index>(row_size)};

                smXcd::IndexVector wi{temp_A.outerSize()};

                for(auto& i: mna.A) { wi(i.first) = i.second.size(); }

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
                ::Eigen::SparseLU<smXcd, ::Eigen::COLAMDOrdering<int>> solver{};
                solver.analyzePattern(temp_A);
                solver.factorize(temp_A);
                if(!solver.factorizationIsOk()) [[unlikely]] { return false; }
                mna.X = solver.solve(temp_Z);
            }

            return true;
        }
    };

}  // namespace phy_engine
