#pragma once
#include <cstdint>
#include <utility>
#include <fast_io/fast_io_dsal/vector.h>
#ifdef PHY_ENGINE_PRINT_MARTIX
    #include <iostream>
#endif
#include <Eigen/Sparse>
#include <Eigen/SparseLU>

#include "environment/environment.h"
#include "../netlist/netlist.h"
#include "MNA/mna.h"
#include "analyze.h"
#include "analyzer/impl.h"

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

        // func
        constexpr ::phy_engine::environment& get_environment() noexcept { return env; }

        constexpr ::phy_engine::netlist::netlist& get_netlist() noexcept { return nl; }

        constexpr void set_analyze_type(::phy_engine::analyze_type other) noexcept { at = other; }

        constexpr ::phy_engine::analyzer::analyzer_storage_t& get_analyze_setting() noexcept { return analyzer_setting; }

        constexpr bool analyze() noexcept
        {
            switch(at)
            {
                case phy_engine::OP: [[fallthrough]];
                case phy_engine::DC: [[fallthrough]];
                case phy_engine::AC: [[fallthrough]];
                case phy_engine::ACOP:
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
                case phy_engine::TR: [[fallthrough]];
                case phy_engine::TROP:
                {
                    double t_time{};
                    auto const t_step{analyzer_setting.tr.t_step};
                    auto const t_stop{analyzer_setting.tr.t_stop};
                    prepare();

                    for(; t_time < t_stop; t_time += t_step)
                    {
                        if(!solve()) [[unlikely]] { return false; }

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

        void reset() noexcept
        {
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
                    if(c->num_of_analog_node != 0) [[likely]]
                    {
                        size_t_to_node_p.push_back_unchecked(c);
                        c->node_index = node_counter++;
                    }
                }
            }

            nl.ground_node.node_index = SIZE_MAX;

            // count branch and internal node
            branch_counter = 0;
            size_t_to_branch_p.clear();

            for(auto& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
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
                ::Eigen::SparseLU<::phy_engine::MNA::sparse_complex_matrix, ::Eigen::COLAMDOrdering<int>> solver{};
                solver.analyzePattern(mna.A);
                solver.factorize(mna.A);
                if(!solver.factorizationIsOk()) [[unlikely]] { return false; }
                mna.X = solver.solve(mna.Z);
            }

            return true;
        }

        bool gmin_optimizer_singular_giag(::std::size_t curRow, ::std::size_t nRows) noexcept
        {
#if 0
            A[curRow * nRows + curRow] = m_gmin;
            ret = 0;

            if(curRow != m_lastFixRow)
            {
                if(curRow < m_netlist->getNumNodes()) { std::cout << "A large resistor is inserted between the node" << curRow << " and ground." << std::endl; }
                else { std::cout << "A tiny resistor is inserted at branch" << (curRow - m_netlist->getNumNodes()) << std::endl; }
                m_lastFixRow = curRow;
            }
#endif
            return false;
        }
    };

}  // namespace phy_engine
