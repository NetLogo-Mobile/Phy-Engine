#pragma once
#include <cstdint>
#include <utility>
#include <fast_io/fast_io_dsal/vector.h>

#include "environment/environment.h"
#include "../netlist/netlist.h"
#include "MNA/mna.h"
#include "analyze.h"
#include "solver/integral_corrector_gear.h"

namespace phy_engine
{

    struct circult
    {
    public:
        // setting
        ::phy_engine::environment env{};
        ::phy_engine::netlist::netlist nl{};
        ::phy_engine::analyze_type at{};
        double t_time{};
        double t_step{};

        // storage
        ::phy_engine::MNA::MNA mna{};
        double m_currentOmega{};  // AC

        ::std::size_t _lastFixRow{static_cast<::std::size_t>(-1)};  // Gmin Optimizer

        ::fast_io::vector<::phy_engine::model::node_t*> size_t_to_node_p{};
        ::fast_io::vector<::phy_engine::model::branch*> size_t_to_branch_p{};

        ::phy_engine::solver::integral_corrector_gear icg{};

        // func
        void prepare() noexcept
        {
            m_currentOmega = 0.0;

            // set gmin optimizer
            if(env.g_min != 0.0) { _lastFixRow = -1; }

            // node
            ::std::size_t node_counter{};

            size_t_to_node_p.clear();
            auto all_node_size{nl.nodes.size() * ::phy_engine::netlist::details::netlist_node_block::chunk_module_size};
            if(nl.ground_node.num_of_analog_node != 0) { ++all_node_size; }
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

            if(nl.ground_node.num_of_analog_node != 0)  // ground
            {
                size_t_to_node_p.push_back_unchecked(__builtin_addressof(nl.ground_node));
                nl.ground_node.node_index = node_counter++;
            }

            // clear mna (set zero)
            mna.clear();

            mna.resize(node_counter
#if 0
                + static_cast<::std::size_t>(env.g_min != 0.0) // Gmin, to do
#endif
                       ,
                       nl.m_numBranches);

            // clear
            size_t_to_branch_p.clear();
            size_t_to_branch_p.reserve(nl.m_numBranches);
            ::std::size_t branch_counter{};

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
                            if(!c->ptr->prepare_op()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }

                            auto const branch_view{c->ptr->generate_branch_view()};
                            for(auto branch_c{branch_view.branches}; branch_c != branch_view.branches + branch_view.size; ++branch_c)
                            {

                                size_t_to_branch_p.push_back_unchecked(branch_c);
                                branch_c->index = branch_counter++;
                            }
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
                            if(!c->ptr->prepare_dc()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }

                            auto const branch_view{c->ptr->generate_branch_view()};
                            for(auto branch_c{branch_view.branches}; branch_c != branch_view.branches + branch_view.size; ++branch_c)
                            {

                                size_t_to_branch_p.push_back(branch_c);
                                branch_c->index = branch_counter++;
                            }
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
                            if(!c->ptr->prepare_ac()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }

                            auto const branch_view{c->ptr->generate_branch_view()};
                            for(auto branch_c{branch_view.branches}; branch_c != branch_view.branches + branch_view.size; ++branch_c)
                            {

                                size_t_to_branch_p.push_back(branch_c);
                                branch_c->index = branch_counter++;
                            }
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
                            if(!c->ptr->prepare_tr()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }

                            auto const branch_view{c->ptr->generate_branch_view()};
                            for(auto branch_c{branch_view.branches}; branch_c != branch_view.branches + branch_view.size; ++branch_c)
                            {

                                size_t_to_branch_p.push_back(branch_c);
                                branch_c->index = branch_counter++;
                            }
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
                            if(!c->ptr->prepare_trop()) [[unlikely]] { ::fast_io::fast_terminate(); }
                            if(!c->ptr->load_temperature(env.norm_temperature)) [[unlikely]] { ::fast_io::fast_terminate(); }

                            auto const branch_view{c->ptr->generate_branch_view()};
                            for(auto branch_c{branch_view.branches}; branch_c != branch_view.branches + branch_view.size; ++branch_c)
                            {

                                size_t_to_branch_p.push_back(branch_c);
                                branch_c->index = branch_counter++;
                            }
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
        }

        constexpr void solve() noexcept
        {
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

                            if(!c->ptr->iterate_ac(mna, m_currentOmega)) [[unlikely]] { ::fast_io::fast_terminate(); }
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

                            if(!c->ptr->iterate_tr(mna, t_time + t_step, icg)) [[unlikely]] { ::fast_io::fast_terminate(); }
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

                            if(!c->ptr->iterate_trop(mna, icg)) [[unlikely]] { ::fast_io::fast_terminate(); }
                        }
                    }

                    break;
                }
                default:
                {
                    ::fast_io::unreachable();
                }
            }
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
