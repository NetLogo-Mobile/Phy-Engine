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
#include "solver/integral_corrector_gear.h"
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
        double t_time{};
        double t_step{};

        ::phy_engine::MNA::MNA mna{};
        double m_currentOmega{};  // AC

        ::std::size_t node_counter{};
        ::std::size_t branch_counter{};

        ::std::size_t _lastFixRow{static_cast<::std::size_t>(-1)};  // Gmin Optimizer

        ::fast_io::vector<::phy_engine::model::node_t*> size_t_to_node_p{};
        ::fast_io::vector<::phy_engine::model::branch*> size_t_to_branch_p{};

        ::phy_engine::solver::integral_corrector_gear icg{};

        ::phy_engine::MNA::sparse_complex_vector last_X{};
        ::phy_engine::MNA::sparse_complex_vector last_Z{};

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

                    /* Rotate state queue */

                    for(auto& i: nl.models)
                    {
                        for(auto c{i.begin}; c != i.curr; ++c)
                        {
                            if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                            auto const [integral_X, integral_Y, integral_size]{c->ptr->generate_integral_history_view()};

                            if(integral_size == 0) { continue; }
                            for(auto ci{integral_X}; ci != integral_X + integral_size; ++ci) { ci->push(); }
                            for(auto ci{integral_Y}; ci != integral_Y + integral_size; ++ci) { ci->push(); }
                        }
                    }

                    if(!solve()) [[unlikely]] { return false; }

                    ::std::size_t i{};
                    for(; i < mna.node_size; ++i) { size_t_to_node_p[i]->node_information.an.voltage = mna.X_ref(i); }
                    nl.ground_node.node_information.an.voltage = {};
                    ::std::size_t c{};
                    for(; i < mna.node_size + mna.branch_size; ++i) { size_t_to_branch_p[c++]->current = mna.X_ref(i); }

                    break;
                }
                case phy_engine::TR: break;
                case phy_engine::TROP: break;
                default:
                {
                    ::fast_io::unreachable();
                }
            }
            return true;
        }

    private:
        void prepare() noexcept
        {
            m_currentOmega = 0.0;

            icg.m_integralU.clear();
            icg.m_integralJ.clear();

            // set gmin optimizer
            if(env.g_min != 0.0) { _lastFixRow = static_cast<::std::size_t>(-1); }

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
            branch_counter = 0;

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
                            if(!c->ptr->prepare_tr(icg)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
                            if(!c->ptr->prepare_trop(icg)) [[unlikely]] { ::fast_io::fast_terminate(); }
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

        bool solve() noexcept
        {
            int rc{};
            bool converged{};
            bool cont{true};
            ::std::size_t iteration{};
            constexpr ::std::size_t max_iterations{100};

            do {
                mna.clear();

                mna.resize(node_counter
#if 0
                + static_cast<::std::size_t>(env.g_min != 0.0) // Gmin, to do
#endif
                           ,
                           nl.m_numBranches);

                converged = false;

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

#ifdef PHY_ENGINE_PRINT_MARTIX
                ::std::cout << mna.A << "\n";
                ::std::cout << mna.Z << "\n";
#endif  // _DEBUG

                ::std::ranges::sort(icg.m_integralU);
                ::std::ranges::sort(icg.m_integralJ);

                // solve
                {
                    ::Eigen::SparseLU<::phy_engine::MNA::sparse_complex_matrix, ::Eigen::COLAMDOrdering<int>> solver{};
                    solver.analyzePattern(mna.A);
                    solver.factorize(mna.A);
                    if(!solver.factorizationIsOk()) [[unlikely]] { return false; }
                    mna.X = solver.solve(mna.Z);
                }

                if(at != analyze_type::TR && at != analyze_type::TROP) { break; }

                if(iteration) [[likely]] { converged = is_converged(); }
                cont = !converged;

                // copy x and z
                last_X = mna.X;
                last_Z = mna.Z;

                iteration++;
            }
            while(cont && (iteration < max_iterations));

            return true;
        }

        constexpr bool is_converged() noexcept
        {
            /*
             * Check infinity norm || x - x_1 || and norm || z - z_1 ||
             */
            for(::std::size_t i{}; i < node_counter; i++)
            {
                auto const x_i{mna.X_ref(i)};
                auto const last_x_i{last_X.coeffRef(i)};
                auto const z_i{mna.Z_ref(i)};
                auto const last_z_i{last_Z.coeffRef(i)};

                double maxX{::std::max(::std::abs(x_i), ::std::abs(last_x_i))};
                double maxZ{::std::max(::std::abs(z_i), ::std::abs(last_z_i))};

                /* U */
                double Veps{::std::abs(x_i - last_x_i)};
                if(Veps > env.V_eps_max + env.V_epsr_max * maxX) [[unlikely]] { return false; }

                /* I */
                double Ieps{std::abs(z_i - last_z_i)};
                if(Ieps > env.I_eps_max + env.I_epsr_max * maxZ) [[unlikely]] { return false; }
            }

            ::std::size_t lowerb{node_counter};
            ::std::size_t upperb{node_counter + branch_counter};

            for(::std::size_t i{lowerb}; i < upperb; i++)
            {
                auto const x_i{mna.X_ref(i)};
                auto const last_x_i{last_X.coeffRef(i)};
                auto const z_i{mna.Z_ref(i)};
                auto const last_z_i{last_Z.coeffRef(i)};

                double maxX{::std::max(::std::abs(x_i), ::std::abs(last_x_i))};
                double maxZ{::std::max(::std::abs(z_i), ::std::abs(last_z_i))};

                /* J */
                double Ieps{::std::abs(x_i - last_x_i)};
                if(Ieps > env.I_eps_max + env.I_epsr_max * maxX) [[unlikely]] { return false; }

                /* E */
                double Veps = std::abs(z_i - last_z_i);
                if(Veps > env.V_eps_max + env.V_epsr_max * maxZ) [[unlikely]] { return false; }
            }

            /*
             * Call model-specific convergence check
             */
            for(auto& i: nl.models)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->type != ::phy_engine::model::model_type::normal) [[unlikely]] { continue; }
                    c->ptr->check_convergence();
                }
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
