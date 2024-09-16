#pragma once
#include <cstdint>
#include <utility>

#include "environment/environment.h"
#include "../netlist/netlist.h"
#include "MNA/mna.h"
#include "analyze.h"

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

        // func
        void prepare() noexcept
        {
            ::std::size_t counter{};
            for(auto& i: nl.nodes)
            {
                for(auto c{i.begin}; c != i.curr; ++c)
                {
                    if(c->num_of_analog_node != 0) [[likely]] { c->node_index = counter++; }
                }
            }

            mna.resize(counter, mna.branch_size);

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
                        }
                    }

                    break;
                }
                default:
                {
                    ::fast_io ::unreachable();
                }
            }
        }

        constexpr void iterate() noexcept
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

                            if(!c->ptr->iterate_tr(mna, t_time + t_step)) [[unlikely]] { ::fast_io::fast_terminate(); }
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
        }
    };

}  // namespace phy_engine
