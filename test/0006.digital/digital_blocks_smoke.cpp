#include <cstdint>

#include <fast_io/fast_io.h>

#include <phy_engine/phy_engine.h>

namespace
{
    using dns = ::phy_engine::model::digital_node_statement_t;
    using vt = ::phy_engine::model::variant_type;

    bool set_input(::phy_engine::model::model_base* m, dns s)
    {
        if(m == nullptr || m->ptr == nullptr) { return false; }
        ::phy_engine::model::variant vi{};
        vi.type = vt::digital;
        vi.digital = s;
        return m->ptr->set_attribute(0, vi);
    }

    ::std::uint8_t get_ui8(::phy_engine::model::model_base* m, ::std::size_t idx)
    {
        if(m == nullptr || m->ptr == nullptr) { return 0; }
        auto v = m->ptr->get_attribute(idx);
        if(v.type != vt::ui8) { return 0; }
        return static_cast<::std::uint8_t>(v.ui8);
    }

    dns get_digital(::phy_engine::model::model_base* m, ::std::size_t idx = 0)
    {
        if(m == nullptr || m->ptr == nullptr) { return dns::X; }
        auto v = m->ptr->get_attribute(idx);
        if(v.type != vt::digital) { return dns::X; }
        return v.digital;
    }
}  // namespace

int main()
{
    auto test_counter = [&]() -> bool
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl{c.get_netlist()};

        auto [clk, clk_pos]{add_model(nl, ::phy_engine::model::INPUT{.outputA = dns::L})};
        auto [ctr, ctr_pos]{add_model(nl, ::phy_engine::model::COUNTER4{})};

        auto [q3, q3_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};
        auto [q2, q2_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};
        auto [q1, q1_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};
        auto [q0, q0_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};

        auto& n_clk{create_node(nl)};
        add_to_node(nl, *clk, 0, n_clk);
        add_to_node(nl, *ctr, 4, n_clk);

        auto& n_q3{create_node(nl)};
        add_to_node(nl, *ctr, 0, n_q3);
        add_to_node(nl, *q3, 0, n_q3);

        auto& n_q2{create_node(nl)};
        add_to_node(nl, *ctr, 1, n_q2);
        add_to_node(nl, *q2, 0, n_q2);

        auto& n_q1{create_node(nl)};
        add_to_node(nl, *ctr, 2, n_q1);
        add_to_node(nl, *q1, 0, n_q1);

        auto& n_q0{create_node(nl)};
        add_to_node(nl, *ctr, 3, n_q0);
        add_to_node(nl, *q0, 0, n_q0);

        if(!c.analyze()) { return false; }

        auto tick_rising = [&]() -> bool
        {
            if(!set_input(clk, dns::L)) { return false; }
            c.digital_clk();
            if(!set_input(clk, dns::H)) { return false; }
            c.digital_clk();
            return true;
        };

        for(unsigned i = 0; i < 10; ++i)
        {
            if(!tick_rising()) { return false; }
            auto const v = get_ui8(ctr, 0) & 0x0F;
            auto const expect = static_cast<::std::uint8_t>((i + 1u) & 0x0F);
            if(v != expect) { return false; }

            auto const b3 = ((expect >> 3u) & 1u) ? dns::H : dns::L;
            auto const b2 = ((expect >> 2u) & 1u) ? dns::H : dns::L;
            auto const b1 = ((expect >> 1u) & 1u) ? dns::H : dns::L;
            auto const b0 = ((expect >> 0u) & 1u) ? dns::H : dns::L;
            if(get_digital(q3) != b3 || get_digital(q2) != b2 || get_digital(q1) != b1 || get_digital(q0) != b0) { return false; }
        }
        return true;
    };

    auto test_rng = [&]() -> bool
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl{c.get_netlist()};

        auto [rng_clk, rng_clk_pos]{add_model(nl, ::phy_engine::model::INPUT{.outputA = dns::L})};
        auto [rng, rng_pos]{add_model(nl, ::phy_engine::model::RANDOM_GENERATOR4{})};
        auto [rng_o, rng_o_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};

        auto& n_rng_clk{create_node(nl)};
        add_to_node(nl, *rng_clk, 0, n_rng_clk);
        add_to_node(nl, *rng, 4, n_rng_clk);

        auto& n_rng_q0{create_node(nl)};
        add_to_node(nl, *rng, 3, n_rng_q0); // q0 is pin 3
        add_to_node(nl, *rng_o, 0, n_rng_q0);

        if(!c.analyze()) { return false; }

        auto tick_rising = [&]() -> bool
        {
            if(!set_input(rng_clk, dns::L)) { return false; }
            c.digital_clk();
            if(!set_input(rng_clk, dns::H)) { return false; }
            c.digital_clk();
            return true;
        };

        auto last_state = get_ui8(rng, 0) & 0x0F;
        bool changed{};
        for(unsigned i = 0; i < 16; ++i)
        {
            if(!tick_rising()) { return false; }
            auto const st = get_ui8(rng, 0) & 0x0F;
            if(st != last_state) { changed = true; }
            last_state = st;
            if(get_digital(rng_o) == dns::X) { return false; }
        }
        return changed;
    };

    auto test_8bit = [&]() -> bool
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl{c.get_netlist()};

        auto [bus_src, bus_src_pos]{add_model(nl, ::phy_engine::model::EIGHT_BIT_INPUT{.value = 0xA5})};
        auto [bus_sink, bus_sink_pos]{add_model(nl, ::phy_engine::model::EIGHT_BIT_DISPLAY{})};

        for(int pin = 0; pin < 8; ++pin)
        {
            auto& node{create_node(nl)};
            add_to_node(nl, *bus_src, static_cast<::std::size_t>(pin), node);
            add_to_node(nl, *bus_sink, static_cast<::std::size_t>(pin), node);
        }

        if(!c.analyze()) { return false; }
        c.digital_clk();
        return get_ui8(bus_sink, 0) == 0xA5;
    };

    auto test_schmitt = [&]() -> bool
    {
        ::phy_engine::circult c{};
        c.set_analyze_type(::phy_engine::analyze_type::DC);
        auto& nl{c.get_netlist()};

        auto [st_i, st_i_pos]{add_model(nl, ::phy_engine::model::INPUT{.outputA = dns::L})};
        auto [st, st_pos]{add_model(nl, ::phy_engine::model::SCHMITT_TRIGGER{.inverted = true})};
        auto [st_o, st_o_pos]{add_model(nl, ::phy_engine::model::OUTPUT{})};

        auto& n_st_i{create_node(nl)};
        auto& n_st_o{create_node(nl)};
        add_to_node(nl, *st_i, 0, n_st_i);
        add_to_node(nl, *st, 0, n_st_i);
        add_to_node(nl, *st, 1, n_st_o);
        add_to_node(nl, *st_o, 0, n_st_o);

        if(!c.analyze()) { return false; }
        c.digital_clk();
        if(get_digital(st_o) != dns::H) { return false; }
        if(!set_input(st_i, dns::H)) { return false; }
        c.digital_clk();
        return get_digital(st_o) == dns::L;
    };

    if(!test_counter())
    {
        ::fast_io::io::perr("digital_blocks_smoke: COUNTER4 failed\n");
        return 1;
    }
    if(!test_rng())
    {
        ::fast_io::io::perr("digital_blocks_smoke: RANDOM_GENERATOR4 failed\n");
        return 1;
    }
    if(!test_8bit())
    {
        ::fast_io::io::perr("digital_blocks_smoke: 8bit I/O failed\n");
        return 1;
    }
    if(!test_schmitt())
    {
        ::fast_io::io::perr("digital_blocks_smoke: SCHMITT_TRIGGER failed\n");
        return 1;
    }

    return 0;
}
