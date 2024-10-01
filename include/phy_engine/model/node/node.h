#pragma once
#include <cstdint>
//#include <set>
#include <complex>

#include <absl/container/btree_set.h>

#include <fast_io/fast_io.h>
#include "../pin/pin.h"

namespace phy_engine::model
{
    struct analog_node_t
    {
        ::std::complex<double> voltage{};
    };

    enum class digital_update_method_t : ::std::uint_fast8_t
    {
        update_table = 0x00,
        before_all_clk = 0x01,
        after_all_clk = 0x02,
    };

    enum class digital_node_statement_t : ::std::uint_fast8_t
    {
        false_state = 0,           // L
        true_state = 1,            // H
        indeterminate_state = 2,   // X
        high_impedence_state = 3,  // Z

        L = false_state,
        H = true_state,
        X = indeterminate_state,
        Z = high_impedence_state,
    };

    template <::std::integral char_type>
    inline constexpr ::std::size_t print_reserve_size(::fast_io::io_reserve_type_t<char_type, digital_node_statement_t>) noexcept
    {
        return 1;
    }

    template <::std::integral char_type>
    inline constexpr char_type*
        print_reserve_define(::fast_io::io_reserve_type_t<char_type, digital_node_statement_t>, char_type* iter, digital_node_statement_t dns) noexcept
    {
        switch(dns)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state:
            {
                constexpr auto L{::fast_io::char_literal_v<u8'L', char_type>};
                *(iter++) = L;
                return iter;
            }
            case ::phy_engine::model::digital_node_statement_t::true_state:
            {
                constexpr auto H{::fast_io::char_literal_v<u8'H', char_type>};
                *(iter++) = H;
                return iter;
            }
            case ::phy_engine::model::digital_node_statement_t::indeterminate_state:
            {
                constexpr auto X{::fast_io::char_literal_v<u8'X', char_type>};
                *(iter++) = X;
                return iter;
            }
            case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
            {
                constexpr auto Z{::fast_io::char_literal_v<u8'Z', char_type>};
                *(iter++) = Z;
                return iter;
            }
            default: ::fast_io::fast_terminate();
        }
    }

    inline constexpr digital_node_statement_t operator& (digital_node_statement_t a, digital_node_statement_t b) noexcept
    {
        switch(a)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state:
            {
                return ::phy_engine::model::digital_node_statement_t::false_state;
            }
            case ::phy_engine::model::digital_node_statement_t::true_state:
            {
                switch(b)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state:
                    {
                        return ::phy_engine::model::digital_node_statement_t::false_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::true_state:
                    {
                        return ::phy_engine::model::digital_node_statement_t::true_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                    {
                        return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                    }
                    default: ::fast_io::unreachable();
                }
            }
            case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
            case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
            {
                switch(b)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state:
                    {
                        return ::phy_engine::model::digital_node_statement_t::false_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::true_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                    {
                        return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                    }
                    default: ::fast_io::unreachable();
                }
            }
            default: ::fast_io::unreachable();
        }
        return {};
    }

    inline constexpr digital_node_statement_t operator| (digital_node_statement_t a, digital_node_statement_t b) noexcept
    {
        switch(a)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state:
            {
                switch(b)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state:
                    {
                        return digital_node_statement_t::false_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::true_state:
                    {
                        return digital_node_statement_t::true_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                    {
                        return digital_node_statement_t::indeterminate_state;
                    }
                    default: ::fast_io::unreachable();
                }
            }
            case ::phy_engine::model::digital_node_statement_t::true_state:
            {
                return digital_node_statement_t::true_state;
            }
            case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
            case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
            {
                switch(b)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state:
                    {
                        return digital_node_statement_t::indeterminate_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::true_state:
                    {
                        return digital_node_statement_t::true_state;
                    }
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                    {
                        return digital_node_statement_t::indeterminate_state;
                    }
                    default: ::fast_io::unreachable();
                }
            }
            default: ::fast_io::unreachable();
        }
        return {};
    }

    inline constexpr digital_node_statement_t operator~(digital_node_statement_t a) noexcept
    {
        switch(a)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state:
            {
                return ::phy_engine::model::digital_node_statement_t::true_state;
            }
            case ::phy_engine::model::digital_node_statement_t::true_state:
            {
                return ::phy_engine::model::digital_node_statement_t::true_state;
            }
            case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
            case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
            {
                return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
            }
            default: ::fast_io::unreachable();
        }
        return {};
    }

    inline constexpr digital_node_statement_t operator^ (digital_node_statement_t a, digital_node_statement_t b) noexcept
    {
        switch(a)
        {
            case ::phy_engine::model::digital_node_statement_t::false_state: [[fallthrough]];
            case ::phy_engine::model::digital_node_statement_t::true_state:
            {
                switch(b)
                {
                    case ::phy_engine::model::digital_node_statement_t::false_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::true_state:
                    {
                        return static_cast<digital_node_statement_t>(static_cast<bool>(a) ^ static_cast<bool>(b));
                    }
                    case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
                    case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
                    {
                        return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
                    }
                    default: ::fast_io::unreachable();
                }
            }
            case ::phy_engine::model::digital_node_statement_t::indeterminate_state: [[fallthrough]];
            case ::phy_engine::model::digital_node_statement_t::high_impedence_state:
            {
                return ::phy_engine::model::digital_node_statement_t::indeterminate_state;
            }
            default: ::fast_io::unreachable();
        }
        return {};
    }

    struct digital_node_t
    {
        digital_node_statement_t state{};
    };

#if 0
    enum node_type_t : ::std::uint_fast8_t
    {
        digital = 0,
        analog = 1,
    };
#endif
    union node_information_union
    {
        digital_node_t dn;
        analog_node_t an;
    };

    struct node_t
    {
        node_information_union node_information{};
        ::absl::btree_set<::phy_engine::model::pin*> pins{};
        ::std::size_t num_of_analog_node{};

        // storage
        ::std::size_t node_index{SIZE_MAX};  // for analyze

        // func
        node_t() noexcept = default;

        // copy and disconnect form the model
        node_t(node_t const& others) noexcept : node_information{others.node_information} {}

        // copy and disconnect form the model
        node_t& operator= (node_t const& others) noexcept
        {
            node_information = others.node_information;
            pins.clear();
            return *this;
        }

        ~node_t() { clear(); }

        node_t& copy_with_model(node_t const& others) noexcept
        {
            node_information = others.node_information;
            pins = others.pins;
            num_of_analog_node = others.num_of_analog_node;
            node_index = others.node_index;
            return *this;
        }

        void destroy() noexcept 
        {
            pins.clear();
            num_of_analog_node = 0;
            node_index = SIZE_MAX;
        }

        void clear_node() noexcept
        {
            for(auto i: pins) { i->nodes = nullptr; }
            destroy();
        }

        void clear() noexcept
        {
            clear_node();
            node_information.dn.state = {};
        }
    };
}  // namespace phy_engine::model
