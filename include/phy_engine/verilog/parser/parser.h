#pragma once

#include <cstddef>
#include <utility>

#include "../ast/module.h"
#include "../digital/digital.h"

namespace phy_engine::verilog
{
    // Legacy API kept for existing tests.
    // Current behavior: lex the file into tokens (no full AST yet) and report backtick-directives as errors.
    template <bool check_bound = false>
    inline void parser_file(::phy_engine::verilog::Verilog_module& vmod, char8_t const* begin, char8_t const* end) noexcept
    {
        vmod.syntaxes.clear();
        vmod.errors.clear();

        if constexpr(check_bound)
        {
            if(begin == nullptr || end == nullptr || end <= begin) [[unlikely]] { return; }
        }
        else
        {
            if(begin == nullptr || end == nullptr || end <= begin) { return; }
        }

        ::fast_io::u8string_view src{begin, static_cast<::std::size_t>(end - begin)};
        auto const lr{::phy_engine::verilog::digital::lex(src)};

        ::phy_engine::verilog::syntax_t syn{};
        syn.tokens.reserve(lr.tokens.size());
        for(auto const& t: lr.tokens)
        {
            if(t.kind == ::phy_engine::verilog::digital::token_kind::eof) { break; }
            auto const* b{t.text.data()};
            syn.tokens.push_back({b, b + t.text.size()});
        }
        vmod.syntaxes.push_back(::std::move(syn));

        for(auto const& e: lr.errors)
        {
            // Map to the existing error category for now.
            vmod.errors.push_back({begin + e.offset, ::phy_engine::verilog::error_type::invaild_pretreatment});
        }
    }
}  // namespace phy_engine::verilog
