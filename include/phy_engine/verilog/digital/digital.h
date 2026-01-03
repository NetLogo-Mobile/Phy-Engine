#pragma once

#include <cstddef>
#include <cstdint>
#include <charconv>
#include <algorithm>
#include <limits>
#include <utility>
#include <vector>

#include <absl/container/btree_map.h>

#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/vector.h>

#include "../../model/node/node.h"

namespace phy_engine::verilog::digital
{
    // This is a deliberately small, synthesizable Verilog subset intended for use as a digital "device" in Phy-Engine.
    // Supported:
    // - `module` / `endmodule`
    // - Simple preprocessor: `define`/`undef`/`ifdef`/`ifndef`/`else`/`endif` + `NAME` macro expansion
    // - Port declarations: `input` / `output` / `inout`, scalar or vectors (`[msb:lsb]`), and `output reg`
    // - `wire` / `reg` declarations (scalar or vectors)
    // - Continuous assignment: `assign lhs = expr;`
    // - always blocks:
    //   - `always @*` / `always @(*)` as combinational (`if`/`case`/begin-end; blocking assignment `=`)
    //   - `always @(posedge/negedge clk)` as sequential (nonblocking assignment `<=`)
    // - Bit-select: `a[3]` (vector expressions are not supported; use bit-select on both sides)
    // - Small delays: `#<int>` before assignments (tick unit defined by the embedding engine)
    // - Module instantiation: named/positional port connections; vector connections require matching widths
    //
    // Not supported: hierarchical name references, part-select/concat, generate, tasks/functions, `include`, macro args, full event lists, strength,
    // multiple-driver resolution.

    using logic_t = ::phy_engine::model::digital_node_statement_t;

    inline constexpr logic_t normalize_z_to_x(logic_t v) noexcept { return v == logic_t::high_impedence_state ? logic_t::indeterminate_state : v; }

    inline constexpr bool is_unknown(logic_t v) noexcept
    {
        v = normalize_z_to_x(v);
        return v == logic_t::indeterminate_state;
    }

    inline constexpr logic_t logic_not(logic_t a) noexcept
    {
        a = normalize_z_to_x(a);
        switch(a)
        {
            case logic_t::false_state: return logic_t::true_state;
            case logic_t::true_state: return logic_t::false_state;
            case logic_t::indeterminate_state: return logic_t::indeterminate_state;
            default: return logic_t::indeterminate_state;
        }
    }

    inline constexpr logic_t logic_and(logic_t a, logic_t b) noexcept
    {
        a = normalize_z_to_x(a);
        b = normalize_z_to_x(b);
        if(a == logic_t::false_state || b == logic_t::false_state) { return logic_t::false_state; }
        if(is_unknown(a) || is_unknown(b)) { return logic_t::indeterminate_state; }
        return logic_t::true_state;
    }

    inline constexpr logic_t logic_or(logic_t a, logic_t b) noexcept
    {
        a = normalize_z_to_x(a);
        b = normalize_z_to_x(b);
        if(a == logic_t::true_state || b == logic_t::true_state) { return logic_t::true_state; }
        if(is_unknown(a) || is_unknown(b)) { return logic_t::indeterminate_state; }
        return logic_t::false_state;
    }

    inline constexpr logic_t logic_xor(logic_t a, logic_t b) noexcept
    {
        a = normalize_z_to_x(a);
        b = normalize_z_to_x(b);
        if(is_unknown(a) || is_unknown(b)) { return logic_t::indeterminate_state; }
        return static_cast<logic_t>(static_cast<bool>(a) ^ static_cast<bool>(b));
    }

    enum class token_kind : ::std::uint_fast8_t
    {
        eof,
        identifier,
        number,
        symbol
    };

    struct token
    {
        token_kind kind{};
        ::fast_io::u8string_view text{};
        ::std::size_t line{1};
        ::std::size_t column{1};
    };

    struct compile_error
    {
        ::fast_io::u8string message{};
        ::std::size_t offset{};
        ::std::size_t line{1};
        ::std::size_t column{1};
    };

    namespace details
    {
        inline constexpr bool is_space(char8_t c) noexcept { return c == u8' ' || c == u8'\t' || c == u8'\n' || c == u8'\r' || c == u8'\f' || c == u8'\v'; }

        inline constexpr bool is_digit(char8_t c) noexcept { return c >= u8'0' && c <= u8'9'; }

        inline constexpr bool is_alpha(char8_t c) noexcept { return (c >= u8'a' && c <= u8'z') || (c >= u8'A' && c <= u8'Z') || c == u8'_'; }

        inline constexpr bool is_ident_continue(char8_t c) noexcept { return is_alpha(c) || is_digit(c) || c == u8'$'; }
    }  // namespace details

    struct lex_result
    {
        ::fast_io::vector<token> tokens{};
        ::fast_io::vector<compile_error> errors{};
    };

    struct preprocess_result
    {
        ::fast_io::u8string output{};
        ::fast_io::vector<compile_error> errors{};
    };

    inline preprocess_result preprocess(::fast_io::u8string_view src) noexcept
    {
        preprocess_result out{};
        out.output.reserve(src.size());

        ::absl::btree_map<::fast_io::u8string, ::fast_io::u8string> macros{};

        struct if_frame
        {
            bool parent_active{};
            bool cond{};
            bool active{};
            bool seen_else{};
        };

        ::fast_io::vector<if_frame> ifs{};

        auto current_active = [&]() noexcept -> bool
        {
            if(ifs.empty()) { return true; }
            return ifs.back_unchecked().active;
        };

        auto parse_ident = [&](char8_t const*& p, char8_t const* end) noexcept -> ::fast_io::u8string_view
        {
            char8_t const* b{p};
            if(p >= end || !(details::is_alpha(*p) || *p == u8'$')) { return {}; }
            ++p;
            while(p < end && details::is_ident_continue(*p)) { ++p; }
            return ::fast_io::u8string_view{b, static_cast<::std::size_t>(p - b)};
        };

        char8_t const* p{src.data()};
        char8_t const* const e{src.data() + src.size()};
        ::std::size_t line{1};

        while(p < e)
        {
            char8_t const* const line_begin{p};
            char8_t const* line_end{p};
            while(line_end < e && *line_end != u8'\n') { ++line_end; }

            // strip trailing \r
            char8_t const* logical_end{line_end};
            if(logical_end > line_begin && logical_end[-1] == u8'\r') { --logical_end; }

            // find first non-space
            char8_t const* q{line_begin};
            while(q < logical_end && details::is_space(*q) && *q != u8'\n' && *q != u8'\r') { ++q; }

            bool const is_directive{q < logical_end && *q == u8'`'};

            if(is_directive)
            {
                char8_t const* t{q + 1};
                while(t < logical_end && details::is_space(*t)) { ++t; }
                auto const kw_sv{parse_ident(t, logical_end)};

                auto kw_eq = [&](::fast_io::u8string_view kw) noexcept { return kw_sv == kw; };

                auto do_error = [&](::fast_io::u8string_view msg, ::std::size_t col) noexcept
                { out.errors.push_back({::fast_io::u8string{msg}, 0, line, col}); };

                auto const col0{static_cast<::std::size_t>(q - line_begin) + 1};

                if(kw_eq(u8"define"))
                {
                    if(!current_active())
                    {
                        // ignore defines in inactive branches, but keep line structure
                    }
                    else
                    {
                        while(t < logical_end && details::is_space(*t)) { ++t; }
                        auto const name_sv{parse_ident(t, logical_end)};
                        if(name_sv.empty()) { do_error(u8"expected macro name after `define", col0); }
                        else
                        {
                            while(t < logical_end && details::is_space(*t)) { ++t; }
                            ::fast_io::u8string value{};
                            value.assign(::fast_io::u8string_view{t, static_cast<::std::size_t>(logical_end - t)});
                            macros.insert_or_assign(::fast_io::u8string{name_sv}, ::std::move(value));
                        }
                    }
                }
                else if(kw_eq(u8"undef"))
                {
                    if(current_active())
                    {
                        while(t < logical_end && details::is_space(*t)) { ++t; }
                        auto const name_sv{parse_ident(t, logical_end)};
                        if(name_sv.empty()) { do_error(u8"expected macro name after `undef", col0); }
                        else
                        {
                            macros.erase(::fast_io::u8string{name_sv});
                        }
                    }
                }
                else if(kw_eq(u8"ifdef") || kw_eq(u8"ifndef"))
                {
                    while(t < logical_end && details::is_space(*t)) { ++t; }
                    auto const name_sv{parse_ident(t, logical_end)};
                    bool const defined{name_sv.empty() ? false : (macros.find(::fast_io::u8string{name_sv}) != macros.end())};
                    bool const cond{kw_eq(u8"ifdef") ? defined : !defined};
                    bool const parent{current_active()};
                    ifs.push_back({parent, cond, parent && cond, false});
                }
                else if(kw_eq(u8"else"))
                {
                    if(ifs.empty()) { do_error(u8"`else without matching `ifdef/`ifndef", col0); }
                    else
                    {
                        auto& fr{ifs.back_unchecked()};
                        if(fr.seen_else) { do_error(u8"multiple `else in the same conditional block", col0); }
                        else
                        {
                            fr.seen_else = true;
                            fr.active = fr.parent_active && !fr.cond;
                        }
                    }
                }
                else if(kw_eq(u8"endif"))
                {
                    if(ifs.empty()) { do_error(u8"`endif without matching `ifdef/`ifndef", col0); }
                    else
                    {
                        ifs.pop_back();
                    }
                }
                else if(kw_eq(u8"include")) { do_error(u8"`include is not supported in this Verilog subset", col0); }
                else
                {
                    do_error(u8"unsupported Verilog preprocessor directive", col0);
                }

                // Preserve line numbering.
                out.output.push_back(u8'\n');
            }
            else if(current_active())
            {
                // Expand simple macros: `NAME
                char8_t const* s{line_begin};
                while(s < logical_end)
                {
                    if(*s == u8'`')
                    {
                        char8_t const* t{s + 1};
                        auto const name_sv{parse_ident(t, logical_end)};
                        if(name_sv.empty())
                        {
                            out.output.push_back(*s);
                            ++s;
                            continue;
                        }

                        auto it{macros.find(::fast_io::u8string{name_sv})};
                        if(it == macros.end())
                        {
                            auto const col{static_cast<::std::size_t>(s - line_begin) + 1};
                            out.errors.push_back({::fast_io::u8string{u8"undefined Verilog macro"}, 0, line, col});
                        }
                        else
                        {
                            out.output.append(it->second);
                        }
                        s = t;
                        continue;
                    }

                    out.output.push_back(*s);
                    ++s;
                }
                out.output.push_back(u8'\n');
            }
            else
            {
                // inactive block: emit blank line
                out.output.push_back(u8'\n');
            }

            if(line_end < e && *line_end == u8'\n') { p = line_end + 1; }
            else
            {
                p = line_end;
            }
            ++line;
        }

        return out;
    }

    inline lex_result lex(::fast_io::u8string_view src) noexcept
    {
        lex_result out{};
        out.tokens.reserve(src.size() / 2);

        char8_t const* p{src.data()};
        char8_t const* const e{src.data() + src.size()};
        char8_t const* const base{src.data()};

        ::std::size_t line{1};
        ::std::size_t col{1};

        auto bump = [&](char8_t c) noexcept
        {
            if(c == u8'\n')
            {
                ++line;
                col = 1;
            }
            else
            {
                ++col;
            }
        };

        auto make_tok = [&](token_kind k, char8_t const* b, char8_t const* en, ::std::size_t l, ::std::size_t c) noexcept
        {
            out.tokens.push_back({
                k,
                ::fast_io::u8string_view{b, static_cast<::std::size_t>(en - b)},
                l,
                c
            });
        };

        while(p < e)
        {
            char8_t c{*p};

            if(details::is_space(c))
            {
                bump(c);
                ++p;
                continue;
            }

            // comments
            if(c == u8'/' && p + 1 < e)
            {
                char8_t n{p[1]};
                if(n == u8'/')
                {
                    // line comment
                    while(p < e && *p != u8'\n')
                    {
                        bump(*p);
                        ++p;
                    }
                    continue;
                }
                if(n == u8'*')
                {
                    // block comment
                    bump(*p);
                    bump(p[1]);
                    p += 2;
                    while(p < e)
                    {
                        if(*p == u8'*' && p + 1 < e && p[1] == u8'/')
                        {
                            bump(*p);
                            bump(p[1]);
                            p += 2;
                            break;
                        }
                        bump(*p);
                        ++p;
                    }
                    continue;
                }
            }

            // preprocessor/directive: skip whole line for now
            if(c == u8'`')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                out.errors.push_back(
                    {::fast_io::u8string{u8"verilog preprocessor directives are not supported yet"}, static_cast<::std::size_t>(p - base), l0, c0});
                while(p < e && *p != u8'\n')
                {
                    bump(*p);
                    ++p;
                }
                continue;
            }

            if(details::is_alpha(c))
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(*p);
                ++p;
                while(p < e && details::is_ident_continue(*p))
                {
                    bump(*p);
                    ++p;
                }
                make_tok(token_kind::identifier, b, p, l0, c0);
                continue;
            }

            if(details::is_digit(c))
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(*p);
                ++p;
                while(p < e && (details::is_digit(*p) || details::is_alpha(*p) || *p == u8'\'' || *p == u8'_'))
                {
                    bump(*p);
                    ++p;
                }
                make_tok(token_kind::number, b, p, l0, c0);
                continue;
            }

            // 2-char symbols
            if(c == u8'=' && p + 1 < e && p[1] == u8'=')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
            if(c == u8'!' && p + 1 < e && p[1] == u8'=')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
            if(c == u8'&' && p + 1 < e && p[1] == u8'&')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
            if(c == u8'|' && p + 1 < e && p[1] == u8'|')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
            if((c == u8'<' || c == u8'>') && p + 1 < e && p[1] == u8'=')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
            if(c == u8'<' && p + 1 < e && p[1] == u8'<')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
            if(c == u8'>' && p + 1 < e && p[1] == u8'>')
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(p[0]);
                bump(p[1]);
                p += 2;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }

            // 1-char symbol
            {
                ::std::size_t const l0{line};
                ::std::size_t const c0{col};
                char8_t const* const b{p};
                bump(*p);
                ++p;
                make_tok(token_kind::symbol, b, p, l0, c0);
                continue;
            }
        }

        out.tokens.push_back({token_kind::eof, {}, line, col});
        return out;
    }

    enum class port_dir : ::std::uint_fast8_t
    {
        unknown,
        input,
        output,
        inout
    };

    enum class expr_kind : ::std::uint_fast8_t
    {
        literal,
        signal,
        unary_not,
        binary_and,
        binary_or,
        binary_xor,
        binary_eq,
        binary_neq
    };

    struct expr_node
    {
        expr_kind kind{expr_kind::literal};
        logic_t literal{logic_t::indeterminate_state};
        ::std::size_t a{SIZE_MAX};
        ::std::size_t b{SIZE_MAX};
        ::std::size_t signal{SIZE_MAX};
    };

    struct continuous_assign
    {
        ::std::size_t lhs_signal{};
        ::std::size_t expr_root{};
    };

    struct case_item
    {
        bool is_default{};
        logic_t match{logic_t::indeterminate_state};  // 1-bit only for now
        ::fast_io::vector<::std::size_t> stmts{};     // indices into stmt_node arena
    };

    struct stmt_node
    {
        enum class kind : ::std::uint_fast8_t
        {
            empty,
            block,
            blocking_assign,
            nonblocking_assign,
            if_stmt,
            case_stmt
        };

        kind k{kind::empty};
        ::std::uint64_t delay_ticks{};

        ::std::size_t lhs_signal{SIZE_MAX};
        ::std::size_t expr_root{SIZE_MAX};  // assign rhs or if cond

        ::fast_io::vector<::std::size_t> stmts{};       // then-branch (or block list)
        ::fast_io::vector<::std::size_t> else_stmts{};  // else-branch

        ::std::size_t case_expr_root{SIZE_MAX};
        ::fast_io::vector<case_item> case_items{};
    };

    struct always_ff
    {
        ::std::size_t clk_signal{};
        bool posedge{true};
        ::fast_io::vector<stmt_node> stmt_nodes{};
        ::fast_io::vector<::std::size_t> roots{};
    };

    struct always_comb
    {
        ::fast_io::vector<stmt_node> stmt_nodes{};
        ::fast_io::vector<::std::size_t> roots{};
    };

    struct port_decl
    {
        port_dir dir{port_dir::unknown};
        bool has_range{};
        int msb{};
        int lsb{};
        bool is_reg{};
    };

    struct vector_desc
    {
        int msb{};
        int lsb{};
        bool is_reg{};
        // Bits are stored in *declaration order*: [0] is msb, last is lsb.
        ::fast_io::vector<::std::size_t> bits{};
    };

    enum class connection_kind : ::std::uint_fast8_t
    {
        unconnected,
        scalar,
        vector,
        literal
    };

    struct connection_ref
    {
        connection_kind kind{connection_kind::unconnected};
        ::std::size_t scalar_signal{SIZE_MAX};
        ::fast_io::u8string vector_base{};
        logic_t literal{logic_t::indeterminate_state};
    };

    struct instance_connection
    {
        ::fast_io::u8string port_name{};  // empty => positional
        connection_ref ref{};
    };

    struct instance
    {
        ::fast_io::u8string module_name{};
        ::fast_io::u8string instance_name{};
        ::fast_io::vector<instance_connection> connections{};
    };

    struct port
    {
        ::fast_io::u8string name{};
        port_dir dir{port_dir::unknown};
        ::std::size_t signal{};
    };

    struct compiled_module
    {
        ::fast_io::u8string name{};

        // Bit-level ports (vector ports are expanded to multiple entries).
        ::fast_io::vector<port> ports{};
        // Port groups in order (base names), used for positional instantiation mapping.
        ::fast_io::vector<::fast_io::u8string> port_order{};
        ::absl::btree_map<::fast_io::u8string, port_decl> port_decls{};
        ::absl::btree_map<::fast_io::u8string, vector_desc> vectors{};

        ::fast_io::vector<::fast_io::u8string> signal_names{};
        ::fast_io::vector<bool> signal_is_reg{};
        ::absl::btree_map<::fast_io::u8string, ::std::size_t> signal_index{};

        ::fast_io::vector<expr_node> expr_nodes{};
        ::fast_io::vector<continuous_assign> assigns{};
        ::fast_io::vector<always_ff> always_ffs{};
        ::fast_io::vector<always_comb> always_combs{};
        ::fast_io::vector<instance> instances{};
    };

    struct compile_result
    {
        ::fast_io::vector<compiled_module> modules{};
        ::fast_io::vector<compile_error> errors{};
    };

    namespace details
    {
        inline constexpr bool is_kw(::fast_io::u8string_view t, ::fast_io::u8string_view kw) noexcept { return t == kw; }

        inline constexpr bool is_sym(::fast_io::u8string_view t, char8_t c) noexcept { return t.size() == 1 && t[0] == c; }

        inline constexpr bool is_sym2(::fast_io::u8string_view t, char8_t a, char8_t b) noexcept { return t.size() == 2 && t[0] == a && t[1] == b; }

        inline constexpr logic_t parse_1bit_literal(::fast_io::u8string_view t) noexcept
        {
            // supported: 0/1, 1'b0/1'b1/1'bx/1'bz (case-insensitive on base+digit)
            if(t == u8"0") { return logic_t::false_state; }
            if(t == u8"1") { return logic_t::true_state; }

            if(t.size() >= 4 && t[1] == u8'\'' && (t[2] == u8'b' || t[2] == u8'B'))
            {
                char8_t const v{t.back()};
                if(v == u8'0') { return logic_t::false_state; }
                if(v == u8'1') { return logic_t::true_state; }
                if(v == u8'x' || v == u8'X') { return logic_t::indeterminate_state; }
                if(v == u8'z' || v == u8'Z') { return logic_t::high_impedence_state; }
            }

            return logic_t::indeterminate_state;
        }

        struct parser
        {
            ::fast_io::vector<token> const* toks{};
            ::std::size_t pos{};
            ::fast_io::vector<compile_error>* errors{};

            [[nodiscard]] token const& peek() const noexcept { return toks->index_unchecked(pos); }

            [[nodiscard]] bool eof() const noexcept { return peek().kind == token_kind::eof; }

            token const& consume() noexcept
            {
                token const& t{peek()};
                if(!eof()) { ++pos; }
                return t;
            }

            void err(token const& t, ::fast_io::u8string_view msg) noexcept { errors->push_back({::fast_io::u8string{msg}, 0, t.line, t.column}); }

            [[nodiscard]] bool accept_sym(char8_t c) noexcept
            {
                auto const& t{peek()};
                if(t.kind == token_kind::symbol && is_sym(t.text, c))
                {
                    consume();
                    return true;
                }
                return false;
            }

            [[nodiscard]] bool accept_sym2(char8_t a, char8_t b) noexcept
            {
                auto const& t{peek()};
                if(t.kind == token_kind::symbol && is_sym2(t.text, a, b))
                {
                    consume();
                    return true;
                }
                return false;
            }

            [[nodiscard]] bool accept_kw(::fast_io::u8string_view kw) noexcept
            {
                auto const& t{peek()};
                if(t.kind == token_kind::identifier && is_kw(t.text, kw))
                {
                    consume();
                    return true;
                }
                return false;
            }

            [[nodiscard]] ::fast_io::u8string expect_ident(::fast_io::u8string_view what) noexcept
            {
                auto const& t{peek()};
                if(t.kind != token_kind::identifier)
                {
                    err(t, what);
                    return {};
                }
                consume();
                return ::fast_io::u8string{t.text};
            }

            void skip_until_semicolon() noexcept
            {
                for(; !eof();)
                {
                    if(accept_sym(u8';')) { return; }
                    consume();
                }
            }
        };

        inline ::std::size_t get_or_create_signal(compiled_module& m, ::fast_io::u8string_view name) noexcept
        {
            auto it{m.signal_index.find(::fast_io::u8string{name})};
            if(it != m.signal_index.end()) { return it->second; }
            ::std::size_t const idx{m.signal_names.size()};
            m.signal_names.push_back(::fast_io::u8string{name});
            m.signal_is_reg.push_back(false);
            m.signal_index.insert({::fast_io::u8string{name}, idx});
            return idx;
        }

        inline ::std::size_t get_or_create_signal(compiled_module& m, ::fast_io::u8string const& name) noexcept
        { return get_or_create_signal(m, ::fast_io::u8string_view{name.data(), name.size()}); }

        inline void append_decimal(::fast_io::u8string& s, int v) noexcept
        {
            char buf[32];
            auto const* const b{buf};
            auto const* const e{buf + sizeof(buf)};
            auto const r{::std::to_chars(const_cast<char*>(b), const_cast<char*>(e), v)};
            if(r.ec != ::std::errc{}) [[unlikely]] { return; }
            for(auto const* p{b}; p != r.ptr; ++p) { s.push_back(static_cast<char8_t>(*p)); }
        }

        inline bool parse_dec_int(::fast_io::u8string_view t, int& out) noexcept
        {
            if(t.empty()) { return false; }
            bool neg{};
            ::std::int64_t value{};
            ::std::size_t i{};
            if(t[0] == u8'-')
            {
                neg = true;
                i = 1;
            }
            for(; i < t.size(); ++i)
            {
                char8_t const c{t[i]};
                if(c == u8'_') { continue; }
                if(c < u8'0' || c > u8'9') { return false; }
                value = value * 10 + (c - u8'0');
                if(value > static_cast<::std::int64_t>(::std::numeric_limits<int>::max())) { return false; }
            }
            out = neg ? -static_cast<int>(value) : static_cast<int>(value);
            return true;
        }

        struct range_desc
        {
            bool has_range{};
            int msb{};
            int lsb{};
        };

        inline bool accept_range(parser& p, range_desc& r) noexcept
        {
            if(!p.accept_sym(u8'[')) { return false; }

            int a{};
            auto const& t0{p.peek()};
            if(t0.kind != token_kind::number || !parse_dec_int(t0.text, a))
            {
                p.err(t0, u8"expected integer in range");
                while(!p.eof() && !p.accept_sym(u8']')) { p.consume(); }
                return false;
            }
            p.consume();

            int b{a};
            if(p.accept_sym(u8':'))
            {
                auto const& t1{p.peek()};
                if(t1.kind != token_kind::number || !parse_dec_int(t1.text, b))
                {
                    p.err(t1, u8"expected integer after ':'");
                    while(!p.eof() && !p.accept_sym(u8']')) { p.consume(); }
                    return false;
                }
                p.consume();
            }

            if(!p.accept_sym(u8']')) { p.err(p.peek(), u8"expected ']'"); }

            r.has_range = true;
            r.msb = a;
            r.lsb = b;
            return true;
        }

        inline ::fast_io::u8string make_bit_name(::fast_io::u8string_view base, int idx) noexcept
        {
            ::fast_io::u8string name{base};
            name.push_back(u8'[');
            append_decimal(name, idx);
            name.push_back(u8']');
            return name;
        }

        inline ::std::size_t vector_width(vector_desc const& vd) noexcept
        {
            auto const d{vd.msb - vd.lsb};
            return static_cast<::std::size_t>(d >= 0 ? d + 1 : -d + 1);
        }

        inline bool vector_contains(vector_desc const& vd, int idx) noexcept
        {
            if(vd.msb >= vd.lsb) { return idx <= vd.msb && idx >= vd.lsb; }
            return idx >= vd.msb && idx <= vd.lsb;
        }

        inline ::std::size_t vector_pos(vector_desc const& vd, int idx) noexcept
        {
            if(!vector_contains(vd, idx)) { return SIZE_MAX; }
            if(vd.msb >= vd.lsb) { return static_cast<::std::size_t>(vd.msb - idx); }
            return static_cast<::std::size_t>(idx - vd.msb);
        }

        inline int vector_index_at(vector_desc const& vd, ::std::size_t pos) noexcept
        {
            if(vd.msb >= vd.lsb) { return vd.msb - static_cast<int>(pos); }
            return vd.msb + static_cast<int>(pos);
        }

        inline vector_desc& declare_vector_range(parser* p, compiled_module& m, ::fast_io::u8string_view base, int msb, int lsb, bool is_reg) noexcept
        {
            ::fast_io::u8string key{base};
            auto it{m.vectors.find(key)};
            if(it != m.vectors.end())
            {
                auto& vd{it->second};
                if(vd.msb != msb || vd.lsb != lsb)
                {
                    if(p) { p->err(p->peek(), u8"redeclared vector with a different range is not supported"); }
                }
                if(is_reg && !vd.is_reg)
                {
                    vd.is_reg = true;
                    for(auto const sig: vd.bits)
                    {
                        if(sig < m.signal_is_reg.size()) { m.signal_is_reg.index_unchecked(sig) = true; }
                    }
                }
                return vd;
            }

            vector_desc vd{};
            vd.msb = msb;
            vd.lsb = lsb;
            vd.is_reg = is_reg;

            ::std::size_t const w{vector_width(vd)};
            vd.bits.resize(w);

            for(::std::size_t pos{}; pos < w; ++pos)
            {
                int const idx{vector_index_at(vd, pos)};
                auto const bit_name{make_bit_name(base, idx)};
                auto const sig{get_or_create_signal(m, bit_name)};
                vd.bits.index_unchecked(pos) = sig;
                if(is_reg) { m.signal_is_reg.index_unchecked(sig) = true; }
            }

            auto res{m.vectors.insert({::std::move(key), ::std::move(vd)})};
            return res.first->second;
        }

        inline ::std::size_t get_or_create_vector_bit(compiled_module& m, ::fast_io::u8string_view base, int idx, bool is_reg) noexcept
        {
            ::fast_io::u8string key{base};
            auto it{m.vectors.find(key)};
            if(it != m.vectors.end())
            {
                auto const pos{vector_pos(it->second, idx)};
                if(pos != SIZE_MAX) { return it->second.bits.index_unchecked(pos); }
            }

            auto const bit_name{make_bit_name(base, idx)};
            auto const sig{get_or_create_signal(m, bit_name)};
            if(is_reg) { m.signal_is_reg.index_unchecked(sig) = true; }
            return sig;
        }

        inline port_decl& ensure_port_decl(compiled_module& m, ::fast_io::u8string_view base) noexcept
        {
            ::fast_io::u8string key{base};
            auto it{m.port_decls.find(key)};
            if(it != m.port_decls.end()) { return it->second; }
            m.port_order.push_back(key);
            auto res{m.port_decls.insert({::std::move(key), port_decl{}})};
            return res.first->second;
        }

        struct expr_parser
        {
            parser* p{};
            compiled_module* m{};

            [[nodiscard]] ::std::size_t add_node(expr_node n) noexcept
            {
                ::std::size_t const idx{m->expr_nodes.size()};
                m->expr_nodes.push_back(n);
                return idx;
            }

            [[nodiscard]] ::std::size_t parse_primary() noexcept
            {
                auto const& t{p->peek()};
                if(t.kind == token_kind::identifier)
                {
                    ::fast_io::u8string name{t.text};
                    p->consume();

                    // bit-select: a[3]
                    if(p->accept_sym(u8'['))
                    {
                        int idx{};
                        auto const& ti{p->peek()};
                        if(ti.kind != token_kind::number || !parse_dec_int(ti.text, idx))
                        {
                            p->err(ti, u8"expected constant integer bit-select index");
                        }
                        else { p->consume(); }
                        if(!p->accept_sym(u8']')) { p->err(p->peek(), u8"expected ']'"); }

                        return add_node({.kind = expr_kind::signal,
                                         .signal = get_or_create_vector_bit(*m, ::fast_io::u8string_view{name.data(), name.size()}, idx, false)});
                    }

                    // If the identifier is a vector, require bit-select unless width==1.
                    auto it{m->vectors.find(name)};
                    if(it != m->vectors.end() && vector_width(it->second) != 1)
                    {
                        p->err(t, u8"vector used in 1-bit expression without bit-select");
                        return add_node({.kind = expr_kind::literal, .literal = logic_t::indeterminate_state});
                    }

                    if(it != m->vectors.end() && !it->second.bits.empty())
                    {
                        return add_node({.kind = expr_kind::signal, .signal = it->second.bits.front_unchecked()});
                    }

                    return add_node({.kind = expr_kind::signal, .signal = get_or_create_signal(*m, name)});
                }
                if(t.kind == token_kind::number)
                {
                    auto const lit{parse_1bit_literal(t.text)};
                    p->consume();
                    return add_node({.kind = expr_kind::literal, .literal = lit});
                }
                if(t.kind == token_kind::symbol && is_sym(t.text, u8'('))
                {
                    p->consume();
                    ::std::size_t const e{parse_expr()};
                    if(!p->accept_sym(u8')')) { p->err(p->peek(), u8"expected ')'"); }
                    return e;
                }
                p->err(t, u8"expected expression");
                p->consume();
                return add_node({.kind = expr_kind::literal, .literal = logic_t::indeterminate_state});
            }

            [[nodiscard]] ::std::size_t parse_unary() noexcept
            {
                if(p->accept_sym(u8'~'))
                {
                    ::std::size_t const a{parse_unary()};
                    return add_node({.kind = expr_kind::unary_not, .a = a});
                }
                return parse_primary();
            }

            [[nodiscard]] ::std::size_t parse_and() noexcept
            {
                ::std::size_t lhs{parse_unary()};
                for(;;)
                {
                    if(p->accept_sym(u8'&'))
                    {
                        ::std::size_t const rhs{parse_unary()};
                        lhs = add_node({.kind = expr_kind::binary_and, .a = lhs, .b = rhs});
                        continue;
                    }
                    return lhs;
                }
            }

            [[nodiscard]] ::std::size_t parse_xor() noexcept
            {
                ::std::size_t lhs{parse_and()};
                for(;;)
                {
                    if(p->accept_sym(u8'^'))
                    {
                        ::std::size_t const rhs{parse_and()};
                        lhs = add_node({.kind = expr_kind::binary_xor, .a = lhs, .b = rhs});
                        continue;
                    }
                    return lhs;
                }
            }

            [[nodiscard]] ::std::size_t parse_or() noexcept
            {
                ::std::size_t lhs{parse_xor()};
                for(;;)
                {
                    if(p->accept_sym(u8'|'))
                    {
                        ::std::size_t const rhs{parse_xor()};
                        lhs = add_node({.kind = expr_kind::binary_or, .a = lhs, .b = rhs});
                        continue;
                    }
                    return lhs;
                }
            }

            [[nodiscard]] ::std::size_t parse_eq() noexcept
            {
                ::std::size_t lhs{parse_or()};
                for(;;)
                {
                    if(p->accept_sym2(u8'=', u8'='))
                    {
                        ::std::size_t const rhs{parse_or()};
                        lhs = add_node({.kind = expr_kind::binary_eq, .a = lhs, .b = rhs});
                        continue;
                    }
                    if(p->accept_sym2(u8'!', u8'='))
                    {
                        ::std::size_t const rhs{parse_or()};
                        lhs = add_node({.kind = expr_kind::binary_neq, .a = lhs, .b = rhs});
                        continue;
                    }
                    return lhs;
                }
            }

            [[nodiscard]] ::std::size_t parse_land() noexcept
            {
                ::std::size_t lhs{parse_eq()};
                for(;;)
                {
                    if(p->accept_sym2(u8'&', u8'&'))
                    {
                        ::std::size_t const rhs{parse_eq()};
                        lhs = add_node({.kind = expr_kind::binary_and, .a = lhs, .b = rhs});
                        continue;
                    }
                    return lhs;
                }
            }

            [[nodiscard]] ::std::size_t parse_lor() noexcept
            {
                ::std::size_t lhs{parse_land()};
                for(;;)
                {
                    if(p->accept_sym2(u8'|', u8'|'))
                    {
                        ::std::size_t const rhs{parse_land()};
                        lhs = add_node({.kind = expr_kind::binary_or, .a = lhs, .b = rhs});
                        continue;
                    }
                    return lhs;
                }
            }

            [[nodiscard]] ::std::size_t parse_expr() noexcept { return parse_lor(); }
        };

        inline bool parse_lvalue(parser& p, compiled_module& m, ::std::size_t& sig_out, bool mark_reg) noexcept
        {
            auto const name{p.expect_ident(u8"expected identifier")};
            if(name.empty()) { return false; }

            if(p.accept_sym(u8'['))
            {
                int idx{};
                auto const& ti{p.peek()};
                if(ti.kind != token_kind::number || !parse_dec_int(ti.text, idx))
                {
                    p.err(ti, u8"expected constant integer bit-select index");
                }
                else { p.consume(); }
                if(!p.accept_sym(u8']')) { p.err(p.peek(), u8"expected ']'"); }
                sig_out = get_or_create_vector_bit(m, ::fast_io::u8string_view{name.data(), name.size()}, idx, mark_reg);
                return true;
            }

            sig_out = get_or_create_signal(m, name);
            if(mark_reg && sig_out < m.signal_is_reg.size()) { m.signal_is_reg.index_unchecked(sig_out) = true; }
            return true;
        }

        inline connection_ref parse_connection_ref(parser& p, compiled_module& m) noexcept
        {
            connection_ref r{};
            auto const& t{p.peek()};
            if(t.kind == token_kind::symbol && (is_sym(t.text, u8')') || is_sym(t.text, u8','))) { return r; }

            if(t.kind == token_kind::number)
            {
                r.kind = connection_kind::literal;
                r.literal = parse_1bit_literal(t.text);
                p.consume();
                return r;
            }

            if(t.kind == token_kind::identifier)
            {
                ::fast_io::u8string name{t.text};
                p.consume();

                if(p.accept_sym(u8'['))
                {
                    int idx{};
                    auto const& ti{p.peek()};
                    if(ti.kind != token_kind::number || !parse_dec_int(ti.text, idx))
                    {
                        p.err(ti, u8"expected constant integer bit-select index");
                    }
                    else { p.consume(); }
                    if(!p.accept_sym(u8']')) { p.err(p.peek(), u8"expected ']'"); }

                    r.kind = connection_kind::scalar;
                    r.scalar_signal = get_or_create_vector_bit(m, ::fast_io::u8string_view{name.data(), name.size()}, idx, false);
                    return r;
                }

                if(m.vectors.find(name) != m.vectors.end())
                {
                    r.kind = connection_kind::vector;
                    r.vector_base = ::std::move(name);
                    return r;
                }

                r.kind = connection_kind::scalar;
                r.scalar_signal = get_or_create_signal(m, name);
                return r;
            }

            p.err(t, u8"expected connection expression");
            p.consume();
            return r;
        }

        inline bool try_parse_instance(parser& p, compiled_module& m) noexcept
        {
            ::std::size_t const pos0{p.pos};
            auto const& t0{p.peek()};
            if(t0.kind != token_kind::identifier) { return false; }

            ::fast_io::u8string module_name{t0.text};
            p.consume();

            auto const inst_name{p.expect_ident(u8"expected instance identifier")};
            if(inst_name.empty())
            {
                p.pos = pos0;
                return false;
            }

            if(!p.accept_sym(u8'('))
            {
                p.pos = pos0;
                return false;
            }

            instance inst{};
            inst.module_name = ::std::move(module_name);
            inst.instance_name = inst_name;

            if(!p.accept_sym(u8')'))
            {
                for(;;)
                {
                    instance_connection ic{};
                    if(p.accept_sym(u8'.'))
                    {
                        ic.port_name = p.expect_ident(u8"expected port identifier after '.'");
                        if(!p.accept_sym(u8'(')) { p.err(p.peek(), u8"expected '(' after .port"); }
                        if(!p.accept_sym(u8')'))
                        {
                            ic.ref = parse_connection_ref(p, m);
                            if(!p.accept_sym(u8')')) { p.err(p.peek(), u8"expected ')' after connection"); }
                        }
                    }
                    else
                    {
                        ic.ref = parse_connection_ref(p, m);
                    }

                    inst.connections.push_back(::std::move(ic));

                    if(p.accept_sym(u8')')) { break; }
                    if(!p.accept_sym(u8','))
                    {
                        p.err(p.peek(), u8"expected ',' or ')' in instance connection list");
                        while(!p.eof() && !p.accept_sym(u8')')) { p.consume(); }
                        break;
                    }
                }
            }

            if(!p.accept_sym(u8';'))
            {
                p.err(p.peek(), u8"expected ';' after instance");
                p.skip_until_semicolon();
            }

            m.instances.push_back(::std::move(inst));
            return true;
        }

        inline ::std::size_t add_stmt(::fast_io::vector<stmt_node>& arena, stmt_node n) noexcept
        {
            ::std::size_t const idx{arena.size()};
            arena.push_back(::std::move(n));
            return idx;
        }

        inline ::std::uint64_t parse_delay_ticks(parser& p) noexcept
        {
            auto const& t{p.peek()};
            if(t.kind != token_kind::number)
            {
                p.err(t, u8"expected integer delay after '#'");
                return 0;
            }

            int v{};
            if(!parse_dec_int(t.text, v) || v < 0)
            {
                p.err(t, u8"expected non-negative integer delay after '#'");
                p.consume();
                return 0;
            }

            p.consume();
            return static_cast<::std::uint64_t>(v);
        }

        inline ::std::size_t parse_proc_stmt(parser& p, compiled_module& m, ::fast_io::vector<stmt_node>& arena, bool allow_nonblocking) noexcept
        {
            if(p.accept_sym(u8';')) { return add_stmt(arena, {}); }

            ::std::uint64_t delay{};
            if(p.accept_sym(u8'#')) { delay = parse_delay_ticks(p); }

            if(p.accept_kw(u8"begin"))
            {
                if(delay != 0) { p.err(p.peek(), u8"delay before begin/end block is not supported"); }

                stmt_node blk{};
                blk.k = stmt_node::kind::block;
                while(!p.eof() && !p.accept_kw(u8"end"))
                {
                    blk.stmts.push_back(parse_proc_stmt(p, m, arena, allow_nonblocking));
                }
                return add_stmt(arena, ::std::move(blk));
            }

            if(p.accept_kw(u8"if"))
            {
                if(delay != 0) { p.err(p.peek(), u8"delay before if is not supported"); }

                if(!p.accept_sym(u8'(')) { p.err(p.peek(), u8"expected '(' after if"); }
                expr_parser ep{&p, &m};
                ::std::size_t const cond{ep.parse_expr()};
                if(!p.accept_sym(u8')')) { p.err(p.peek(), u8"expected ')' after if condition"); }

                stmt_node st{};
                st.k = stmt_node::kind::if_stmt;
                st.expr_root = cond;
                st.stmts.push_back(parse_proc_stmt(p, m, arena, allow_nonblocking));
                if(p.accept_kw(u8"else")) { st.else_stmts.push_back(parse_proc_stmt(p, m, arena, allow_nonblocking)); }
                return add_stmt(arena, ::std::move(st));
            }

            if(p.accept_kw(u8"case"))
            {
                if(delay != 0) { p.err(p.peek(), u8"delay before case is not supported"); }

                if(!p.accept_sym(u8'(')) { p.err(p.peek(), u8"expected '(' after case"); }
                expr_parser ep{&p, &m};
                ::std::size_t const cexpr{ep.parse_expr()};
                if(!p.accept_sym(u8')')) { p.err(p.peek(), u8"expected ')' after case expression"); }

                stmt_node st{};
                st.k = stmt_node::kind::case_stmt;
                st.case_expr_root = cexpr;

                while(!p.eof())
                {
                    if(p.accept_kw(u8"endcase")) { break; }

                    if(p.accept_kw(u8"default"))
                    {
                        if(!p.accept_sym(u8':')) { p.err(p.peek(), u8"expected ':' after default"); }
                        case_item ci{};
                        ci.is_default = true;
                        ci.stmts.push_back(parse_proc_stmt(p, m, arena, allow_nonblocking));
                        st.case_items.push_back(::std::move(ci));
                        continue;
                    }

                    ::fast_io::vector<logic_t> matches{};
                    for(;;)
                    {
                        auto const& mt{p.peek()};
                        if(mt.kind != token_kind::number)
                        {
                            p.err(mt, u8"expected 1-bit literal in case item");
                            break;
                        }
                        matches.push_back(parse_1bit_literal(mt.text));
                        p.consume();
                        if(!p.accept_sym(u8',')) { break; }
                    }

                    if(!p.accept_sym(u8':')) { p.err(p.peek(), u8"expected ':' after case item"); }
                    ::std::size_t const body{parse_proc_stmt(p, m, arena, allow_nonblocking)};

                    for(auto const mv: matches)
                    {
                        case_item ci{};
                        ci.is_default = false;
                        ci.match = mv;
                        ci.stmts.push_back(body);
                        st.case_items.push_back(::std::move(ci));
                    }
                }

                return add_stmt(arena, ::std::move(st));
            }

            ::std::size_t lhs{};
            if(!parse_lvalue(p, m, lhs, true))
            {
                p.skip_until_semicolon();
                return add_stmt(arena, {});
            }

            bool nonblocking{};
            if(p.accept_sym2(u8'<', u8'='))
            {
                if(!allow_nonblocking) { p.err(p.peek(), u8"nonblocking assignments are not supported here"); }
                nonblocking = true;
            }
            else if(!p.accept_sym(u8'='))
            {
                p.err(p.peek(), u8"expected '=' or '<=' in assignment");
                p.skip_until_semicolon();
                return add_stmt(arena, {});
            }

            expr_parser ep{&p, &m};
            ::std::size_t const rhs{ep.parse_expr()};

            if(!p.accept_sym(u8';'))
            {
                p.err(p.peek(), u8"expected ';' after assignment");
                p.skip_until_semicolon();
            }

            stmt_node st{};
            st.k = nonblocking ? stmt_node::kind::nonblocking_assign : stmt_node::kind::blocking_assign;
            st.delay_ticks = delay;
            st.lhs_signal = lhs;
            st.expr_root = rhs;
            if(lhs < m.signal_is_reg.size()) { m.signal_is_reg.index_unchecked(lhs) = true; }
            return add_stmt(arena, ::std::move(st));
        }

        inline void parse_decl_list(parser& p, compiled_module& m, port_dir d) noexcept
        {
            bool is_reg{};
            if(d == port_dir::output)
            {
                if(p.accept_kw(u8"reg")) { is_reg = true; }
                else if(p.accept_kw(u8"wire"))
                {
                    // ignored
                }
            }

            range_desc r{};
            accept_range(p, r);

            for(;;)
            {
                auto const name{p.expect_ident(u8"expected identifier in declaration")};
                if(name.empty()) { return; }

                auto& pd{ensure_port_decl(m, ::fast_io::u8string_view{name.data(), name.size()})};
                pd.dir = d;
                if(is_reg) { pd.is_reg = true; }
                if(r.has_range)
                {
                    pd.has_range = true;
                    pd.msb = r.msb;
                    pd.lsb = r.lsb;
                    (void)declare_vector_range(&p, m, ::fast_io::u8string_view{name.data(), name.size()}, r.msb, r.lsb, is_reg);
                }
                else
                {
                    auto const sig{get_or_create_signal(m, name)};
                    if(is_reg && sig < m.signal_is_reg.size()) { m.signal_is_reg.index_unchecked(sig) = true; }
                }

                if(!p.accept_sym(u8',')) { break; }
            }

            if(!p.accept_sym(u8';'))
            {
                p.err(p.peek(), u8"expected ';'");
                p.skip_until_semicolon();
            }
        }

        inline void parse_wire_list(parser& p, compiled_module& m, bool is_reg) noexcept
        {
            range_desc r{};
            accept_range(p, r);

            for(;;)
            {
                auto const name{p.expect_ident(u8"expected identifier in declaration")};
                if(name.empty()) { return; }

                if(r.has_range)
                {
                    (void)declare_vector_range(&p, m, ::fast_io::u8string_view{name.data(), name.size()}, r.msb, r.lsb, is_reg);
                }
                else
                {
                    auto const sig{get_or_create_signal(m, name)};
                    if(is_reg && sig < m.signal_is_reg.size()) { m.signal_is_reg.index_unchecked(sig) = true; }
                }

                if(!p.accept_sym(u8',')) { break; }
            }

            if(!p.accept_sym(u8';'))
            {
                p.err(p.peek(), u8"expected ';'");
                p.skip_until_semicolon();
            }
        }

        inline void parse_assign_stmt(parser& p, compiled_module& m) noexcept
        {
            ::std::size_t lhs{};
            if(!parse_lvalue(p, m, lhs, false))
            {
                p.skip_until_semicolon();
                return;
            }
            if(!p.accept_sym(u8'='))
            {
                p.err(p.peek(), u8"expected '=' in assign");
                p.skip_until_semicolon();
                return;
            }

            expr_parser ep{&p, &m};
            ::std::size_t const root{ep.parse_expr()};

            if(!p.accept_sym(u8';'))
            {
                p.err(p.peek(), u8"expected ';' after assign");
                p.skip_until_semicolon();
            }

            m.assigns.push_back({lhs, root});
        }

        inline void parse_always(parser& p, compiled_module& m) noexcept
        {
            if(!p.accept_sym(u8'@')) { p.err(p.peek(), u8"expected '@' after always"); p.skip_until_semicolon(); return; }

            bool comb{};
            if(p.accept_sym(u8'*'))
            {
                comb = true;
            }
            else
            {
                if(!p.accept_sym(u8'(')) { p.err(p.peek(), u8"expected '(' after @"); p.skip_until_semicolon(); return; }
                if(p.accept_sym(u8'*'))
                {
                    comb = true;
                    if(!p.accept_sym(u8')')) { p.err(p.peek(), u8"expected ')' after @*"); }
                }
            }

            if(comb)
            {
                always_comb ac{};
                ac.stmt_nodes.reserve(64);
                ac.roots.push_back(parse_proc_stmt(p, m, ac.stmt_nodes, false));
                m.always_combs.push_back(::std::move(ac));
                return;
            }

            bool posedge{true};
            if(p.accept_kw(u8"posedge")) { posedge = true; }
            else if(p.accept_kw(u8"negedge")) { posedge = false; }
            else { p.err(p.peek(), u8"expected posedge/negedge or '*'"); }

            auto const clk_name{p.expect_ident(u8"expected clock identifier")};
            if(clk_name.empty())
            {
                while(!p.eof() && !p.accept_sym(u8')')) { p.consume(); }
                return;
            }

            while(!p.eof() && !p.accept_sym(u8')')) { p.consume(); }

            always_ff ff{};
            ff.clk_signal = get_or_create_signal(m, clk_name);
            ff.posedge = posedge;
            ff.stmt_nodes.reserve(64);
            ff.roots.push_back(parse_proc_stmt(p, m, ff.stmt_nodes, true));
            m.always_ffs.push_back(::std::move(ff));
        }

        inline compiled_module parse_module(parser& p) noexcept
        {
            compiled_module m{};
            m.expr_nodes.reserve(256);
            m.assigns.reserve(128);
            m.always_ffs.reserve(32);
            m.always_combs.reserve(32);
            m.instances.reserve(64);

            m.name = p.expect_ident(u8"expected module name");

            if(!p.accept_sym(u8'('))
            {
                p.err(p.peek(), u8"expected '(' after module name");
                p.skip_until_semicolon();
                return m;
            }

            // port list: plain names or ANSI declarations (scalar/vectors)
            if(!p.accept_sym(u8')'))
            {
                port_dir current_dir{port_dir::unknown};
                bool current_is_reg{};
                range_desc current_range{};

                for(;;)
                {
                    if(p.accept_kw(u8"input"))
                    {
                        current_dir = port_dir::input;
                        current_is_reg = false;
                        current_range = {};
                        accept_range(p, current_range);
                    }
                    else if(p.accept_kw(u8"output"))
                    {
                        current_dir = port_dir::output;
                        current_is_reg = false;
                        current_range = {};
                        if(p.accept_kw(u8"reg")) { current_is_reg = true; }
                        else if(p.accept_kw(u8"wire"))
                        {
                            // ignored
                        }
                        accept_range(p, current_range);
                    }
                    else if(p.accept_kw(u8"inout"))
                    {
                        current_dir = port_dir::inout;
                        current_is_reg = false;
                        current_range = {};
                        accept_range(p, current_range);
                    }

                    auto const port_name{p.expect_ident(u8"expected port identifier")};
                    if(!port_name.empty())
                    {
                        auto& pd{ensure_port_decl(m, ::fast_io::u8string_view{port_name.data(), port_name.size()})};
                        if(current_dir != port_dir::unknown) { pd.dir = current_dir; }
                        if(current_is_reg) { pd.is_reg = true; }
                        if(current_range.has_range)
                        {
                            pd.has_range = true;
                            pd.msb = current_range.msb;
                            pd.lsb = current_range.lsb;
                            (void)declare_vector_range(&p,
                                                       m,
                                                       ::fast_io::u8string_view{port_name.data(), port_name.size()},
                                                       current_range.msb,
                                                       current_range.lsb,
                                                       current_is_reg);
                        }
                    }

                    if(p.accept_sym(u8')')) { break; }
                    if(!p.accept_sym(u8',')) { p.err(p.peek(), u8"expected ',' or ')' in port list"); }
                }
            }

            if(!p.accept_sym(u8';'))
            {
                p.err(p.peek(), u8"expected ';' after module header");
                p.skip_until_semicolon();
            }

            // body
            while(!p.eof())
            {
                if(p.accept_kw(u8"endmodule")) { break; }

                if(p.accept_kw(u8"input"))
                {
                    parse_decl_list(p, m, port_dir::input);
                    continue;
                }
                if(p.accept_kw(u8"output"))
                {
                    parse_decl_list(p, m, port_dir::output);
                    continue;
                }
                if(p.accept_kw(u8"inout"))
                {
                    parse_decl_list(p, m, port_dir::inout);
                    continue;
                }

                if(p.accept_kw(u8"wire"))
                {
                    parse_wire_list(p, m, false);
                    continue;
                }
                if(p.accept_kw(u8"reg"))
                {
                    parse_wire_list(p, m, true);
                    continue;
                }

                if(p.accept_kw(u8"assign"))
                {
                    parse_assign_stmt(p, m);
                    continue;
                }
                if(p.accept_kw(u8"always"))
                {
                    parse_always(p, m);
                    continue;
                }

                if(try_parse_instance(p, m)) { continue; }

                // unknown statement: skip
                p.skip_until_semicolon();
            }

            // finalize bit-level ports (expand vectors)
            m.ports.clear();
            for(auto const& base: m.port_order)
            {
                auto it{m.port_decls.find(base)};
                port_decl pd{};
                if(it != m.port_decls.end()) { pd = it->second; }

                if(pd.has_range)
                {
                    auto& vd{declare_vector_range(nullptr, m, ::fast_io::u8string_view{base.data(), base.size()}, pd.msb, pd.lsb, pd.is_reg)};
                    ::std::size_t const w{vector_width(vd)};
                    for(::std::size_t pos{}; pos < w; ++pos)
                    {
                        int const idx{vector_index_at(vd, pos)};
                        ::fast_io::u8string pn{make_bit_name(::fast_io::u8string_view{base.data(), base.size()}, idx)};
                        m.ports.push_back({::std::move(pn), pd.dir, vd.bits.index_unchecked(pos)});
                    }
                }
                else
                {
                    auto const sig{get_or_create_signal(m, base)};
                    if(pd.is_reg && sig < m.signal_is_reg.size()) { m.signal_is_reg.index_unchecked(sig) = true; }
                    m.ports.push_back({base, pd.dir, sig});
                }
            }

            return m;
        }
    }  // namespace details

    inline compile_result compile(::fast_io::u8string_view src) noexcept
    {
        compile_result out{};
        auto pp{preprocess(src)};
        out.errors = ::std::move(pp.errors);

        auto const lr{lex(::fast_io::u8string_view{pp.output.data(), pp.output.size()})};
        out.errors.reserve(out.errors.size() + lr.errors.size());
        for(auto const& e: lr.errors) { out.errors.push_back(e); }

        details::parser p{__builtin_addressof(lr.tokens), 0, __builtin_addressof(out.errors)};

        while(!p.eof())
        {
            if(p.accept_kw(u8"module"))
            {
                out.modules.push_back(details::parse_module(p));
                continue;
            }
            p.consume();
        }

        return out;
    }

    inline logic_t eval_expr(compiled_module const& m, ::std::size_t root, ::fast_io::vector<logic_t> const& values) noexcept
    {
        auto const& n{m.expr_nodes.index_unchecked(root)};
        switch(n.kind)
        {
            case expr_kind::literal: return n.literal;
            case expr_kind::signal:
            {
                if(n.signal >= values.size()) { return logic_t::indeterminate_state; }
                return values.index_unchecked(n.signal);
            }
            case expr_kind::unary_not: return logic_not(eval_expr(m, n.a, values));
            case expr_kind::binary_and: return logic_and(eval_expr(m, n.a, values), eval_expr(m, n.b, values));
            case expr_kind::binary_or: return logic_or(eval_expr(m, n.a, values), eval_expr(m, n.b, values));
            case expr_kind::binary_xor: return logic_xor(eval_expr(m, n.a, values), eval_expr(m, n.b, values));
            case expr_kind::binary_eq:
            {
                auto const a{normalize_z_to_x(eval_expr(m, n.a, values))};
                auto const b{normalize_z_to_x(eval_expr(m, n.b, values))};
                if(is_unknown(a) || is_unknown(b)) { return logic_t::indeterminate_state; }
                return a == b ? logic_t::true_state : logic_t::false_state;
            }
            case expr_kind::binary_neq:
            {
                auto const a{normalize_z_to_x(eval_expr(m, n.a, values))};
                auto const b{normalize_z_to_x(eval_expr(m, n.b, values))};
                if(is_unknown(a) || is_unknown(b)) { return logic_t::indeterminate_state; }
                return a != b ? logic_t::true_state : logic_t::false_state;
            }
            default: return logic_t::indeterminate_state;
        }
    }

    struct scheduled_event
    {
        ::std::uint64_t due_tick{};
        bool nonblocking{};
        ::std::size_t lhs_signal{SIZE_MAX};
        ::std::size_t expr_root{SIZE_MAX};
    };

    struct module_state
    {
        compiled_module const* mod{};
        ::fast_io::vector<logic_t> values{};
        ::fast_io::vector<logic_t> prev_values{};
        ::fast_io::vector<scheduled_event> events{};
        ::fast_io::vector<::std::pair<::std::size_t, logic_t>> nba_queue{};
    };

    inline void init_state(module_state& st, compiled_module const& m) noexcept
    {
        st.mod = __builtin_addressof(m);
        st.values.assign(m.signal_names.size(), logic_t::indeterminate_state);
        st.prev_values.assign(m.signal_names.size(), logic_t::indeterminate_state);
        st.events.clear();
        st.nba_queue.clear();
    }

    struct port_binding
    {
        enum class kind : ::std::uint_fast8_t
        {
            unconnected,
            parent_signal,
            literal
        };

        kind k{kind::unconnected};
        ::std::size_t parent_signal{SIZE_MAX};
        logic_t literal{logic_t::indeterminate_state};
    };

    struct instance_state
    {
        compiled_module const* mod{};
        ::fast_io::u8string instance_name{};
        module_state state{};
        // For each bit-level port in `mod->ports`, a binding into the parent instance.
        ::fast_io::vector<port_binding> bindings{};
        ::std::vector<instance_state> children{};
    };

    struct compiled_design
    {
        ::fast_io::vector<compiled_module> modules{};
        ::absl::btree_map<::fast_io::u8string, ::std::size_t> module_index{};
    };

    inline compiled_design build_design(compile_result&& cr) noexcept
    {
        compiled_design d{};
        d.modules = ::std::move(cr.modules);
        for(::std::size_t i{}; i < d.modules.size(); ++i)
        {
            auto const& nm{d.modules.index_unchecked(i).name};
            if(d.module_index.find(nm) == d.module_index.end()) { d.module_index.insert({nm, i}); }
        }
        return d;
    }

    inline compiled_module const* find_module(compiled_design const& d, ::fast_io::u8string_view name) noexcept
    {
        auto it{d.module_index.find(::fast_io::u8string{name})};
        if(it == d.module_index.end()) { return nullptr; }
        return __builtin_addressof(d.modules.index_unchecked(it->second));
    }

    inline compiled_module const* find_module(compiled_design const& d, ::fast_io::u8string const& name) noexcept
    {
        return find_module(d, ::fast_io::u8string_view{name.data(), name.size()});
    }

    namespace runtime_details
    {
        inline bool assign_signal(module_state& st, ::std::size_t sig, logic_t v) noexcept
        {
            if(sig >= st.values.size()) { return false; }
            auto& dst{st.values.index_unchecked(sig)};
            if(dst != v)
            {
                dst = v;
                return true;
            }
            return false;
        }

        inline bool exec_stmt(compiled_module const& m,
                              ::fast_io::vector<stmt_node> const& arena,
                              ::std::size_t stmt_idx,
                              module_state& st,
                              ::std::uint64_t tick,
                              bool treat_nonblocking_as_blocking) noexcept
        {
            if(stmt_idx >= arena.size()) { return false; }
            auto const& n{arena.index_unchecked(stmt_idx)};

            switch(n.k)
            {
                case stmt_node::kind::empty: return false;
                case stmt_node::kind::block:
                {
                    bool changed{};
                    for(auto const sub: n.stmts) { changed = exec_stmt(m, arena, sub, st, tick, treat_nonblocking_as_blocking) || changed; }
                    return changed;
                }
                case stmt_node::kind::blocking_assign:
                {
                    if(n.lhs_signal >= st.values.size()) { return false; }
                    if(n.delay_ticks != 0)
                    {
                        st.events.push_back({tick + n.delay_ticks, false, n.lhs_signal, n.expr_root});
                        return false;
                    }
                    auto const v{eval_expr(m, n.expr_root, st.values)};
                    return assign_signal(st, n.lhs_signal, v);
                }
                case stmt_node::kind::nonblocking_assign:
                {
                    if(n.lhs_signal >= st.values.size()) { return false; }
                    if(treat_nonblocking_as_blocking)
                    {
                        if(n.delay_ticks != 0)
                        {
                            st.events.push_back({tick + n.delay_ticks, false, n.lhs_signal, n.expr_root});
                            return false;
                        }
                        auto const v{eval_expr(m, n.expr_root, st.values)};
                        return assign_signal(st, n.lhs_signal, v);
                    }

                    if(n.delay_ticks != 0)
                    {
                        st.events.push_back({tick + n.delay_ticks, true, n.lhs_signal, n.expr_root});
                        return false;
                    }
                    auto const v{eval_expr(m, n.expr_root, st.values)};
                    st.nba_queue.push_back({n.lhs_signal, v});
                    return false;
                }
                case stmt_node::kind::if_stmt:
                {
                    auto const c{normalize_z_to_x(eval_expr(m, n.expr_root, st.values))};
                    bool const take{c == logic_t::true_state};

                    bool changed{};
                    auto const& list{take ? n.stmts : n.else_stmts};
                    for(auto const sub: list) { changed = exec_stmt(m, arena, sub, st, tick, treat_nonblocking_as_blocking) || changed; }
                    return changed;
                }
                case stmt_node::kind::case_stmt:
                {
                    auto const key{normalize_z_to_x(eval_expr(m, n.case_expr_root, st.values))};
                    case_item const* def{};
                    case_item const* match{};

                    for(auto const& ci: n.case_items)
                    {
                        if(ci.is_default)
                        {
                            def = __builtin_addressof(ci);
                            continue;
                        }
                        if(normalize_z_to_x(ci.match) == key)
                        {
                            match = __builtin_addressof(ci);
                            break;
                        }
                    }

                    auto const* chosen{match ? match : def};
                    if(chosen == nullptr) { return false; }

                    bool changed{};
                    for(auto const sub: chosen->stmts) { changed = exec_stmt(m, arena, sub, st, tick, treat_nonblocking_as_blocking) || changed; }
                    return changed;
                }
                default: return false;
            }
        }

        inline void process_due_events(module_state& st, ::std::uint64_t tick) noexcept
        {
            if(st.mod == nullptr) { return; }
            auto const& m{*st.mod};

            ::fast_io::vector<scheduled_event> keep{};
            keep.reserve(st.events.size());

            for(auto const& ev: st.events)
            {
                if(ev.due_tick > tick)
                {
                    keep.push_back(ev);
                    continue;
                }

                if(ev.lhs_signal >= st.values.size()) { continue; }

                if(ev.nonblocking)
                {
                    st.nba_queue.push_back({ev.lhs_signal, eval_expr(m, ev.expr_root, st.values)});
                }
                else
                {
                    (void)assign_signal(st, ev.lhs_signal, eval_expr(m, ev.expr_root, st.values));
                }
            }

            st.events = ::std::move(keep);
        }

        inline bool apply_nba(module_state& st) noexcept
        {
            bool changed{};
            for(auto const& [lhs, v]: st.nba_queue) { changed = assign_signal(st, lhs, v) || changed; }
            st.nba_queue.clear();
            return changed;
        }

        inline void run_sequential(instance_state& inst, ::std::uint64_t tick) noexcept
        {
            if(inst.mod == nullptr) { return; }
            auto const& m{*inst.mod};
            auto& st{inst.state};

            for(auto const& ff: m.always_ffs)
            {
                if(ff.clk_signal >= st.values.size() || ff.clk_signal >= st.prev_values.size()) { continue; }

                auto const clk_prev{normalize_z_to_x(st.prev_values.index_unchecked(ff.clk_signal))};
                auto const clk_now{normalize_z_to_x(st.values.index_unchecked(ff.clk_signal))};

                bool fire{};
                if(ff.posedge) { fire = (clk_prev == logic_t::false_state) && (clk_now == logic_t::true_state); }
                else { fire = (clk_prev == logic_t::true_state) && (clk_now == logic_t::false_state); }

                if(!fire) { continue; }

                for(auto const root: ff.roots) { (void)exec_stmt(m, ff.stmt_nodes, root, st, tick, false); }
            }
        }

        inline bool run_comb_local(instance_state& inst, ::std::uint64_t tick) noexcept
        {
            if(inst.mod == nullptr) { return false; }
            auto const& m{*inst.mod};
            auto& st{inst.state};

            bool changed{};
            for(auto const& ac: m.always_combs)
            {
                for(auto const root: ac.roots) { changed = exec_stmt(m, ac.stmt_nodes, root, st, tick, true) || changed; }
            }

            for(auto const& a: m.assigns)
            {
                auto const v{eval_expr(m, a.expr_root, st.values)};
                changed = assign_signal(st, a.lhs_signal, v) || changed;
            }

            return changed;
        }

        inline bool propagate_parent_to_child(instance_state& parent, instance_state& child) noexcept
        {
            if(child.mod == nullptr) { return false; }

            auto const& cm{*child.mod};
            auto& cst{child.state};

            bool changed{};
            ::std::size_t const nports{cm.ports.size()};
            if(child.bindings.size() < nports) { return false; }

            for(::std::size_t i{}; i < nports; ++i)
            {
                auto const& p{cm.ports.index_unchecked(i)};
                if(p.dir == port_dir::output) { continue; }

                logic_t v{logic_t::indeterminate_state};
                auto const& b{child.bindings.index_unchecked(i)};
                if(b.k == port_binding::kind::parent_signal)
                {
                    if(b.parent_signal < parent.state.values.size()) { v = parent.state.values.index_unchecked(b.parent_signal); }
                }
                else if(b.k == port_binding::kind::literal) { v = b.literal; }

                changed = assign_signal(cst, p.signal, v) || changed;
            }

            return changed;
        }

        inline bool propagate_child_to_parent(instance_state& child, instance_state& parent) noexcept
        {
            if(child.mod == nullptr) { return false; }

            auto const& cm{*child.mod};
            auto& cst{child.state};

            bool changed{};
            ::std::size_t const nports{cm.ports.size()};
            if(child.bindings.size() < nports) { return false; }

            for(::std::size_t i{}; i < nports; ++i)
            {
                auto const& p{cm.ports.index_unchecked(i)};
                if(p.dir == port_dir::input) { continue; }

                auto const& b{child.bindings.index_unchecked(i)};
                if(b.k != port_binding::kind::parent_signal) { continue; }
                if(b.parent_signal >= parent.state.values.size() || p.signal >= cst.values.size()) { continue; }

                changed = assign_signal(parent.state, b.parent_signal, cst.values.index_unchecked(p.signal)) || changed;
            }

            return changed;
        }

        inline void sequential_pass(instance_state& inst, ::std::uint64_t tick, bool process_sequential) noexcept
        {
            inst.state.nba_queue.clear();
            process_due_events(inst.state, tick);
            if(process_sequential) { run_sequential(inst, tick); }

            for(auto& child: inst.children)
            {
                (void)propagate_parent_to_child(inst, child);
                sequential_pass(child, tick, process_sequential);
            }
        }

        inline bool comb_resolve(instance_state& inst, ::std::uint64_t tick) noexcept
        {
            constexpr ::std::size_t max_iter{64};
            bool any_changed{};

            for(::std::size_t iter{}; iter < max_iter; ++iter)
            {
                bool changed{};

                for(auto& child: inst.children) { changed = propagate_parent_to_child(inst, child) || changed; }
                for(auto& child: inst.children) { changed = comb_resolve(child, tick) || changed; }
                for(auto& child: inst.children) { changed = propagate_child_to_parent(child, inst) || changed; }

                changed = run_comb_local(inst, tick) || changed;

                if(!changed) { break; }
                any_changed = true;
            }

            return any_changed;
        }

        inline bool apply_nba_recursive(instance_state& inst) noexcept
        {
            bool changed{apply_nba(inst.state)};
            for(auto& c: inst.children) { changed = apply_nba_recursive(c) || changed; }
            return changed;
        }

        inline void update_prev_recursive(instance_state& inst) noexcept
        {
            inst.state.prev_values = inst.state.values;
            for(auto& c: inst.children) { update_prev_recursive(c); }
        }

        inline void fill_bindings(compiled_module const& pm,
                                  compiled_module const& cm,
                                  instance const& idef,
                                  ::fast_io::vector<port_binding>& bindings) noexcept
        {
            // Map child base ports (cm.port_order) to connection_ref
            ::absl::btree_map<::fast_io::u8string, connection_ref> named{};
            bool any_named{};
            for(auto const& c: idef.connections)
            {
                if(!c.port_name.empty())
                {
                    any_named = true;
                    named.insert_or_assign(c.port_name, c.ref);
                }
            }

            ::std::size_t positional_idx{};
            ::std::size_t port_offset{};

            for(auto const& base: cm.port_order)
            {
                connection_ref ref{};
                if(any_named)
                {
                    auto it{named.find(base)};
                    if(it != named.end()) { ref = it->second; }
                }
                else
                {
                    if(positional_idx < idef.connections.size()) { ref = idef.connections.index_unchecked(positional_idx).ref; }
                    ++positional_idx;
                }

                port_decl pd{};
                auto itpd{cm.port_decls.find(base)};
                if(itpd != cm.port_decls.end()) { pd = itpd->second; }

                ::std::size_t width{1};
                if(pd.has_range) { width = static_cast<::std::size_t>((pd.msb - pd.lsb) >= 0 ? (pd.msb - pd.lsb + 1) : (pd.lsb - pd.msb + 1)); }

                // helper to bind one bit
                auto bind_bit = [&](::std::size_t port_bit, port_binding b) noexcept
                {
                    if(port_bit < bindings.size()) { bindings.index_unchecked(port_bit) = b; }
                };

                if(width == 1)
                {
                    if(ref.kind == connection_kind::scalar)
                    {
                        bind_bit(port_offset, {port_binding::kind::parent_signal, ref.scalar_signal, {}});
                    }
                    else if(ref.kind == connection_kind::literal)
                    {
                        bind_bit(port_offset, {port_binding::kind::literal, SIZE_MAX, ref.literal});
                    }
                    else if(ref.kind == connection_kind::vector)
                    {
                        auto itv{pm.vectors.find(ref.vector_base)};
                        if(itv != pm.vectors.end() && details::vector_width(itv->second) == 1)
                        {
                            bind_bit(port_offset, {port_binding::kind::parent_signal, itv->second.bits.front_unchecked(), {}});
                        }
                    }
                }
                else
                {
                    if(ref.kind == connection_kind::vector)
                    {
                        auto itv{pm.vectors.find(ref.vector_base)};
                        if(itv != pm.vectors.end() && details::vector_width(itv->second) == width)
                        {
                            for(::std::size_t pos{}; pos < width; ++pos)
                            {
                                bind_bit(port_offset + pos, {port_binding::kind::parent_signal, itv->second.bits.index_unchecked(pos), {}});
                            }
                        }
                    }
                }

                port_offset += width;
            }
        }

        inline void elaborate_children(compiled_design const& d, instance_state& parent, ::std::size_t depth) noexcept
        {
            if(parent.mod == nullptr) { return; }
            if(depth > 64) { return; }

            auto const& pm{*parent.mod};
            for(auto const& idef: pm.instances)
            {
                auto const* cm{find_module(d, idef.module_name)};
                if(cm == nullptr) { continue; }

                instance_state child{};
                child.mod = cm;
                child.instance_name = idef.instance_name;
                init_state(child.state, *cm);
                child.bindings.assign(cm->ports.size(), {});

                fill_bindings(pm, *cm, idef, child.bindings);
                elaborate_children(d, child, depth + 1);

                parent.children.push_back(::std::move(child));
            }
        }
    }  // namespace runtime_details

    inline instance_state elaborate(compiled_design const& d, compiled_module const& top) noexcept
    {
        instance_state root{};
        root.mod = __builtin_addressof(top);
        root.instance_name = top.name;
        init_state(root.state, top);
        runtime_details::elaborate_children(d, root, 0);
        return root;
    }

    inline void simulate(instance_state& top, ::std::uint64_t tick, bool process_sequential) noexcept
    {
        runtime_details::sequential_pass(top, tick, process_sequential);
        (void)runtime_details::comb_resolve(top, tick);
        (void)runtime_details::apply_nba_recursive(top);
        (void)runtime_details::comb_resolve(top, tick);
        runtime_details::update_prev_recursive(top);
    }

    inline void simulate(instance_state& top, ::std::uint64_t tick) noexcept { simulate(top, tick, true); }

}  // namespace phy_engine::verilog::digital
