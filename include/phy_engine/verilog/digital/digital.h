#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>

#include <absl/container/btree_map.h>

#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/vector.h>

#include "../../model/node/node.h"

namespace phy_engine::verilog::digital
{
    // This is a deliberately small, synthesizable Verilog subset intended for use as a digital "device" in Phy-Engine.
    // Supported (initial):
    // - `module` / `endmodule`
    // - Port declarations: `input` / `output` / `inout` (scalar only)
    // - `wire` / `reg` declarations (scalar only)
    // - Continuous assignment: `assign lhs = expr;`
    // - Sequential always: `always @(posedge clk)` / `always @(negedge clk)` with nonblocking assigns `<=`
    // - Expressions: identifiers, 1-bit literals, parentheses, unary `~`, binary `&` `^` `|`
    //
    // Not supported yet: vectors/buses, generate, functions/tasks, delays, blocking assignment semantics, case/if, strength, multiple drivers.

    using logic_t = ::phy_engine::model::digital_node_statement_t;

    inline constexpr logic_t normalize_z_to_x(logic_t v) noexcept
    {
        return v == logic_t::high_impedence_state ? logic_t::indeterminate_state : v;
    }

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
        inline constexpr bool is_space(char8_t c) noexcept
        {
            return c == u8' ' || c == u8'\t' || c == u8'\n' || c == u8'\r' || c == u8'\f' || c == u8'\v';
        }

        inline constexpr bool is_digit(char8_t c) noexcept { return c >= u8'0' && c <= u8'9'; }

        inline constexpr bool is_alpha(char8_t c) noexcept
        {
            return (c >= u8'a' && c <= u8'z') || (c >= u8'A' && c <= u8'Z') || c == u8'_';
        }

        inline constexpr bool is_ident_continue(char8_t c) noexcept
        {
            return is_alpha(c) || is_digit(c) || c == u8'$';
        }
    }  // namespace details

    struct lex_result
    {
        ::fast_io::vector<token> tokens{};
        ::fast_io::vector<compile_error> errors{};
    };

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
            else { ++col; }
        };

        auto make_tok = [&](token_kind k, char8_t const* b, char8_t const* en, ::std::size_t l, ::std::size_t c) noexcept
        {
            out.tokens.push_back({k, ::fast_io::u8string_view{b, static_cast<::std::size_t>(en - b)}, l, c});
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
                out.errors.push_back({::fast_io::u8string{u8"verilog preprocessor directives are not supported yet"},
                                      static_cast<::std::size_t>(p - base),
                                      l0,
                                      c0});
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
        binary_xor
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

    struct always_ff
    {
        ::std::size_t clk_signal{};
        bool posedge{true};
        ::fast_io::vector<::std::pair<::std::size_t, ::std::size_t>> assigns{};  // (lhs_signal, expr_root)
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

        ::fast_io::vector<port> ports{};

        ::fast_io::vector<::fast_io::u8string> signal_names{};
        ::fast_io::vector<bool> signal_is_reg{};
        ::absl::btree_map<::fast_io::u8string, ::std::size_t> signal_index{};

        ::fast_io::vector<expr_node> expr_nodes{};
        ::fast_io::vector<continuous_assign> assigns{};
        ::fast_io::vector<always_ff> always_ffs{};
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

        inline constexpr bool is_sym2(::fast_io::u8string_view t, char8_t a, char8_t b) noexcept
        {
            return t.size() == 2 && t[0] == a && t[1] == b;
        }

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

            void err(token const& t, ::fast_io::u8string_view msg) noexcept
            {
                errors->push_back({::fast_io::u8string{msg}, 0, t.line, t.column});
            }

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
        {
            return get_or_create_signal(m, ::fast_io::u8string_view{name.data(), name.size()});
        }

        inline void set_port_dir(compiled_module& m, ::fast_io::u8string_view port_name, port_dir d) noexcept
        {
            for(auto& p: m.ports)
            {
                if(p.name == port_name)
                {
                    p.dir = d;
                    return;
                }
            }
            // implicit port not in header
            ::std::size_t const sig{get_or_create_signal(m, port_name)};
            m.ports.push_back({::fast_io::u8string{port_name}, d, sig});
        }

        inline void set_port_dir(compiled_module& m, ::fast_io::u8string const& port_name, port_dir d) noexcept
        {
            set_port_dir(m, ::fast_io::u8string_view{port_name.data(), port_name.size()}, d);
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
                    auto const name{t.text};
                    p->consume();
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

            [[nodiscard]] ::std::size_t parse_expr() noexcept { return parse_or(); }
        };

        inline void parse_decl_list(parser& p, compiled_module& m, port_dir d) noexcept
        {
            // optional range: [msb:lsb] (ignored for now)
            if(p.accept_sym(u8'['))
            {
                while(!p.eof() && !p.accept_sym(u8']')) { p.consume(); }
            }

            for(;;)
            {
                auto const name{p.expect_ident(u8"expected identifier in declaration")};
                if(name.empty()) { return; }

                auto const sig{get_or_create_signal(m, name)};
                if(d != port_dir::unknown) { set_port_dir(m, name, d); }

                if(!p.accept_sym(u8',')) { break; }
            }
            if(!p.accept_sym(u8';')) { p.err(p.peek(), u8"expected ';'"); p.skip_until_semicolon(); }
        }

        inline void parse_wire_list(parser& p, compiled_module& m, bool is_reg) noexcept
        {
            for(;;)
            {
                auto const name{p.expect_ident(u8"expected identifier in declaration")};
                if(name.empty()) { return; }
                auto const sig{get_or_create_signal(m, name)};
                if(is_reg) { m.signal_is_reg.index_unchecked(sig) = true; }
                if(!p.accept_sym(u8',')) { break; }
            }
            if(!p.accept_sym(u8';')) { p.err(p.peek(), u8"expected ';'"); p.skip_until_semicolon(); }
        }

        inline void parse_assign_stmt(parser& p, compiled_module& m) noexcept
        {
            auto const lhs_name{p.expect_ident(u8"expected lhs identifier after 'assign'")};
            if(lhs_name.empty()) { p.skip_until_semicolon(); return; }
            if(!p.accept_sym(u8'=')) { p.err(p.peek(), u8"expected '=' in assign"); p.skip_until_semicolon(); return; }

            expr_parser ep{&p, &m};
            ::std::size_t const root{ep.parse_expr()};

            if(!p.accept_sym(u8';')) { p.err(p.peek(), u8"expected ';' after assign"); p.skip_until_semicolon(); }

            m.assigns.push_back({get_or_create_signal(m, lhs_name), root});
        }

        inline void parse_always_ff(parser& p, compiled_module& m) noexcept
        {
            // always @ ( posedge clk ) <stmt>
            if(!p.accept_sym(u8'@')) { p.err(p.peek(), u8"expected '@' after always"); p.skip_until_semicolon(); return; }
            if(!p.accept_sym(u8'(')) { p.err(p.peek(), u8"expected '(' after @"); p.skip_until_semicolon(); return; }

            bool posedge{true};
            if(p.accept_kw(u8"posedge")) { posedge = true; }
            else if(p.accept_kw(u8"negedge")) { posedge = false; }
            else { p.err(p.peek(), u8"expected posedge/negedge"); }

            auto const clk_name{p.expect_ident(u8"expected clock identifier")};
            if(clk_name.empty()) { p.skip_until_semicolon(); return; }

            if(!p.accept_sym(u8')')) { p.err(p.peek(), u8"expected ')' after event"); }

            always_ff ff{};
            ff.clk_signal = get_or_create_signal(m, clk_name);
            ff.posedge = posedge;

            auto parse_one_nb_assign = [&]() noexcept
            {
                auto const lhs{p.expect_ident(u8"expected lhs identifier in nonblocking assignment")};
                if(lhs.empty()) { p.skip_until_semicolon(); return; }
                if(!p.accept_sym2(u8'<', u8'=')) { p.err(p.peek(), u8"expected '<='"); p.skip_until_semicolon(); return; }
                expr_parser ep{&p, &m};
                ::std::size_t const root{ep.parse_expr()};
                if(!p.accept_sym(u8';')) { p.err(p.peek(), u8"expected ';'"); p.skip_until_semicolon(); }
                ff.assigns.push_back({get_or_create_signal(m, lhs), root});
            };

            if(p.accept_kw(u8"begin"))
            {
                while(!p.eof() && !p.accept_kw(u8"end"))
                {
                    parse_one_nb_assign();
                }
            }
            else
            {
                parse_one_nb_assign();
            }

            if(!ff.assigns.empty()) { m.always_ffs.push_back(::std::move(ff)); }
        }

        inline compiled_module parse_module(parser& p) noexcept
        {
            compiled_module m{};
            m.expr_nodes.reserve(128);
            m.assigns.reserve(64);
            m.always_ffs.reserve(16);

            m.name = p.expect_ident(u8"expected module name");

            if(!p.accept_sym(u8'(')) { p.err(p.peek(), u8"expected '(' after module name"); p.skip_until_semicolon(); return m; }

            // port list
            if(!p.accept_sym(u8')'))
            {
                port_dir current_dir{port_dir::unknown};

                // Parse both styles:
                // - module m(a,b,y);
                // - module m(input a, input b, output y);
                for(;;)
                {
                    if(p.accept_kw(u8"input")) { current_dir = port_dir::input; }
                    else if(p.accept_kw(u8"output")) { current_dir = port_dir::output; }
                    else if(p.accept_kw(u8"inout")) { current_dir = port_dir::inout; }

                    // optional range: [msb:lsb] (ignored for now)
                    if(p.accept_sym(u8'['))
                    {
                        while(!p.eof() && !p.accept_sym(u8']')) { p.consume(); }
                    }

                    auto const port_name{p.expect_ident(u8"expected port identifier")};
                    if(!port_name.empty())
                    {
                        ::std::size_t const sig{get_or_create_signal(m, port_name)};
                        m.ports.push_back({::fast_io::u8string{port_name}, current_dir, sig});
                    }

                    if(p.accept_sym(u8')')) { break; }
                    if(!p.accept_sym(u8',')) { p.err(p.peek(), u8"expected ',' or ')' in port list"); }

                    // If next token is a direction keyword, it starts a new declaration and resets dir.
                    auto const& nt{p.peek()};
                    if(nt.kind == token_kind::identifier &&
                       (is_kw(nt.text, u8"input") || is_kw(nt.text, u8"output") || is_kw(nt.text, u8"inout")))
                    {
                        current_dir = port_dir::unknown;
                    }
                }
            }

            if(!p.accept_sym(u8';')) { p.err(p.peek(), u8"expected ';' after module header"); p.skip_until_semicolon(); }

            // body
            while(!p.eof())
            {
                if(p.accept_kw(u8"endmodule")) { break; }

                if(p.accept_kw(u8"input")) { parse_decl_list(p, m, port_dir::input); continue; }
                if(p.accept_kw(u8"output")) { parse_decl_list(p, m, port_dir::output); continue; }
                if(p.accept_kw(u8"inout")) { parse_decl_list(p, m, port_dir::inout); continue; }

                if(p.accept_kw(u8"wire")) { parse_wire_list(p, m, false); continue; }
                if(p.accept_kw(u8"reg")) { parse_wire_list(p, m, true); continue; }

                if(p.accept_kw(u8"assign")) { parse_assign_stmt(p, m); continue; }
                if(p.accept_kw(u8"always")) { parse_always_ff(p, m); continue; }

                // unknown statement/instantiation: skip
                p.skip_until_semicolon();
            }

            return m;
        }
    }  // namespace details

    inline compile_result compile(::fast_io::u8string_view src) noexcept
    {
        compile_result out{};
        auto const lr{lex(src)};
        out.errors = lr.errors;

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
            default: return logic_t::indeterminate_state;
        }
    }

    struct module_state
    {
        compiled_module const* mod{};
        ::fast_io::vector<logic_t> values{};
        ::fast_io::vector<logic_t> prev_values{};
    };

    inline void init_state(module_state& st, compiled_module const& m) noexcept
    {
        st.mod = __builtin_addressof(m);
        st.values.assign(m.signal_names.size(), logic_t::indeterminate_state);
        st.prev_values.assign(m.signal_names.size(), logic_t::indeterminate_state);
    }

    inline bool simulate_tick(module_state& st, bool process_sequential) noexcept
    {
        if(st.mod == nullptr) [[unlikely]] { return false; }
        auto const& m{*st.mod};

        // sequential (edge triggered)
        if(process_sequential)
        {
            ::fast_io::vector<::std::pair<::std::size_t, logic_t>> pending{};
            pending.reserve(16);

            for(auto const& ff: m.always_ffs)
            {
                auto const clk_prev{st.prev_values.index_unchecked(ff.clk_signal)};
                auto const clk_now{st.values.index_unchecked(ff.clk_signal)};

                bool fire{};
                if(ff.posedge)
                {
                    fire = (normalize_z_to_x(clk_prev) == logic_t::false_state) && (normalize_z_to_x(clk_now) == logic_t::true_state);
                }
                else
                {
                    fire = (normalize_z_to_x(clk_prev) == logic_t::true_state) && (normalize_z_to_x(clk_now) == logic_t::false_state);
                }

                if(!fire) { continue; }

                for(auto const& [lhs, root]: ff.assigns)
                {
                    pending.push_back({lhs, eval_expr(m, root, st.values)});
                }
            }

            for(auto const& [lhs, v]: pending)
            {
                if(lhs < st.values.size()) { st.values.index_unchecked(lhs) = v; }
            }
        }

        // combinational (continuous assign), fixed-point
        constexpr ::std::size_t max_iter{64};
        for(::std::size_t iter{}; iter < max_iter; ++iter)
        {
            bool changed{};
            for(auto const& a: m.assigns)
            {
                auto const v{eval_expr(m, a.expr_root, st.values)};
                if(a.lhs_signal < st.values.size())
                {
                    auto& dst{st.values.index_unchecked(a.lhs_signal)};
                    if(dst != v)
                    {
                        dst = v;
                        changed = true;
                    }
                }
            }
            if(!changed) { break; }
        }

        st.prev_values = st.values;
        return true;
    }

}  // namespace phy_engine::verilog::digital
