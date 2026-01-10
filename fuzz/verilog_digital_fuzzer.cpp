#include <cstddef>
#include <cstdint>

#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/string_view.h>

#include <phy_engine/verilog/digital/digital.h>

namespace
{
    using namespace ::phy_engine::verilog::digital;
    using u8sv = ::fast_io::u8string_view;

    struct include_pack
    {
        u8sv a{};
        u8sv b{};
        u8sv c{};
    };

    inline u8sv as_u8sv(::std::uint8_t const* data, ::std::size_t size) noexcept
    {
        return u8sv{reinterpret_cast<char8_t const*>(data), size};
    }

    inline include_pack split_includes(u8sv all, u8sv& main) noexcept
    {
        auto take = [&](u8sv src, ::std::size_t idx) noexcept -> u8sv
        {
            if(idx > src.size()) { return {}; }
            return u8sv{src.data(), idx};
        };

        auto drop = [&](u8sv src, ::std::size_t idx) noexcept -> u8sv
        {
            if(idx > src.size()) { return {}; }
            return u8sv{src.data() + idx, src.size() - idx};
        };

        // Layout: main\0a\0b\0c (remaining segments optional).
        include_pack out{};

        u8sv rest{all};
        auto next = [&](u8sv& seg) noexcept
        {
            ::std::size_t pos{};
            while(pos < rest.size() && rest[pos] != u8'\0') { ++pos; }
            seg = take(rest, pos);
            rest = (pos < rest.size()) ? drop(rest, pos + 1) : u8sv{};
        };

        next(main);
        next(out.a);
        next(out.b);
        next(out.c);
        return out;
    }

    bool include_resolver(void* user, u8sv path, ::fast_io::u8string& out_text) noexcept
    {
        auto const* pk{static_cast<include_pack const*>(user)};
        if(pk == nullptr) { return false; }

        // Keep path matching deliberately small/deterministic so the fuzzer can discover it.
        if(path == u8sv{u8"a.vh"})
        {
            out_text.assign(pk->a);
            return true;
        }
        if(path == u8sv{u8"b.vh"})
        {
            out_text.assign(pk->b);
            return true;
        }
        if(path == u8sv{u8"c.vh"})
        {
            out_text.assign(pk->c);
            return true;
        }

        return false;
    }
}  // namespace

extern "C" int LLVMFuzzerTestOneInput(::std::uint8_t const* data, ::std::size_t size)
{
    if(data == nullptr || size == 0) { return 0; }
    if(size > (1u << 20)) { return 0; }  // keep worst-case allocations bounded

    u8sv main{};
    auto const pk{split_includes(as_u8sv(data, size), main)};

    compile_options opt{};
    opt.preprocess.user = const_cast<include_pack*>(__builtin_addressof(pk));
    opt.preprocess.include_resolver = &include_resolver;
    opt.preprocess.include_depth_limit = 8;

    // Exercise preprocessing alone (including `include parsing).
    (void)preprocess(main, opt.preprocess);

    // Exercise full pipeline: preprocess -> lex -> parse -> compile.
    auto cr{compile(main, opt)};
    if(cr.modules.empty()) { return 0; }

    auto d{build_design(::std::move(cr))};
    if(d.modules.empty()) { return 0; }

    // Pick a stable top if present; otherwise just take the first module.
    compiled_module const* top{find_module(d, u8sv{u8"top"})};
    if(top == nullptr) { top = __builtin_addressof(d.modules.front_unchecked()); }

    auto inst{elaborate(d, *top)};

    // A couple of ticks to exercise scheduling and net resolution paths.
    simulate(inst, 1, true);
    simulate(inst, 2, false);

    return 0;
}

