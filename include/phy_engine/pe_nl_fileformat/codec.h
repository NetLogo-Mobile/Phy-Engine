#pragma once

#include <bit>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>
#include <string_view>
#include <type_traits>

#include <fast_io/fast_io_dsal/string.h>
#include <fast_io/fast_io_dsal/string_view.h>
#include <fast_io/fast_io_dsal/vector.h>

#include "status.h"

namespace phy_engine::pe_nl_fileformat
{
    namespace details
    {
        inline void append_bytes(std::string& out, void const* p, std::size_t n)
        {
            auto const* b = static_cast<char const*>(p);
            out.append(b, b + n);
        }

        template <typename T>
        inline void append_trivial(std::string& out, T const& v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            append_bytes(out, &v, sizeof(T));
        }

        inline void append_u8(std::string& out, std::uint8_t v) { out.push_back(static_cast<char>(v)); }

        inline void append_uleb128(std::string& out, std::uint64_t v)
        {
            while(v >= 0x80u)
            {
                append_u8(out, static_cast<std::uint8_t>(v) | 0x80u);
                v >>= 7u;
            }
            append_u8(out, static_cast<std::uint8_t>(v));
        }

        inline status read_uleb128(std::string_view in, std::size_t& off, std::uint64_t& v)
        {
            v = 0;
            unsigned shift = 0;
            for(;;)
            {
                if(off >= in.size()) { return {errc::corrupt, "unexpected EOF while reading varint"}; }
                auto const b = static_cast<std::uint8_t>(in[off++]);
                v |= (static_cast<std::uint64_t>(b & 0x7fu) << shift);
                if((b & 0x80u) == 0) { break; }
                shift += 7;
                if(shift >= 64) { return {errc::corrupt, "varint overflow"}; }
            }
            return {};
        }

        template <typename T>
        inline status read_trivial(std::string_view in, std::size_t& off, T& v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            if(in.size() - off < sizeof(T)) { return {errc::corrupt, "unexpected EOF while reading fixed-size value"}; }
            std::memcpy(&v, in.data() + off, sizeof(T));
            off += sizeof(T);
            return {};
        }

        inline void append_f64(std::string& out, double x)
        {
            static_assert(std::numeric_limits<double>::is_iec559);
            std::uint64_t u{};
            std::memcpy(&u, &x, sizeof(u));
            if constexpr(std::endian::native == std::endian::little)
            {
                append_trivial(out, u);
            }
            else
            {
                std::uint64_t r{};
                for(int i = 0; i < 8; ++i) { r = (r << 8) | ((u >> (8 * i)) & 0xffu); }
                append_trivial(out, r);
            }
        }

        inline status read_f64(std::string_view in, std::size_t& off, double& x)
        {
            std::uint64_t u{};
            auto st = read_trivial(in, off, u);
            if(!st) { return st; }
            if constexpr(std::endian::native != std::endian::little)
            {
                std::uint64_t r{};
                for(int i = 0; i < 8; ++i) { r = (r << 8) | ((u >> (8 * i)) & 0xffu); }
                u = r;
            }
            std::memcpy(&x, &u, sizeof(x));
            return {};
        }

        inline void append_string(std::string& out, std::string_view s)
        {
            append_uleb128(out, s.size());
            append_bytes(out, s.data(), s.size());
        }

        inline status read_string(std::string_view in, std::size_t& off, std::string& s)
        {
            std::uint64_t n{};
            auto st = read_uleb128(in, off, n);
            if(!st) { return st; }
            if(n > (in.size() - off)) { return {errc::corrupt, "invalid string length"}; }
            s.assign(in.data() + off, in.data() + off + static_cast<std::size_t>(n));
            off += static_cast<std::size_t>(n);
            return {};
        }

        inline std::string u8sv_to_bytes(::fast_io::u8string_view v)
        {
            std::string out{};
            out.reserve(v.size());
            auto const* p = v.data();
            for(std::size_t i{}; i < v.size(); ++i) { out.push_back(static_cast<char>(p[i])); }
            return out;
        }

        inline ::fast_io::u8string bytes_to_u8string(std::string_view v)
        {
            ::fast_io::u8string out{};
            out.reserve(v.size());
            for(char c: v) { out.push_back(static_cast<char8_t>(static_cast<unsigned char>(c))); }
            return out;
        }

        template <typename T>
        inline void append_trivial_vector(std::string& out, ::fast_io::vector<T> const& v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            append_uleb128(out, v.size());
            if(!v.empty()) { append_bytes(out, v.data(), v.size() * sizeof(T)); }
        }

        template <typename T>
        inline status read_trivial_vector(std::string_view in, std::size_t& off, ::fast_io::vector<T>& v)
        {
            static_assert(std::is_trivially_copyable_v<T>);
            std::uint64_t n{};
            auto st = read_uleb128(in, off, n);
            if(!st) { return st; }
            if(n > (std::numeric_limits<std::size_t>::max() / sizeof(T))) { return {errc::corrupt, "vector length overflow"}; }
            std::size_t const bytes = static_cast<std::size_t>(n) * sizeof(T);
            if(bytes > (in.size() - off)) { return {errc::corrupt, "unexpected EOF while reading vector"}; }
            v.resize(static_cast<std::size_t>(n));
            if(bytes != 0) { std::memcpy(v.data(), in.data() + off, bytes); }
            off += bytes;
            return {};
        }
    }  // namespace details
}  // namespace phy_engine::pe_nl_fileformat

