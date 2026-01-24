#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "status.h"

namespace phy_engine::pe_nl_fileformat
{
    namespace details
    {
        inline constexpr std::uint64_t fnv1a_basis{14695981039346656037ull};
        inline constexpr std::uint64_t fnv1a_prime{1099511628211ull};

        inline std::uint64_t fnv1a_update(std::uint64_t h, void const* data, std::size_t n) noexcept
        {
            auto const* p = static_cast<unsigned char const*>(data);
            for(std::size_t i{}; i < n; ++i)
            {
                h ^= static_cast<std::uint64_t>(p[i]);
                h *= fnv1a_prime;
            }
            return h;
        }

        inline bool read_exact(std::istream& in, void* dst, std::size_t n)
        {
            return static_cast<bool>(in.read(static_cast<char*>(dst), static_cast<std::streamsize>(n)));
        }

        inline bool write_exact(std::ostream& out, void const* src, std::size_t n)
        {
            out.write(static_cast<char const*>(src), static_cast<std::streamsize>(n));
            return static_cast<bool>(out);
        }

        inline status write_u32(std::ostream& out, std::uint32_t v)
        {
            if(!write_exact(out, &v, sizeof(v))) { return {errc::io_error, "failed writing u32"}; }
            return {};
        }

        inline status write_u64(std::ostream& out, std::uint64_t v)
        {
            if(!write_exact(out, &v, sizeof(v))) { return {errc::io_error, "failed writing u64"}; }
            return {};
        }

        inline status read_u32(std::istream& in, std::uint32_t& v)
        {
            if(!read_exact(in, &v, sizeof(v))) { return {errc::io_error, "failed reading u32"}; }
            return {};
        }

        inline status read_u64(std::istream& in, std::uint64_t& v)
        {
            if(!read_exact(in, &v, sizeof(v))) { return {errc::io_error, "failed reading u64"}; }
            return {};
        }

        inline status write_string(std::ostream& out, std::string_view s)
        {
            auto st = write_u64(out, static_cast<std::uint64_t>(s.size()));
            if(!st) { return st; }
            if(!write_exact(out, s.data(), s.size())) { return {errc::io_error, "failed writing string bytes"}; }
            return {};
        }

        inline status read_string(std::istream& in, std::string& s, std::size_t max_len)
        {
            std::uint64_t n{};
            auto st = read_u64(in, n);
            if(!st) { return st; }
            if(n > max_len) { return {errc::corrupt, "string length too large"}; }
            s.resize(static_cast<std::size_t>(n));
            if(n != 0 && !read_exact(in, s.data(), static_cast<std::size_t>(n))) { return {errc::io_error, "failed reading string bytes"}; }
            return {};
        }

        inline status ensure_safe_relative_path(std::string_view rel)
        {
            std::filesystem::path p{std::string(rel)};
            if(p.empty()) { return {errc::corrupt, "empty path in archive"}; }
            if(p.is_absolute()) { return {errc::corrupt, "absolute paths not allowed in archive"}; }
            auto norm = p.lexically_normal();
            for(auto const& part: norm)
            {
                if(part == "..") { return {errc::corrupt, "parent path (..) not allowed in archive"}; }
            }
            return {};
        }
    }  // namespace details

    // Simple single-file container:
    // [magic 8 bytes "PENLDBA1"]
    // [u32 version=1]
    // [u64 file_count]
    // repeated file_count times:
    //   [u64 path_len][path bytes (utf-8, relative)]
    //   [u64 file_size][file bytes]
    // [u64 fnv1a_hash over all payload fields (after header) + file bytes]
    inline status pack_directory_to_file(std::filesystem::path const& dir, std::filesystem::path const& out_file)
    {
        if(!std::filesystem::exists(dir) || !std::filesystem::is_directory(dir)) { return {errc::invalid_argument, "pack: input is not a directory"}; }

        std::vector<std::filesystem::path> files{};
        for(auto const& ent: std::filesystem::recursive_directory_iterator(dir))
        {
            if(!ent.is_regular_file()) { continue; }
            files.push_back(ent.path());
        }

        std::ofstream out(out_file, std::ios::binary | std::ios::trunc);
        if(!out) { return {errc::io_error, "failed to open output file"}; }

        {
            char const magic[8] = {'P', 'E', 'N', 'L', 'D', 'B', 'A', '1'};
            if(!details::write_exact(out, magic, sizeof(magic))) { return {errc::io_error, "failed writing magic"}; }
            auto st = details::write_u32(out, 1);
            if(!st) { return st; }
            st = details::write_u64(out, static_cast<std::uint64_t>(files.size()));
            if(!st) { return st; }
        }

        std::uint64_t hash = details::fnv1a_basis;

        for(auto const& abs: files)
        {
            auto rel = std::filesystem::relative(abs, dir).generic_string();
            auto st = details::ensure_safe_relative_path(rel);
            if(!st) { return st; }

            auto const sz = std::filesystem::file_size(abs);

            // path
            {
                std::uint64_t const path_len = static_cast<std::uint64_t>(rel.size());
                st = details::write_u64(out, path_len);
                if(!st) { return st; }
                hash = details::fnv1a_update(hash, &path_len, sizeof(path_len));
                if(path_len != 0)
                {
                    if(!details::write_exact(out, rel.data(), rel.size())) { return {errc::io_error, "failed writing path bytes"}; }
                    hash = details::fnv1a_update(hash, rel.data(), rel.size());
                }
            }

            // size
            {
                std::uint64_t const file_sz = static_cast<std::uint64_t>(sz);
                st = details::write_u64(out, file_sz);
                if(!st) { return st; }
                hash = details::fnv1a_update(hash, &file_sz, sizeof(file_sz));
            }

            // bytes
            std::ifstream in(abs, std::ios::binary);
            if(!in) { return {errc::io_error, "failed to open input file while packing"}; }
            constexpr std::size_t kBuf = 1u << 20;
            std::string buf;
            buf.resize(kBuf);
            std::uint64_t remaining = static_cast<std::uint64_t>(sz);
            while(remaining != 0)
            {
                std::size_t const n = static_cast<std::size_t>(std::min<std::uint64_t>(remaining, kBuf));
                if(!details::read_exact(in, buf.data(), n)) { return {errc::io_error, "failed reading file bytes while packing"}; }
                if(!details::write_exact(out, buf.data(), n)) { return {errc::io_error, "failed writing file bytes while packing"}; }
                hash = details::fnv1a_update(hash, buf.data(), n);
                remaining -= n;
            }
        }

        auto st = details::write_u64(out, hash);
        if(!st) { return st; }
        out.flush();
        if(!out) { return {errc::io_error, "failed finalizing archive file"}; }
        return {};
    }

    inline status unpack_file_to_directory(std::filesystem::path const& in_file, std::filesystem::path const& dir)
    {
        std::ifstream in(in_file, std::ios::binary);
        if(!in) { return {errc::io_error, "failed to open archive file"}; }

        char magic[8]{};
        if(!details::read_exact(in, magic, sizeof(magic))) { return {errc::io_error, "failed reading magic"}; }
        if(std::string_view{magic, sizeof(magic)} != std::string_view{"PENLDBA1", 8})
        {
            return {errc::unsupported, "not a pe_nl single-file archive"};
        }

        std::uint32_t ver{};
        auto st = details::read_u32(in, ver);
        if(!st) { return st; }
        if(ver != 1) { return {errc::unsupported, "unsupported archive version"}; }

        std::uint64_t file_count{};
        st = details::read_u64(in, file_count);
        if(!st) { return st; }
        if(file_count > 10'000'000ull) { return {errc::corrupt, "archive file_count too large"}; }

        std::filesystem::create_directories(dir);

        std::uint64_t hash = details::fnv1a_basis;

        for(std::uint64_t i{}; i < file_count; ++i)
        {
            std::uint64_t path_len{};
            st = details::read_u64(in, path_len);
            if(!st) { return st; }
            hash = details::fnv1a_update(hash, &path_len, sizeof(path_len));

            if(path_len > 4096ull) { return {errc::corrupt, "archive path too long"}; }
            std::string rel{};
            rel.resize(static_cast<std::size_t>(path_len));
            if(path_len != 0 && !details::read_exact(in, rel.data(), static_cast<std::size_t>(path_len)))
            {
                return {errc::io_error, "failed reading path bytes"};
            }
            if(path_len != 0) { hash = details::fnv1a_update(hash, rel.data(), rel.size()); }

            st = details::ensure_safe_relative_path(rel);
            if(!st) { return st; }

            std::uint64_t file_sz{};
            st = details::read_u64(in, file_sz);
            if(!st) { return st; }
            hash = details::fnv1a_update(hash, &file_sz, sizeof(file_sz));

            auto out_path = dir / std::filesystem::path{rel};
            std::filesystem::create_directories(out_path.parent_path());

            std::ofstream out(out_path, std::ios::binary | std::ios::trunc);
            if(!out) { return {errc::io_error, "failed to create output file while unpacking"}; }

            constexpr std::size_t kBuf = 1u << 20;
            std::string buf;
            buf.resize(kBuf);
            std::uint64_t remaining = file_sz;
            while(remaining != 0)
            {
                std::size_t const n = static_cast<std::size_t>(std::min<std::uint64_t>(remaining, kBuf));
                if(!details::read_exact(in, buf.data(), n)) { return {errc::io_error, "failed reading file bytes while unpacking"}; }
                hash = details::fnv1a_update(hash, buf.data(), n);
                if(!details::write_exact(out, buf.data(), n)) { return {errc::io_error, "failed writing file bytes while unpacking"}; }
                remaining -= n;
            }
        }

        std::uint64_t stored_hash{};
        st = details::read_u64(in, stored_hash);
        if(!st) { return st; }
        if(stored_hash != hash) { return {errc::corrupt, "archive checksum mismatch"}; }

        return {};
    }
}  // namespace phy_engine::pe_nl_fileformat
