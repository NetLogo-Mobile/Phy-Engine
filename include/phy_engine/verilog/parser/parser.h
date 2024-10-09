#pragma once
#include <fast_io/fast_io.h>
#include "../ast/module.h"

namespace phy_engine::verilog
{
    namespace details
    {
        template <typename type, ::std::size_t size>
        inline consteval ::fast_io::intrinsics::simd_vector<type, size> generate_vector_of_same_var(type t) noexcept
        {
#if __has_cpp_attribute(__gnu__::__vector_size__)
            if constexpr(size == 64)
            {
                return ::fast_io::intrinsics::simd_vector<type, size>{t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t,
                                                                      t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t,
                                                                      t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t};
            }
            else if constexpr(size == 32)
            {
                return ::fast_io::intrinsics::simd_vector<type, size>{t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t,
                                                                      t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t};
            }
            else if constexpr(size == 16) { return ::fast_io::intrinsics::simd_vector<type, size>{t, t, t, t, t, t, t, t, t, t, t, t, t, t, t, t}; }
            else if constexpr(size == 8) { return ::fast_io::intrinsics::simd_vector<type, size>{t, t, t, t, t, t, t, t}; }
            else if constexpr(size == 4) { return ::fast_io::intrinsics::simd_vector<type, size>{t, t, t, t}; }
            else if constexpr(size == 2) { return ::fast_io::intrinsics::simd_vector<type, size>{t, t}; }
            else if constexpr(size == 1) { return ::fast_io::intrinsics::simd_vector<type, size>{t}; }
#else
            ::fast_io::intrinsics::simd_vector<type, size> res{};
            auto data{res.value};
            for(::std::size_t i{}; i < size; i++) { data[i] = t; }
            return res;
#endif
        }
    }  // namespace details

    template <bool check_bound = false>  // only need to check when loading file using normal allocator not mmap
    inline constexpr ::phy_engine::verilog::Verilog_module parser_file(char8_t const* begin, char8_t const* end) noexcept
    {
        ::phy_engine::verilog::Verilog_module vmod{};

        char8_t const* curr{begin};
        bool is_new_line{true};

        if constexpr(check_bound)
        {
            if(end <= begin) [[unlikely]] { return {}; }
        }

        constexpr auto vec_size{::fast_io::intrinsics::optimal_simd_vector_run_with_cpu_instruction_size_with_mask_countr};

        if constexpr(vec_size != 0)
        {
            constexpr auto char8_t_vec_size{vec_size / sizeof(char8_t)};

            using char8_t_vec_t = ::fast_io::intrinsics::simd_vector<char8_t, char8_t_vec_size>;

            // cspace
            // space
            constexpr auto f_vec{details::generate_vector_of_same_var<char8_t, char8_t_vec_size>(u8'\f')};     // 12
            constexpr auto t_vec{details::generate_vector_of_same_var<char8_t, char8_t_vec_size>(u8'\t')};     // 9
            constexpr auto v_vec{details::generate_vector_of_same_var<char8_t, char8_t_vec_size>(u8'\v')};     // 11
            constexpr auto space_vec{details::generate_vector_of_same_var<char8_t, char8_t_vec_size>(u8' ')};  // 32

            // line
            constexpr auto n_vec{details::generate_vector_of_same_var<char8_t, char8_t_vec_size>(u8'\n')};  // 10
            constexpr auto r_vec{details::generate_vector_of_same_var<char8_t, char8_t_vec_size>(u8'\r')};  // 13

            char8_t_vec_t str_vec{};

            ::std::size_t last_space_length{};

            for(; curr < end;)
            {
#if 0
                // No need to use in the case of mmap

                if constexpr(check_bound)
                {
                    if(end - curr <= char8_t_vec_size) [[unlikely]]
                    {
                        ::fast_io::fast_terminate();
                    }
                }
#endif
                str_vec.load(curr);
                auto const is_cspace{((str_vec >= t_vec) & (str_vec <= r_vec)) | (str_vec == space_vec)};
                auto const rone{::fast_io::intrinsics::vector_mask_countr_one(is_cspace)}; // space len
                auto const rzero{::fast_io::intrinsics::vector_mask_countr_zero(is_cspace)}; // next word

                curr += rone; // skip the space

                if(rone != char8_t_vec_size)
                {
                    last_space_length += rone;

                    // is pretreatment
                    if(*curr == u8'`') [[unlikely]]
                    {
                        bool is_valid_pretreatment_line{};
                        if(last_space_length != 0)
                        {
                            auto const space_begin{curr - last_space_length};

                            if(space_begin == begin)  // valid pretreatment line
                            {
                                is_valid_pretreatment_line = true;
                            }
                            else
                            {
                                // find \n or \r in c_space vec
                                char8_t_vec_t space_vec{};

                                ::std::size_t last_ln_length{};

                                for(auto space_curr{space_begin}; space_curr < curr;)
                                {
                                    space_vec.load(space_curr);
                                    auto const is_ln{(space_vec == n_vec) | (space_vec == r_vec)};
                                    auto const rone{::fast_io::intrinsics::vector_mask_countr_zero(is_ln)};  // next ln
                                    last_ln_length += rone;
                                    space_curr += char8_t_vec_size;
                                    if(rone != char8_t_vec_size) { break; }
                                }
                                if(last_ln_length < last_space_length) // Is ln in the cspace
                                { 
                                    is_valid_pretreatment_line = true; 
                                }
                            }
                        }

                        if(is_valid_pretreatment_line) 
                        { 
                            // to do
                            ::fast_io::io::perr("[debug] valid!\n");
                        }
                        else 
                        {
                            // unvalid
                        }
                    }
                    else 
                    { 
                        // to do
                        ::std::size_t sz{1};
                        curr += sz;  // skip the word
                    }

                    last_space_length = 0;
                }
                else { last_space_length += char8_t_vec_size; }
            }
        }
        else {}

        return {};
    }
}  // namespace phy_engine::verilog
