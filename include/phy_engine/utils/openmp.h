#pragma once

// Phy-Engine OpenMP configuration.
//
// Users may define `PHY_ENGINE_ENABLE_OPENMP` to 0/1 before including any
// Phy-Engine headers to force OpenMP-aware code paths on/off.
//
// If the compiler is not building this translation unit with OpenMP enabled
// (i.e. `_OPENMP` is not defined), the library always falls back to the
// single-threaded implementation regardless of `PHY_ENGINE_ENABLE_OPENMP`.

#if defined(_OPENMP)
    #define PHY_ENGINE_COMPILER_HAS_OPENMP 1
#else
    #define PHY_ENGINE_COMPILER_HAS_OPENMP 0
#endif

#ifndef PHY_ENGINE_ENABLE_OPENMP
    #define PHY_ENGINE_ENABLE_OPENMP PHY_ENGINE_COMPILER_HAS_OPENMP
#endif

// If OpenMP isn't enabled for this translation unit, force-disable it.
#if PHY_ENGINE_ENABLE_OPENMP && !PHY_ENGINE_COMPILER_HAS_OPENMP
    #undef PHY_ENGINE_ENABLE_OPENMP
    #define PHY_ENGINE_ENABLE_OPENMP 0
#endif

#if PHY_ENGINE_ENABLE_OPENMP
    #include <omp.h>
#endif

#define PHY_ENGINE_OMP_STR1(x) #x
#define PHY_ENGINE_OMP_STR(x) PHY_ENGINE_OMP_STR1(x)

#if PHY_ENGINE_ENABLE_OPENMP
    #define PHY_ENGINE_OMP_PRAGMA(x) _Pragma(PHY_ENGINE_OMP_STR(x))
    #define PHY_ENGINE_OMP_PARALLEL_FOR(...) PHY_ENGINE_OMP_PRAGMA(omp parallel for __VA_ARGS__)
#else
    #define PHY_ENGINE_OMP_PRAGMA(x)
    #define PHY_ENGINE_OMP_PARALLEL_FOR(...)
#endif

namespace phy_engine::utils
{
    [[nodiscard]] inline int omp_max_threads() noexcept
    {
#if PHY_ENGINE_ENABLE_OPENMP
        return ::omp_get_max_threads();
#else
        return 1;
#endif
    }

    [[nodiscard]] inline int omp_thread_num() noexcept
    {
#if PHY_ENGINE_ENABLE_OPENMP
        return ::omp_get_thread_num();
#else
        return 0;
#endif
    }
}  // namespace phy_engine::utils

