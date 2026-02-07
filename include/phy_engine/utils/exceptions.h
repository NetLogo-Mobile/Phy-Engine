#pragma once

// Phy-Engine exception configuration.
//
// Users may define `PHY_ENGINE_ENABLE_EXCEPTIONS` to 0/1 before including any
// Phy-Engine headers to force exception-aware code paths on/off.
//
// If the compiler is built without exception support (`__cpp_exceptions` etc.),
// the library always falls back to the error-code/no-exception implementation
// regardless of `PHY_ENGINE_ENABLE_EXCEPTIONS`.

// Does the compiler support C++ exceptions for this translation unit?
#if defined(_MSC_VER)
    #if defined(_CPPUNWIND) && _CPPUNWIND
        #define PHY_ENGINE_COMPILER_HAS_EXCEPTIONS 1
    #else
        #define PHY_ENGINE_COMPILER_HAS_EXCEPTIONS 0
    #endif
#else
    #if defined(__cpp_exceptions) || defined(__EXCEPTIONS)
        #define PHY_ENGINE_COMPILER_HAS_EXCEPTIONS 1
    #else
        #define PHY_ENGINE_COMPILER_HAS_EXCEPTIONS 0
    #endif
#endif

// Library-level switch (default: follow compiler support).
#ifndef PHY_ENGINE_ENABLE_EXCEPTIONS
    #define PHY_ENGINE_ENABLE_EXCEPTIONS PHY_ENGINE_COMPILER_HAS_EXCEPTIONS
#endif

// If exceptions are not supported by the compiler, force-disable them.
#if PHY_ENGINE_ENABLE_EXCEPTIONS && !PHY_ENGINE_COMPILER_HAS_EXCEPTIONS
    #undef PHY_ENGINE_ENABLE_EXCEPTIONS
    #define PHY_ENGINE_ENABLE_EXCEPTIONS 0
#endif

#if PHY_ENGINE_ENABLE_EXCEPTIONS
    #define PHY_ENGINE_TRY try
    #define PHY_ENGINE_CATCH(...) catch(__VA_ARGS__)
    #define PHY_ENGINE_CATCH_ALL catch(...)
#else
    #define PHY_ENGINE_TRY if(true)
    #define PHY_ENGINE_CATCH(...) if(false)
    #define PHY_ENGINE_CATCH_ALL if(false)
#endif

