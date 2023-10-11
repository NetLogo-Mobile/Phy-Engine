#include "../../../phy_engine_utils/fast_io/fast_io.h"
#include "../../devices/native_io.h"

#include "version.h"
#include "../parameters.h"
#include "../../version/phy_engine.h"
#include "../../../phy_engine_utils/ansies/impl.h"
#include "../../version/git.h"

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::version_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept{
	::fast_io::io::println(::phy_engine::u8out,
						   u8"Phy Engine\n"
#ifdef _DEBUG
						   u8"(Debug Mode)\n"
#endif  // _DEBUG
						   u8"Copyright (C) 2023-present Phy Engine Open Source Group"

						   // Version
						   u8"\nVersion: ",
						   ::phy_engine::phy_engine_version,

						   // Git
						   u8"\n",
						   ::phy_engine::git::fetch_head,

						   // Compiler
						   u8"\nCompiler: "
#ifdef __clang__
						   u8"LLVM clang " __clang_version__
#elif defined(__GNUC__) && defined(__VERSION__)
						   u8"GCC " __VERSION__
#elif defined(_MSC_VER)
						   u8"Microsoft Visual C++ ",
						   _MSC_VER,
#else
						   u8"Unknown C++ compiler"
#endif

						   // std Lib
						   u8"\nSTD Library: "
#if defined(_LIBCPP_VERSION)
						   u8"LLVM libc++ ",
						   _LIBCPP_VERSION,
#elif defined(__GLIBCXX__)
						   u8"GNU C++ Library ",
						   _GLIBCXX_RELEASE, u8" ", __GLIBCXX__,
#elif defined(_MSVC_STL_UPDATE)
						   u8"Microsoft Visual C++ STL ",
						   _MSVC_STL_UPDATE,
#else
						   u8"Unknown C++ standard library",
#endif

						   // architecture
						   u8"\nArchitecture: "
#if defined(__alpha__)
						   u8"DEC Alpha",
#elif defined(__arm64__) || defined(__aarch64__) || defined(_M_ARM64)
						   u8"ARM64"
#elif defined(__arm__) || defined(_M_ARM)
						   u8"ARM
#elif defined(__x86_64__) || defined(_M_AMD64)
						   u8"x86_64"
#elif defined(__i386__) || defined(_M_IX86)
						   u8"i386"
#elif defined(__BFIN__)
						   u8"Blackfin"
#elif defined(__convex__)
						   u8"Convex Computer"
#elif defined(__e2k__)
						   u8"E2K"
#elif defined(__IA64__) || defined(_M_IA64)
						   u8"Intel Itanium 64"
#elif defined(__loongarch__)
						   u8"LoongArch"
#elif defined(__m68k__)
						   u8"Motorola 68k"
#elif defined(__MIPS__)
						   u8"MIPS"
#elif defined(__HPPA__)
						   u8"HP/PA RISC"
#elif defined(__riscv)
						   u8"RISC-V"
#elif defined(__370__) || defined(__THW_370__)
						   u8"System/370"
#elif defined(__s390__) || defined(__s390x__)
						   u8"System/390"
#elif defined(__powerpc64__) || defined(__ppc64__) || defined(__PPC64__)
						   u8"PowerPC64"
#elif defined(__THW_RS6000) || defined(_IBMR2) || defined(_POWER) || defined(_ARCH_PWR) || defined(_ARCH_PWR2)
						   u8"RS/6000"
#elif defined(__CUDA_ARCH__)
						   u8"PTX"
#elif defined(__sparc__)
						   u8"SPARC"
#elif defined(__sh__)
						   u8"SuperH"
#elif defined(__SYSC_ZARCH__)
						   u8"z/Architecture"
#else
						   u8"Unknown Arch"
#endif

						   // SIMD
#if defined(__wasm_simd128__)
						   u8"\nSIMD: WebAssembly SIMD"
#elif defined(__ARM_NEON) || ((defined(_MSC_VER) && !defined(__clang__)) && defined(_M_ARM64) && !defined(_KERNEL_MODE))
						   u8"\nSIMD: ARM NEON"
#elif (defined(__x86_64__) || defined(_M_AMD64) || defined(__i386__) || defined(_M_IX86)) && defined(__MMX__)
						   u8"\nSIMD: "
#if defined(__MMX__)
						   u8"MMX "
#endif
#if defined(__SSE__)
						   u8"SSE "
#endif
#if defined(__SSE2__)
						   u8"SSE2 "
#endif
#if defined(__SSE3__)
						   u8"SSE3 "
#endif
#if defined(__SSSE3__)
						   u8"SSSE3 "
#endif
#if defined(__SSE4_1__)
						   u8"SSE4.1 "
#endif
#if defined(__SSE4_2__)
						   u8"SSE4.2 "
#endif
#if defined(__FMA__)
						   u8"FMA "
#endif
#if defined(__AVX__)
						   u8"AVX "
#endif
#if defined(__AVX2__)
						   u8"AVX2 "
#endif
#if defined(__MIC__)
						   u8"MIC "
#endif
#if defined(__AVX512BW__)
						   u8"AVX512BW "
#endif
#if defined(__AVX512VL__)
						   u8"AVX512VL "
#endif
#if defined(__AVX512DQ__)
						   u8"AVX512DQ "
#endif
#if defined(__AVX512F__)
						   u8"AVX512F "
#endif
#if defined(__AVX512VBMI__)
						   u8"AVX512VBMI "
#endif
#elif defined(__VECTOR4DOUBLE__) || defined(__VSX__) || (defined(__ALTIVEC__) || defined(__VEC__))
						   u8"\nSIMD: PPC SIMD"
#endif

						   // OS
						   u8"\nOS: "
#if defined(__CYGWIN__)
						   u8"Cygwin"
#elif defined(_WIN32) || defined(_WIN64) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || defined(_WIN32_WINNT) || defined(_WIN32_WINDOWS)
						   u8"Microsoft Windows"
#if defined(_WIN32_WINDOWS)
						   u8", minimum system support: "
#if _WIN32_WINDOWS >= 0x0490
						   u8"Windows ME"
#elif _WIN32_WINDOWS >= 0x0410
						   u8"Windows 98"
#elif _WIN32_WINDOWS >= 0x0400
						   u8"Windows 95"
#endif
						   u8" (",
						   ::fast_io::mnp::hex0xupper(_WIN32_WINDOWS),
						   u8")"
#elif defined(_WIN32_WINNT)
						   u8", minimum system support: "
#if _WIN32_WINNT >= 0x0A00
						   u8"Windows 10"
#elif _WIN32_WINNT >= 0x0603
						   u8"Windows 8.1"
#elif _WIN32_WINNT >= 0x0602
						   u8"Windows 8"
#elif _WIN32_WINNT >= 0x0601
						   u8"Windows 7"
#elif _WIN32_WINNT >= 0x0600
						   u8"Windows Server 2008, Windows Vista"
#elif _WIN32_WINNT >= 0x0502
						   u8"Windows Server 2003 with SP1, Windows XP with SP2"
#elif _WIN32_WINNT >= 0x0501
						   u8"Windows Server 2003, Windows XP"
#endif
						   u8" (",
						   ::fast_io::mnp::hex0xupper(_WIN32_WINNT),
						   u8")"
#endif
#elif defined(__MSDOS__)
						   u8"Microsoft Dos"

#elif defined(__linux) || defined(__linux__) || defined(__gnu_linux__)
						   u8"Linux"
#elif defined(__APPLE__) && defined(__MACH__)
#if defined(__ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__)
						   u8"iOS"
#else
						   u8"Mac OS"
#endif
#elif defined(BSD) ||  defined(_SYSTYPE_BSD) 
						   u8"BSD"
#elif defined(__VMS)
						   u8"VMS"
#elif defined(__sun)
						   u8"Solaris"
#elif defined(__QNX__) || defined(__QNXNTO__)
						   u8"QNX"
#elif defined(__BEOS__)
						   u8"BeOS"
#elif defined(AMIGA) || defined(__amigaos__)
						   u8"AmigaOS"
#elif defined(_AIX) || defined(__TOS_AIX__)
						   u8 "IBM AIX"
#elif defined(__OS400__)
						   u8"IBM OS/400"
#elif defined(__sgi)
						   u8"IRIX"
#elif defined(__hpux)
						   u8"HP-UX"
#elif defined(__HAIKU__)
						   u8"Haiku"
#elif defined(__unix) || defined(_XOPEN_SOURCE) || defined(_POSIX_SOURCE)
						   u8"Unix Environment"
#else
						   u8"Unknown OS"
#endif

#if 0
						   // mk
						   u8"\nMaths Kernel: "

						   // ff
						   u8"\nSupported file formats: "

						   // i18n
						   u8"\nI18n Information: "
#endif  // 0
	);
	return ::phy_engine::command_line::parameter_return_type::def;
}