#include "../../../phy_engine_utils/fast_io/fast_io.h"
#include "../../devices/native_io.h"

#include "version.h"
#include "../parameters.h"
#include "../../version/phy_engine.h"
#include "../../../phy_engine_utils/ansies/impl.h"
#include "../../version/git.h"

::phy_engine::command_line::parameter_return_type(::phy_engine::parameter::details::version_callback)(::std::size_t, ::fast_io::vector<::phy_engine::command_line::parameter_parsing_results>&) noexcept{
	::fast_io::io::println(::phy_engine::u8out,
						   ::phy_engine::ansi_escape_sequences::rst::all,
						   u8"Phy Engine\n"
						   u8"Copyright (C) 2023-present Phy Engine Open Source Group\n"
#ifdef _DEBUG
						   u8"Debug Mode\n"
#endif  // _DEBUG
						   // Version
						   u8"Version: ",
						   ::phy_engine::phy_engine_version,
						   // Get
						   u8" ",
						   ::phy_engine::git::commit_hash,
						   // Compiler
						   u8"\nCompiler: "
#ifdef __clang__
						   u8"LLVM clang " __clang_version__
						   u8"\n"
#elif defined(__GNUC__) && defined(__VERSION__)
						   u8"GCC " __VERSION__
						   u8"\n"
#elif defined(_MSC_VER)
						   u8"Microsoft Visual C++ ",
						   _MSC_VER,
						   u8"\n"
#else
						   u8"Unknown C++ compiler\n"
#endif
						   // Lib
						   u8"Lib: "
#if defined(_LIBCPP_VERSION)
						   u8"LLVM libc++ ",
						   _LIBCPP_VERSION
#elif defined(__GLIBCXX__)
						   u8"GNU C++ Library ",
						   _GLIBCXX_RELEASE, u8" ", __GLIBCXX__
#elif defined(_MSVC_STL_UPDATE)
						   u8"Microsoft Visual C++ STL ",
						   _MSVC_STL_UPDATE
#else
						   u8"Unknown C++ standard library"
#endif
#if 0
						   // Host
						   u8"\nHost: "
#if defined(__x86_64__)
						   u8"x86_64"
#elif (defined(_M_IX86) || defined(__i386__))
						   u8"i386"
#elif defined(__x86__)
						   u8"x86"
#endif
						   u8"-"
#if defined(__WIN64__)
						   u8"w64"
#elif defined(__WIN32__)
						   u8"w32"
#endif
						   u8"-"
#if defined(__MINGW64__)
						   u8"mingw64"
#elif defined(__MINGW32__)
						   u8"mingw"
#endif
						   // ISA
						   u8"\nISA: "
#if defined(__x86_64__)
						   u8"x86_64 "
#elif (defined(_M_IX86) || defined(__i386__))
						   u8"i386 "
#elif defined(__x86__)
						   u8"x86 "
#endif
#if defined(__SSE__)
						   "SSE "
#endif

#endif  // 0
	);
	return ::phy_engine::command_line::parameter_return_type::def;
}