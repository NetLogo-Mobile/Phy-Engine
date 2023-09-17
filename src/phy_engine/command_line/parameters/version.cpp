#include "../../../phy_engine_utils/fast_io/fast_io.h"
#include "../../devices/native_io.h"

#include "version.h"
#include "../parameters.h"
#include "../../version/phy_engine.h"

bool(::phy_engine::parameter::details::version_callback)(int argc, char8_t** argv, int pos, ::std::u8string_view var) noexcept {
	::fast_io::io::println(::phy_engine::u8out,
						   ::phy_engine::ansi_escape_sequences::rst::all,
						   u8"Phy Engine (Copyright 2023-present Phy Engine Open Source Group)\n"
						   u8"Version: ",
						   ::phy_engine::phy_engine_version,
						   u8"\nCompiler: "
#ifdef __clang__
						   "LLVM clang " __clang_version__
						   "\n"
#elif defined(__GNUC__) && defined(__VERSION__)
						   "GCC " __VERSION__
						   "\n"
#elif defined(_MSC_VER)
						   "Microsoft Visual C++ ", _MSC_VER,
						   u8"\n"
#else
						   "Unknown C++ compiler\n"
#endif
						   "STL: "
#if defined(_LIBCPP_VERSION)
						   "LLVM libc++ ",
						   _LIBCPP_VERSION
#elif defined(__GLIBCXX__)
						   "GNU C++ Library ",
						   _GLIBCXX_RELEASE, u8" ", __GLIBCXX__
#elif defined(_MSVC_STL_UPDATE)
						   "Microsoft Visual C++ STL ",
						   _MSVC_STL_UPDATE
#else
						   "Unknown C++ standard library"
#endif
	);
	return true;
}