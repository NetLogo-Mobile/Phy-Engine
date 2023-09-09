/**************
 * Phy Engine *
 *************/

#include "../../phy_engine_utils/fast_io/fast_io.h"
#include "native_io.h"

#if !defined(PHY_ENGINE_USE_CUSTOM_IO_OBSERVER) && ((__STDC_HOSTED__ == 1 && (!defined(_GLIBCXX_HOSTED) || _GLIBCXX_HOSTED == 1) && !defined(_LIBCPP_FREESTANDING)) || defined(FAST_IO_ENABLE_HOSTED_FEATURES)) && __has_include(<stdio.h>)
#ifndef __AVR__
::fast_io::u8native_io_observer(::phy_engine::native_io::u8in){::fast_io::u8in()};
::fast_io::u8native_io_observer(::phy_engine::native_io::u8out){::fast_io::u8out()};
::fast_io::u8native_io_observer(::phy_engine::native_io::u8err){::fast_io::u8err()};
#else
::fast_io::u8c_io_observer(::phy_engine::native_io::u8in){::fast_io::u8c_stdin()};
::fast_io::u8c_io_observer(::phy_engine::native_io::u8out){::fast_io::u8c_stdout()};
::fast_io::u8c_io_observer(::phy_engine::native_io::u8err){::fast_io::u8c_stderr()};
#endif
#endif