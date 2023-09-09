/**************
 * Phy Engine *
 *************/

#pragma once

#include "../../phy_engine_utils/fast_io/fast_io_device.h"

namespace phy_engine::native_io {
#if !defined(PHY_ENGINE_USE_CUSTOM_IO_OBSERVER) && ((__STDC_HOSTED__ == 1 && (!defined(_GLIBCXX_HOSTED) || _GLIBCXX_HOSTED == 1) && !defined(_LIBCPP_FREESTANDING)) || defined(FAST_IO_ENABLE_HOSTED_FEATURES)) && __has_include(<stdio.h>)
#ifndef __AVR__
inline ::fast_io::u8native_io_observer u8in{::fast_io::u8in()};
inline ::fast_io::u8native_io_observer u8out{::fast_io::u8out()};
inline ::fast_io::u8native_io_observer u8err{::fast_io::u8err()};
#else
inline ::fast_io::u8c_io_observer u8in{::fast_io::u8c_stdin()};
inline ::fast_io::u8c_io_observer u8out{::fast_io::u8c_stdout()};
inline ::fast_io::u8c_io_observer u8err{::fast_io::u8c_stderr()};
#endif
#else
#include "../../../customize/io_observer.h"
#endif
}
