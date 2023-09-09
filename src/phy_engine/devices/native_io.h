/**************
 * Phy Engine *
 *************/

#pragma once

#include "../../phy_engine_utils/fast_io/fast_io_device.h"

#if defined(PHY_ENGINE_USE_CUSTOM_IO_OBSERVER) || !(((__STDC_HOSTED__ == 1 && (!defined(_GLIBCXX_HOSTED) || _GLIBCXX_HOSTED == 1) && !defined(_LIBCPP_FREESTANDING)) || defined(FAST_IO_ENABLE_HOSTED_FEATURES)) && __has_include(<stdio.h>))
class custom_io_observer;
#endif

namespace phy_engine::native_io {
#if !defined(PHY_ENGINE_USE_CUSTOM_IO_OBSERVER) && ((__STDC_HOSTED__ == 1 && (!defined(_GLIBCXX_HOSTED) || _GLIBCXX_HOSTED == 1) && !defined(_LIBCPP_FREESTANDING)) || defined(FAST_IO_ENABLE_HOSTED_FEATURES)) && __has_include(<stdio.h>)
#ifndef __AVR__
extern ::fast_io::u8native_io_observer u8in;
extern ::fast_io::u8native_io_observer u8out;
extern ::fast_io::u8native_io_observer u8err;
#else
extern ::fast_io::u8c_io_observer u8in;
extern ::fast_io::u8c_io_observer u8out;
extern ::fast_io::u8c_io_observer u8err;
#endif
#else
#include "../../../customize/io_observer.h"
extern custom_io_observer u8in;
extern custom_io_observer u8out;
extern custom_io_observer u8err;
#endif
}