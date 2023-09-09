/**************
 * Phy Engine *
 *************/

#include "io_observer.h"
#include "../src/phy_engine/devices/native_io.h"
/*
*  You can define your own io observer here
*/

#if defined(PHY_ENGINE_USE_CUSTOM_IO_OBSERVER) || !(((__STDC_HOSTED__ == 1 && (!defined(_GLIBCXX_HOSTED) || _GLIBCXX_HOSTED == 1) && !defined(_LIBCPP_FREESTANDING)) || defined(FAST_IO_ENABLE_HOSTED_FEATURES)) && __has_include(<stdio.h>))

custom_io_observer(::phy_engine::native_io::u8in){};
custom_io_observer(::phy_engine::native_io::u8out){};
custom_io_observer(::phy_engine::native_io::u8err){};

#endif