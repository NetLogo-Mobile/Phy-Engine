/**************
 * Phy Engine *
 *************/

#pragma once

/*
 *  You can define your own io observer here
 */

#if defined(PHY_ENGINE_USE_CUSTOM_IO_OBSERVER) || !(((__STDC_HOSTED__ == 1 && (!defined(_GLIBCXX_HOSTED) || _GLIBCXX_HOSTED == 1) && !defined(_LIBCPP_FREESTANDING)) || defined(FAST_IO_ENABLE_HOSTED_FEATURES)) && __has_include(<stdio.h>))

// custom io observer definition
class custom_io_observer {
};

// operation function definition



#endif