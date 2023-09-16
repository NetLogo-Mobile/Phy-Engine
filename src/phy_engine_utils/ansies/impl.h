#pragma once

#include "colsty.h"
#include "cursor.h"

#if (defined(_WIN32) || defined(__CYGWIN__)) && !defined(_WIN32_WINDOWS)
#include "win32.h"
#endif 