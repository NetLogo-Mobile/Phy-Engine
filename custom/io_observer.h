/**************
 * Phy Engine *
 *************/

#pragma once

/*
 *  You can define your own io observer here
 *  io_observer.h will be included in namespace phy_engine::native_io
 */

// custom io observer definition
class custom_io_observer {
};

// define u8in, u8out, u8err
inline custom_io_observer u8in{};
inline custom_io_observer u8out{};
inline custom_io_observer u8err{};

// operation function definition

