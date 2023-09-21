set_xmakever("2.5.6")

set_project("Phy Engine")

set_version("1.0.0", {build = "%Y%m%d"})

set_allowedplats("windows", "mingw", "linux", "msdosdjgpp", "android", "freebds")

--find_tool("git", {version = true})

add_rules("mode.debug", "mode.release")

set_allowedmodes("release", "debug")

set_languages("c11", "cxx23")

set_kind("binary")

set_encodings("utf-8")

if is_mode("release") then
    set_optimize("aggressive")
    set_strip("all")
elseif is_mode("debug") then
    set_optimize("none")
    set_symbols("debug")
    add_defines("_DEBUG")
end

set_defaultarchs("msdos|i386")

if is_plat("windows") then
    set_allowedarchs("x64", "x86", "ARM64", "ARM")

    if is_mode("debug") then
        set_runtimes("MTd")
    else
        set_fpmodels("fast")
        add_cxflags("-GL")
        set_runtimes("MT")
    end
    add_cxflags("-GR-")
elseif is_plat("mingw") then
    add_cxflags("-fno-rtti")
    if is_mode("release") then
        add_cxflags("-flto")
    end
    add_cxflags("-static-libstdc++")
    add_syslinks("ntdll")
elseif is_plat("linux") then
    add_cxflags("-fno-rtti")
    if is_mode("release") then
        add_cxflags("-flto")
    end

    add_cxflags("-static-libstdc++")

    if is_arch("x86_64") then
        -- none
    elseif is_arch("i386") then
        -- none
    elseif is_arch("loongarch64") then
        -- none
    end
elseif is_plat("android") then
    --none
elseif is_plat("msdosdjgpp") then
    set_allowedarchs("i386") -- x86 ms-dos not support (out of memory)

    add_cxflags("-fno-rtti")
    if is_mode("release") then
        add_cxflags("-flto")
    end

    add_cxflags("-static-libstdc++")
elseif is_plat("freebds") then
    --none
elseif is_plat("cross") then
    add_cxflags("-fno-rtti")
    if is_mode("release") then
        add_cxflags("-flto")
    end

    add_cxflags("-static-libstdc++")
end


option("native")
    set_default(false)
    set_showmenu(true)
    if is_plat("windows") then
        add_vectorexts("all")
    else
        add_cxflags("-march=native")
    end
option_end()

option("maths-kernel")
    set_default("default")
    set_showmenu(true)
    set_values("default")
option_end()

option("memory-allocator")
    set_default("default")
    set_showmenu(true)
    set_values("default")
option_end()

option("custom-io-observer")
    set_default(false)
    set_showmenu(true)

    add_defines("PHY_ENGINE_USE_CUSTOM_IO_OBSERVER")
option_end()


target("phy_engine")
    set_options("native")
    set_options("maths-kernel")
    set_options("memory-allocator")
    set_options("custom-io-observer")

    add_files("src/**.cpp")
    --add_files("customize/**.cpp")
target_end()

--
-- If you want to known more usage about xmake, please see https://xmake.io
--
-- ## FAQ
--
-- You can enter the project directory firstly before building project.
--
--   $ cd projectdir
--
-- 1. How to build project?
--
--   $ xmake
--
-- 2. How to configure project?
--
--   $ xmake f -p [macosx|linux|iphoneos ..] -a [x86_64|i386|arm64 ..] -m [debug|release]
--
-- 3. Where is the build output directory?
--
--   The default output directory is `./build` and you can configure the output directory.
--
--   $ xmake f -o outputdir
--   $ xmake
--
-- 4. How to run and debug target after building project?
--
--   $ xmake run [targetname]
--   $ xmake run -d [targetname]
--
-- 5. How to install target to the system directory or other output directory?
--
--   $ xmake install
--   $ xmake install -o installdir
--
-- 6. Add some frequently-used compilation flags in xmake.lua
--
-- @code
--    -- add debug and release modes
--    add_rules("mode.debug", "mode.release")
--
--    -- add macro definition
--    add_defines("NDEBUG", "_GNU_SOURCE=1")
--
--    -- set warning all as error
--    set_warnings("all", "error")
--
--    -- set language: c99, c++11
--    set_languages("c99", "c++11")
--
--    -- set optimization: none, faster, fastest, smallest
--    set_optimize("fastest")
--
--    -- add include search directories
--    add_includedirs("/usr/include", "/usr/local/include")
--
--    -- add link libraries and search directories
--    add_links("tbox")
--    add_linkdirs("/usr/local/lib", "/usr/lib")
--
--    -- add system link libraries
--    add_syslinks("z", "pthread")
--
--    -- add compilation and link flags
--    add_cxflags("-stdnolib", "-fno-strict-aliasing")
--    add_ldflags("-L/usr/local/lib", "-lpthread", {force = true})
--
-- @endcode
--

