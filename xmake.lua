set_languages("c++20")
add_rules("mode.debug", "mode.release")

if is_plat("windows", "mingw") then
    print("On windows we should settle for msvc")
    set_toolchains("msvc")
    add_defines("WIN32_LEAN_AND_MEAN", "NOMINMAX")
    add_cxflags("/utf-8", "/Zc:__cplusplus")
end

add_requires("conan::tracy/0.13.1")
add_requires("hwinfo")
add_requires("conan::coin-lemon/1.3.1")
add_requires("raylib")

target("graph-tutorial")
    set_kind("binary")

    add_files("src/graphs/**.cpp")
    add_includedirs("src/graphs")

    add_packages("conan::coin-lemon/1.3.1", "raylib", "conan::tracy/0.13.1")