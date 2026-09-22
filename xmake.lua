set_languages("c++20")
add_rules("mode.debug", "mode.release")

if is_plat("windows", "mingw") then
    print("On windows we should settle for msvc")
    set_toolchains("msvc")
    add_defines("WIN32_LEAN_AND_MEAN", "NOMINMAX")
    add_cxflags("/utf-8", "/Zc:__cplusplus")
end

add_requires("glfw")
add_requires("imgui", {configs = {glfw = true, opengl3 = true}})
add_requires("imnodes")
add_requires("lemon")
add_requires("tracy")
add_requires("hwinfo")

target("graph-tutorial")
    set_kind("binary")

    add_files("src/graphs/**.cpp")
    add_includedirs("src/graphs")

    add_packages("lemon", "hwinfo", "glfw", "imgui", "imnodes", "tracy")