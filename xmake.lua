--[[
/**
 * Author: R-CO
 * E-Mail: daniel1820kobe@gmail.com
 **/
 ]]
 
set_languages("c11", "cxx17")
if is_mode("debug") then
    set_symbols("debug")
    set_optimize("none")
else
    set_optimize("faster")
end

add_cxxflags("-Wall", "-Wextra")

add_requires("gtest")
if is_plat("linux") then
    add_syslinks("pthread")
end

target("test_dv")
    set_kind("binary")

    add_packages("gtest")

    add_includedirs("$(projectdir)/src/")

    add_files("src/*.cpp")
    add_files("gtest/*.cpp")
target_end()
