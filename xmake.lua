set_xmakever("3.0.0")
set_project("FluidMotion")
set_version("0.1.1")
set_arch("x64")
set_languages("c++23")
set_warnings("allextra")
add_rules("mode.debug", "mode.releasedbg")
set_defaultmode("releasedbg")

set_config("skyrim_se", true)
set_config("skyrim_ae", true)
set_config("skyrim_vr", false)
set_config("rex_ini", true)

includes("lib/commonlibsse-ng")

add_requires("xbyak")

target("FluidMotion", function()
    add_deps("commonlibsse-ng")
    add_packages("xbyak")
    add_syslinks("d3d11", "dxgi")
    add_rules("commonlibsse-ng.plugin", {
        name = "FluidMotion",
        author = "megakuutti",
        description = "Fluid Motion: Pathfinding Overhaul"
    })

    add_files("src/**.cpp")
    add_headerfiles("src/**.h")
    add_includedirs("src")
    set_pcxxheader("src/PCH.h")

    add_installfiles("dist/(SKSE/Plugins/*.ini)")
    add_installfiles("dist/(SKSE/Plugins/*.bin)")
end)
