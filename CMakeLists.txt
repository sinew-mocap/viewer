# SPDX-License-Identifier: MIT
# viewer/ — the polyscope PoseSink hexagon: binds OSC-in (39539), runs the body
# solve (solve_core), and renders.
#   core/      channel_map — the neutral /sinew channel -> SOMA joint mapping
#   ports/     pose_sink.h — the port it renders through (how it changes the world)
#   adapters/  anny_demo (polyscope renderer + OSC-in), sinew_view (FFI), smoke
# The full oscin/solve decomposition onto the port is the load-bearing-ports step.
cmake_minimum_required(VERSION 3.19)
project(sinew_viewer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Standalone-build entry (cloned side-by-side in ~/); a no-op under the superbuild.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
    option(SINEW_TESTS "Build the cluster's unit/property tests" OFF)
    if(SINEW_TESTS)
        enable_testing()
    endif()
endif()

# Sibling: the body solve (pulls mount_drift's vkc transitively) — see
# ports/sibling-repos.txt.
if(NOT TARGET solve_core)
    add_subdirectory(../solve ${CMAKE_BINARY_DIR}/_solve)
endif()

# core/ — the neutral domain (no polyscope, no socket).
add_library(viewer_core STATIC core/channel_map.c)
target_include_directories(viewer_core PUBLIC core ports)

# polyscope is vendored under thirdparty/ (this cluster — the viewer's UI).
add_subdirectory("${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/polyscope" polyscope_build)

# Self-contained shared library: polyscope + the MinGW C++ runtime are bundled
# inside the DLL, which exports only the extern "C" sinew_view_* symbols.  Nothing
# C++ crosses the FFI boundary, so a runtime stub loader — or any other toolchain
# (e.g. the Lean viewer_demo) — can drive it without a libstdc++ ABI clash.
add_library(sinew_view SHARED adapters/sinew_view.cpp)
target_include_directories(sinew_view PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/adapters")
target_link_libraries(sinew_view PUBLIC polyscope)
set_target_properties(sinew_view PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON PREFIX "")
if(MINGW)
    target_link_options(sinew_view PRIVATE -static)  # bundle libgcc/libstdc++/winpthread
endif()

# Link smoke test (headless-safe unless given an argument); needs sinew_view.dll.
add_executable(sinew_view_smoke adapters/smoke.c)
target_link_libraries(sinew_view_smoke PRIVATE sinew_view)

# Real ANNY body: skin the embedded SoA rig (core/solve/soma_rig.h).  Self-skips
# until the baked headers exist.
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../solve/core/soma_rig.h" AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../solve/core/soma_pheno.h")
    # anny_demo binds OSC 39539 and skins the rig.  The phenotype (Lean->Slang,
    # CPU) and per-frame LBS (Vulkan, loading lbs.spv) live in solve_core; the
    # viewer links it and adds the polyscope UI.  (TIC via core_calib is wired in
    # the ports step.)
    add_executable(anny_demo adapters/anny_demo.cpp)
    target_include_directories(anny_demo PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../solve/core"
        "${CMAKE_CURRENT_SOURCE_DIR}/../solve/ports"
        "${CMAKE_CURRENT_SOURCE_DIR}/core"
        "${CMAKE_CURRENT_SOURCE_DIR}/adapters"
        "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/polyscope/deps/imgui/imgui")
    target_link_libraries(anny_demo PRIVATE viewer_core solve_core polyscope)
    if(WIN32)
        target_link_libraries(anny_demo PRIVATE ws2_32)  # winsock for the UDP/OSC receiver
    endif()
    if(MINGW)
        target_link_options(anny_demo PRIVATE -static)  # bundle libgcc/stdc++/winpthread
    endif()
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../solve/core/gen/lbs.spv")
        add_custom_command(TARGET anny_demo POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/../solve/core/gen/lbs.spv" "$<TARGET_FILE_DIR:anny_demo>/lbs.spv"
            COMMENT "Copy lbs.spv next to anny_demo")
        install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/../solve/core/gen/lbs.spv" DESTINATION . COMPONENT server)
    endif()
    # soma_pheno.bin (110 MB) is a RUNTIME asset kept out of the repo (fetched from a
    # release) under assets/.  The build/install only bundles it when present.
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/assets/soma_pheno.bin")
        add_custom_command(TARGET anny_demo POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                "${CMAKE_CURRENT_SOURCE_DIR}/assets/soma_pheno.bin" "$<TARGET_FILE_DIR:anny_demo>/soma_pheno.bin"
            COMMENT "Copy soma_pheno.bin next to anny_demo")
        install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/assets/soma_pheno.bin" DESTINATION . COMPONENT server)
    endif()

    install(TARGETS anny_demo RUNTIME DESTINATION . COMPONENT viewer)  # CPack: the viewer binary
endif()

# ── Unit tests (opt-in: -DSINEW_TESTS=ON) ────────────────────────────────────
if(SINEW_TESTS)
    add_executable(viewer_test tests/test_channel_map.cpp)
    target_link_libraries(viewer_test PRIVATE viewer_core)
    add_test(NAME viewer_channel_map COMMAND viewer_test)
endif()
