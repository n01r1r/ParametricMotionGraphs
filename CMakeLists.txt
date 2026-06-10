cmake_minimum_required(VERSION 3.20)
project(ParametricMotionGraphs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PMG_BUILD_TESTS "Build ParametricMotionGraphs tests" ON)
option(PMG_BUILD_VIEWER "Build the OpenGL BVH viewer (pulls GLFW/GLEW/GLM/ImGui)" ON)

add_library(pmg_core
    src/BvhLoader.cpp
    src/ForwardKinematics.cpp
    src/MotionClip.cpp
    src/MotionDistance.cpp
    src/ParametricMotionGraph.cpp
    src/ParametricMotionSpace.cpp
    src/PmgBuilder.cpp
    src/PoseBlend.cpp
    src/RuntimeController.cpp
)

target_include_directories(pmg_core PUBLIC include)

target_compile_options(pmg_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(pmg_cli apps/pmg_cli.cpp)
target_link_libraries(pmg_cli PRIVATE pmg_core)

if(PMG_BUILD_TESTS)
    enable_testing()

    add_executable(test_pose_blend tests/test_pose_blend.cpp)
    target_link_libraries(test_pose_blend PRIVATE pmg_core)
    add_test(NAME test_pose_blend COMMAND test_pose_blend)

    add_executable(test_motion_clip tests/test_motion_clip.cpp)
    target_link_libraries(test_motion_clip PRIVATE pmg_core)
    add_test(NAME test_motion_clip COMMAND test_motion_clip)

    add_executable(test_parametric_motion_space tests/test_parametric_motion_space.cpp)
    target_link_libraries(test_parametric_motion_space PRIVATE pmg_core)
    add_test(NAME test_parametric_motion_space COMMAND test_parametric_motion_space)

    add_executable(test_bvh_loader tests/test_bvh_loader.cpp)
    target_link_libraries(test_bvh_loader PRIVATE pmg_core)
    add_test(NAME test_bvh_loader COMMAND test_bvh_loader)

    add_executable(test_motion_distance tests/test_motion_distance.cpp)
    target_link_libraries(test_motion_distance PRIVATE pmg_core)
    add_test(NAME test_motion_distance COMMAND test_motion_distance)

    add_executable(test_distance_grid tests/test_distance_grid.cpp)
    target_link_libraries(test_distance_grid PRIVATE pmg_core)
    add_test(NAME test_distance_grid COMMAND test_distance_grid)

    add_executable(test_pmg_builder tests/test_pmg_builder.cpp)
    target_link_libraries(test_pmg_builder PRIVATE pmg_core)
    add_test(NAME test_pmg_builder COMMAND test_pmg_builder)

    add_executable(test_edge_lookup tests/test_edge_lookup.cpp)
    target_link_libraries(test_edge_lookup PRIVATE pmg_core)
    add_test(NAME test_edge_lookup COMMAND test_edge_lookup)

    add_executable(test_runtime_controller tests/test_runtime_controller.cpp)
    target_link_libraries(test_runtime_controller PRIVATE pmg_core)
    add_test(NAME test_runtime_controller COMMAND test_runtime_controller)
endif()

# ---------------------------------------------------------------------------
# Viewer: real-time OpenGL skeleton renderer + ImGui control UI.
# Third-party deps are confined to this target; pmg_core stays dependency-free.
# Pinned tags keep configure reproducible.
# ---------------------------------------------------------------------------
if(PMG_BUILD_VIEWER)
    include(FetchContent)

    find_package(OpenGL REQUIRED)

    # Some pinned third-party deps still declare a pre-3.5 minimum; CMake 4.x
    # rejects that. Allow them to configure under the modern policy set.
    set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

    set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(GLFW_INSTALL OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG 3.4
        GIT_SHALLOW TRUE
    )

    set(ONLY_LIBS ON CACHE BOOL "" FORCE)          # glew-cmake: build libs only
    set(glew-cmake_BUILD_SHARED OFF CACHE BOOL "" FORCE)
    set(glew-cmake_BUILD_STATIC ON CACHE BOOL "" FORCE)
    FetchContent_Declare(
        glew
        GIT_REPOSITORY https://github.com/Perlmint/glew-cmake.git
        GIT_TAG glew-cmake-2.3.1
        GIT_SHALLOW TRUE
    )

    FetchContent_Declare(
        glm
        GIT_REPOSITORY https://github.com/g-truc/glm.git
        GIT_TAG 1.0.1
        GIT_SHALLOW TRUE
    )

    FetchContent_Declare(
        imgui
        GIT_REPOSITORY https://github.com/ocornut/imgui.git
        GIT_TAG v1.92.8-docking
        GIT_SHALLOW TRUE
    )

    FetchContent_MakeAvailable(glfw glew glm imgui)

    # Dear ImGui ships no CMake build; compile core + GLFW/OpenGL3 backends here.
    add_library(imgui_backend STATIC
        ${imgui_SOURCE_DIR}/imgui.cpp
        ${imgui_SOURCE_DIR}/imgui_draw.cpp
        ${imgui_SOURCE_DIR}/imgui_tables.cpp
        ${imgui_SOURCE_DIR}/imgui_widgets.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
        ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    )
    target_include_directories(imgui_backend PUBLIC
        ${imgui_SOURCE_DIR}
        ${imgui_SOURCE_DIR}/backends
    )
    target_link_libraries(imgui_backend PUBLIC glfw OpenGL::GL)

    add_executable(pmg_viewer
        apps/viewer/main.cpp
        apps/viewer/Camera.cpp
        apps/viewer/MeshPrimitives.cpp
        apps/viewer/SkeletonRenderer.cpp
        apps/viewer/ViewerApp.cpp
    )
    target_link_libraries(pmg_viewer PRIVATE
        pmg_core
        glfw
        libglew_static
        glm::glm
        imgui_backend
        OpenGL::GL
    )
    target_compile_definitions(pmg_viewer PRIVATE GLEW_STATIC)

    # Resolve the BVH corpus next to the source tree at runtime.
    target_compile_definitions(pmg_viewer PRIVATE
        PMG_BVH_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/BVH")
endif()
