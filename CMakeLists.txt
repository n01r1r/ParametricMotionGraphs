cmake_minimum_required(VERSION 3.20)
project(ParametricMotionGraphs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PMG_BUILD_TESTS "Build ParametricMotionGraphs tests" OFF)
option(PMG_BUILD_VIEWER "Build the OpenGL BVH viewer (pulls GLFW/GLEW/GLM/ImGui)" ON)

add_library(pmg_core
    src/AlignmentStrategy.cpp
    src/BvhLoader.cpp
    src/CandidateWindowExtractor.cpp
    src/ContactDetection.cpp
    src/CorpusAudit.cpp
    src/Diagnostics.cpp
    src/CyclicContinuity.cpp
    src/FootLocking.cpp
    src/ForwardKinematics.cpp
    src/GoalDirectedLocomotion.cpp
    src/GraphIo.cpp
    src/GraphSpec.cpp
    src/MotionClip.cpp
    src/MotionDistance.cpp
    src/MotionRegistration.cpp
    src/MotionSpacePreparation.cpp
    src/ParametricMotionGraph.cpp
    src/ParametricMotionSpace.cpp
    src/ParameterSupport.cpp
    src/PmgArtifact.cpp
    src/PmgBuilder.cpp
    src/PmgOfflinePipeline.cpp
    src/PoseBlend.cpp
    src/RootCanonicalization.cpp
    src/RuntimeController.cpp
    src/SkeletonCompatibility.cpp
    src/TimeWarp.cpp
    src/TransitionDiagnostics.cpp
    src/TransitionQuality.cpp
    src/TransitionWindow.cpp
)

target_include_directories(pmg_core PUBLIC include)

target_compile_options(pmg_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)



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

    set(ONLY_LIBS ON CACHE BOOL "" FORCE)
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

    add_library(viewer_host STATIC
        apps/viewer/Camera.cpp
        apps/viewer/MeshPrimitives.cpp
        apps/viewer/SkeletonRenderer.cpp
        apps/viewer/ViewerHost.cpp
    )
    target_include_directories(viewer_host PUBLIC apps/viewer)
    target_link_libraries(viewer_host PUBLIC
        glfw
        libglew_static
        glm::glm
        imgui_backend
        OpenGL::GL
    )
    target_compile_definitions(viewer_host PUBLIC GLEW_STATIC)

    add_library(pmg_viewer_workspace STATIC
        apps/viewer/ImGuiViewerUi.cpp
        apps/viewer/PmgViewerWorkspace.cpp
        apps/viewer/PmgViewerWorkspaceGraph.cpp
        apps/viewer/ViewerRuntimeModule.cpp
    )
    target_include_directories(pmg_viewer_workspace PUBLIC apps/viewer)
    target_link_libraries(pmg_viewer_workspace
        PUBLIC viewer_host
        PRIVATE pmg_core
    )

    # Resolve the BVH corpus and spec library next to the source tree at
    # runtime; write built artifacts into the always-writable build tree.
    target_compile_definitions(pmg_viewer_workspace PRIVATE
        PMG_BVH_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/BVH"
        PMG_SPEC_DIRECTORY="${CMAKE_CURRENT_SOURCE_DIR}/specs"
        PMG_OUTPUT_DIRECTORY="${CMAKE_BINARY_DIR}/viewer_out")

    add_executable(pmg_viewer apps/viewer/main.cpp)
    target_link_libraries(pmg_viewer PRIVATE
        pmg_core
        viewer_host
        pmg_viewer_workspace
    )

endif()
