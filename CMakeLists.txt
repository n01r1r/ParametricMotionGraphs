cmake_minimum_required(VERSION 3.20)
project(ParametricMotionGraphs LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(PMG_BUILD_TESTS "Build ParametricMotionGraphs tests" ON)
option(PMG_BUILD_VIEWER "Build the OpenGL BVH viewer (pulls GLFW/GLEW/GLM/ImGui)" OFF)

add_library(pmg_core
    src/AlignmentStrategy.cpp
    src/BvhLoader.cpp
    src/ContactDetection.cpp
    src/CyclicContinuity.cpp
    src/FootLocking.cpp
    src/ForwardKinematics.cpp
    src/GoalDirectedLocomotion.cpp
    src/GraphIo.cpp
    src/GraphSpec.cpp
    src/LegacyFrameCountClipGeneration.cpp
    src/MotionClip.cpp
    src/MotionDistance.cpp
    src/MotionRegistration.cpp
    src/MotionSpacePreparation.cpp
    src/ParametricMotionGraph.cpp
    src/ParametricMotionSpace.cpp
    src/ParameterSupport.cpp
    src/PmgArtifact.cpp
    src/PmgBuilder.cpp
    src/PoseBlend.cpp
    src/RuntimeController.cpp
    src/SkeletonCompatibility.cpp
    src/TimeWarp.cpp
    src/TransitionQuality.cpp
    src/TransitionWindow.cpp
)

target_include_directories(pmg_core PUBLIC include)

target_compile_options(pmg_core PRIVATE
    $<$<CXX_COMPILER_ID:GNU,Clang>:-Wall -Wextra -Wpedantic>
    $<$<CXX_COMPILER_ID:MSVC>:/W4>
)

add_executable(pmg_cli
    apps/pmg_cli.cpp
    apps/PmgBvhCommands.cpp
    apps/PmgCommands.cpp
    apps/PmgDiagnosticCommands.cpp
    apps/PmgGraphCommands.cpp
    apps/PmgRuntimeCommands.cpp
    apps/PmgTransitionCommands.cpp
)
target_link_libraries(pmg_cli PRIVATE pmg_core)

if(PMG_BUILD_TESTS)
    enable_testing()

    add_executable(test_pose_blend tests/test_pose_blend.cpp)
    target_link_libraries(test_pose_blend PRIVATE pmg_core)
    add_test(NAME test_pose_blend COMMAND test_pose_blend)

    add_executable(test_rigid_transform tests/test_rigid_transform.cpp)
    target_link_libraries(test_rigid_transform PRIVATE pmg_core)
    add_test(NAME test_rigid_transform COMMAND test_rigid_transform)

    add_executable(test_motion_clip tests/test_motion_clip.cpp)
    target_link_libraries(test_motion_clip PRIVATE pmg_core)
    add_test(NAME test_motion_clip COMMAND test_motion_clip)

    add_executable(test_parametric_motion_space tests/test_parametric_motion_space.cpp)
    target_link_libraries(test_parametric_motion_space PRIVATE pmg_core)
    add_test(NAME test_parametric_motion_space COMMAND test_parametric_motion_space)

    add_executable(test_parameter_support tests/test_parameter_support.cpp)
    target_link_libraries(test_parameter_support PRIVATE pmg_core)
    add_test(NAME test_parameter_support COMMAND test_parameter_support)

    add_executable(test_bvh_loader tests/test_bvh_loader.cpp)
    target_link_libraries(test_bvh_loader PRIVATE pmg_core)
    add_test(NAME test_bvh_loader COMMAND test_bvh_loader)

    add_executable(test_motion_distance tests/test_motion_distance.cpp)
    target_link_libraries(test_motion_distance PRIVATE pmg_core)
    add_test(NAME test_motion_distance COMMAND test_motion_distance)

    add_executable(test_distance_grid tests/test_distance_grid.cpp)
    target_link_libraries(test_distance_grid PRIVATE pmg_core)
    add_test(NAME test_distance_grid COMMAND test_distance_grid)

    add_executable(test_transition_window_contract
        tests/test_transition_window_contract.cpp)
    target_link_libraries(test_transition_window_contract PRIVATE pmg_core)
    add_test(NAME test_transition_window_contract
        COMMAND test_transition_window_contract)

    add_executable(test_transition_quality
        tests/test_transition_quality.cpp)
    target_link_libraries(test_transition_quality PRIVATE pmg_core)
    add_test(NAME test_transition_quality COMMAND test_transition_quality)

    add_executable(test_cyclic_continuity
        tests/test_cyclic_continuity.cpp)
    target_link_libraries(test_cyclic_continuity PRIVATE pmg_core)
    add_test(NAME test_cyclic_continuity COMMAND test_cyclic_continuity)

    add_executable(test_pmg_builder tests/test_pmg_builder.cpp)
    target_link_libraries(test_pmg_builder PRIVATE pmg_core)
    add_test(NAME test_pmg_builder COMMAND test_pmg_builder)

    add_executable(test_edge_lookup tests/test_edge_lookup.cpp)
    target_link_libraries(test_edge_lookup PRIVATE pmg_core)
    add_test(NAME test_edge_lookup COMMAND test_edge_lookup)

    add_executable(test_runtime_controller tests/test_runtime_controller.cpp)
    target_link_libraries(test_runtime_controller PRIVATE pmg_core)
    add_test(NAME test_runtime_controller COMMAND test_runtime_controller)

    add_executable(test_alignment_strategy tests/test_alignment_strategy.cpp)
    target_link_libraries(test_alignment_strategy PRIVATE pmg_core)
    add_test(NAME test_alignment_strategy COMMAND test_alignment_strategy)

    add_executable(test_skeleton_compatibility tests/test_skeleton_compatibility.cpp)
    target_link_libraries(test_skeleton_compatibility PRIVATE pmg_core)
    add_test(NAME test_skeleton_compatibility COMMAND test_skeleton_compatibility)

    add_executable(test_graph_io tests/test_graph_io.cpp)
    target_link_libraries(test_graph_io PRIVATE pmg_core)
    add_test(NAME test_graph_io COMMAND test_graph_io)

    add_executable(test_graph_spec tests/test_graph_spec.cpp)
    target_link_libraries(test_graph_spec PRIVATE pmg_core)
    add_test(NAME test_graph_spec COMMAND test_graph_spec)

    add_executable(test_motion_space_preparation tests/test_motion_space_preparation.cpp)
    target_link_libraries(test_motion_space_preparation PRIVATE pmg_core)
    add_test(NAME test_motion_space_preparation COMMAND test_motion_space_preparation)

    add_executable(test_time_warp tests/test_time_warp.cpp)
    target_link_libraries(test_time_warp PRIVATE pmg_core)
    add_test(NAME test_time_warp COMMAND test_time_warp)

    add_executable(test_contact_detection tests/test_contact_detection.cpp)
    target_link_libraries(test_contact_detection PRIVATE pmg_core)
    add_test(NAME test_contact_detection COMMAND test_contact_detection)

    add_executable(test_motion_registration tests/test_motion_registration.cpp)
    target_link_libraries(test_motion_registration PRIVATE pmg_core)
    add_test(NAME test_motion_registration COMMAND test_motion_registration)

    add_executable(test_dtw_refine tests/test_dtw_refine.cpp)
    target_link_libraries(test_dtw_refine PRIVATE pmg_core)
    add_test(NAME test_dtw_refine COMMAND test_dtw_refine)

    add_executable(test_foot_locking tests/test_foot_locking.cpp)
    target_link_libraries(test_foot_locking PRIVATE pmg_core)
    add_test(NAME test_foot_locking COMMAND test_foot_locking)

    add_executable(test_registered_blending tests/test_registered_blending.cpp)
    target_link_libraries(test_registered_blending PRIVATE pmg_core)
    add_test(NAME test_registered_blending COMMAND test_registered_blending)

    add_executable(test_goal_directed_locomotion tests/test_goal_directed_locomotion.cpp)
    target_link_libraries(test_goal_directed_locomotion PRIVATE pmg_core)
    add_test(NAME test_goal_directed_locomotion COMMAND test_goal_directed_locomotion)

    set(PMG_TEST_ARTIFACT_ROOT ${CMAKE_CURRENT_BINARY_DIR}/test_artifacts)

    # Canonical course-demo spec. This is the only viewer-facing PMG demo kept
    # for the final clean repository.
    add_test(NAME cli_validate_demo_walk_2d
        COMMAND pmg_cli --validate-graph-spec
            ${CMAKE_CURRENT_SOURCE_DIR}/specs/demo_walk_2d_triangle.pmg_spec)

    add_test(NAME cli_build_artifact_walk_2d
        COMMAND pmg_cli --build-graph
            ${CMAKE_CURRENT_SOURCE_DIR}/specs/demo_walk_2d_triangle.pmg_spec
            ${PMG_TEST_ARTIFACT_ROOT}/walk_2d/artifact.pmg)

    add_test(NAME cli_goto_walk_2d
        COMMAND pmg_cli --goto
            ${CMAKE_CURRENT_SOURCE_DIR}/specs/demo_walk_2d_triangle.pmg_spec 10 10
            --seconds 20
            --max-pop-ratio 8.0)

    # Unit fixtures. These are not presentation demos; they exercise edge
    # classification and transition-domain shrink behavior.
    add_test(NAME cli_build_artifact_selective
        COMMAND pmg_cli --build-graph
            ${CMAKE_CURRENT_SOURCE_DIR}/specs/fixtures/fixture_edge_selective_good_bad.pmg_spec
            ${PMG_TEST_ARTIFACT_ROOT}/selective/artifact.pmg)

    add_test(NAME cli_build_artifact_box_shrink
        COMMAND pmg_cli --build-graph
            ${CMAKE_CURRENT_SOURCE_DIR}/specs/fixtures/fixture_transition_box_shrink.pmg_spec
            ${PMG_TEST_ARTIFACT_ROOT}/box_shrink/artifact.pmg)
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
        apps/viewer/PmgViewerWorkspace.cpp
        apps/viewer/PmgViewerWorkspaceGraph.cpp
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
        viewer_host
        pmg_viewer_workspace
    )

    if(PMG_BUILD_TESTS)
        add_executable(test_viewer_host tests/test_viewer_host.cpp)
        target_link_libraries(test_viewer_host PRIVATE viewer_host)
        add_test(NAME test_viewer_host COMMAND test_viewer_host)

        add_executable(test_viewer_graph_authoring
            tests/test_viewer_graph_authoring.cpp)
        target_include_directories(test_viewer_graph_authoring
            PRIVATE apps/viewer)
        add_test(NAME test_viewer_graph_authoring
            COMMAND test_viewer_graph_authoring)

        add_executable(test_viewer_graph_runtime tests/test_viewer_graph_runtime.cpp)
        target_include_directories(test_viewer_graph_runtime PRIVATE apps/viewer include)
        target_link_libraries(test_viewer_graph_runtime PRIVATE pmg_viewer_workspace pmg_core)
        add_test(NAME test_viewer_graph_runtime COMMAND test_viewer_graph_runtime)
    endif()
endif()
