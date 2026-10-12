set(PMG_CORE_TESTS
    test_scene_coordinates
    test_cmu_runtime_contacts
    test_input_validation
    test_streaming_foot_lock
    test_alignment_strategy
    test_bvh_loader
    test_candidate_window_extractor
    test_contact_detection
    test_corpus_audit
    test_cyclic_continuity
    test_diagnostics
    test_distance_grid
    test_dtw_refine
    test_edge_lookup
    test_foot_locking
    test_goal_directed_locomotion
    test_graph_io
    test_graph_spec
    test_motion_clip
    test_motion_distance
    test_motion_registration
    test_motion_space_preparation
    test_parameter_support
    test_parametric_motion_space
    test_pmg_builder
    test_pmg_offline_pipeline
    test_pose_blend
    test_registered_blending
    test_rigid_transform
    test_root_canonicalization
    test_runtime_controller
    test_skeleton_compatibility
    test_time_warp
    test_transition_quality
    test_transition_window_contract
)

foreach(test_name IN LISTS PMG_CORE_TESTS)
    add_executable(${test_name} ${test_name}.cpp)
    target_link_libraries(${test_name} PRIVATE pmg_core)
    target_compile_definitions(${test_name} PRIVATE PMG_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
    # These legacy tests use assert; keep checks enabled in Release builds too.
    target_compile_options(${test_name} PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-UNDEBUG>
        $<$<CXX_COMPILER_ID:MSVC>:/UNDEBUG>)
    add_test(NAME ${test_name} COMMAND ${test_name})
    set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
endforeach()

if(TARGET pmg_viewer_runtime)
    add_executable(test_viewer_runtime_module test_viewer_runtime_module.cpp)
    target_link_libraries(test_viewer_runtime_module PRIVATE pmg_viewer_runtime)
    target_compile_options(test_viewer_runtime_module PRIVATE
        $<$<CXX_COMPILER_ID:GNU,Clang,AppleClang>:-UNDEBUG>
        $<$<CXX_COMPILER_ID:MSVC>:/UNDEBUG>)
    add_test(NAME test_viewer_runtime_module COMMAND test_viewer_runtime_module)
endif()
