function(run_cli name)
    execute_process(
        COMMAND ${ARGN}
        RESULT_VARIABLE exit_code
        OUTPUT_VARIABLE stdout_text
        ERROR_VARIABLE stderr_text)
    if (NOT exit_code EQUAL 0)
        message(FATAL_ERROR
            "${name} failed\nexit=${exit_code}\nstdout:\n${stdout_text}\nstderr:\n${stderr_text}")
    endif()
    set(${name}_STDOUT "${stdout_text}" PARENT_SCOPE)
endfunction()

file(REMOVE_RECURSE "${OUT_ROOT}")
file(MAKE_DIRECTORY "${OUT_ROOT}")

run_cli(
    INSPECT
    "${PMG_CLI}" inspect-skeleton "${BVH_PATH}"
    --out "${OUT_ROOT}/inspect")
if (INSPECT_STDOUT MATCHES "skeleton_report=")
    if (NOT EXISTS "${OUT_ROOT}/inspect/skeleton_report.md")
        message(FATAL_ERROR "inspect-skeleton did not write skeleton_report.md")
    endif()
else()
    message(FATAL_ERROR "inspect-skeleton missing skeleton_report= stdout marker")
endif()

run_cli(
    LOOP
    "${PMG_CLI}" audit-loop "${BVH_PATH}"
    --out "${OUT_ROOT}/loop")
if (LOOP_STDOUT MATCHES "loop_audit=")
    if (NOT EXISTS "${OUT_ROOT}/loop/loop_audit.md")
        message(FATAL_ERROR "audit-loop did not write loop_audit.md")
    endif()
else()
    message(FATAL_ERROR "audit-loop missing loop_audit= stdout marker")
endif()

run_cli(
    TRANSITION_POP
    "${PMG_CLI}" audit-transition-pop "${PMG_PATH}"
    --out "${OUT_ROOT}/transition_pop"
    --worst-k "3")
if (TRANSITION_POP_STDOUT MATCHES "transition_pop_audit=")
    if (NOT EXISTS "${OUT_ROOT}/transition_pop/transition_pop_audit.md")
        message(FATAL_ERROR "audit-transition-pop did not write transition_pop_audit.md")
    endif()
else()
    message(FATAL_ERROR "audit-transition-pop missing transition_pop_audit= stdout marker")
endif()
