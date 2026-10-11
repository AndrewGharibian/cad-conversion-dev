# =============================================================================
# Tests: for every sample model in tests/data,
#   convert.<model>      converts it to a per-body OBJ; fails on any WARNING
#                        or ERROR in the converter's report;
#   check.<model>        checks that OBJ independently (mesh_check: every
#                        solid closed and consistently oriented, no two
#                        solids overlapping);
#   convert_stl.<model>  converts it with the default output (STL, one object
#                        per part), into an --output-dir;
#   stl_written.<model>  checks that the STL is in that folder.
# Models over 5 MB are labelled "large" and skipped by the test presets; run
# them with: ctest --test-dir build/<preset> [-C Release] -L large
# Large models aren't committed: put them in tests/data/large/ (gitignored).
# =============================================================================
add_executable(mesh_check mesh_check.cpp)
target_link_libraries(mesh_check PRIVATE manifold::manifold)
if(WIN32)
    set(_check_dlls "$<TARGET_RUNTIME_DLLS:mesh_check>")
    add_custom_command(TARGET mesh_check POST_BUILD
        COMMAND "$<$<BOOL:${_check_dlls}>:${CMAKE_COMMAND};-E;copy_if_different;${_check_dlls};$<TARGET_FILE_DIR:mesh_check>>"
        COMMAND_EXPAND_LISTS
        VERBATIM
    )
endif()

file(GLOB _models CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/data/*.step"
    "${CMAKE_CURRENT_SOURCE_DIR}/data/large/*.step"
    "${CMAKE_CURRENT_SOURCE_DIR}/data/large/*.STEP"
    "${CMAKE_CURRENT_SOURCE_DIR}/data/large/*.stp"
)

foreach(_model IN LISTS _models)
    get_filename_component(_name "${_model}" NAME_WE)
    # Each test converts its own copy, so the outputs land in the build tree.
    set(_obj_dir "${CMAKE_CURRENT_BINARY_DIR}/obj")
    set(_stl_dir "${CMAKE_CURRENT_BINARY_DIR}/stl")
    configure_file("${_model}" "${_obj_dir}/${_name}.step" COPYONLY)
    configure_file("${_model}" "${_stl_dir}/${_name}.step" COPYONLY)

    add_test(NAME convert.${_name}
        COMMAND cad_converter --obj --per-body "${_obj_dir}/${_name}.step")
    add_test(NAME check.${_name}
        COMMAND mesh_check "${_obj_dir}/${_name}.obj")
    add_test(NAME convert_stl.${_name}
        COMMAND cad_converter --output-dir "${_stl_dir}/out" "${_stl_dir}/${_name}.step")
    add_test(NAME stl_written.${_name}
        COMMAND ${CMAKE_COMMAND} -E md5sum "${_stl_dir}/out/${_name}.stl")
    set_tests_properties(convert_stl.${_name} PROPERTIES FIXTURES_SETUP stl.${_name})
    set_tests_properties(stl_written.${_name} PROPERTIES FIXTURES_REQUIRED stl.${_name})

    set_tests_properties(convert.${_name} PROPERTIES FIXTURES_SETUP output.${_name})
    set_tests_properties(check.${_name} PROPERTIES FIXTURES_REQUIRED output.${_name})
    # The converter reports problems as WARNING/ERROR lines.
    set_tests_properties(convert.${_name} convert_stl.${_name} PROPERTIES FAIL_REGULAR_EXPRESSION "ERROR;WARNING")

    file(SIZE "${_model}" _size)
    if(_size GREATER 5000000)
        set_tests_properties(convert.${_name} check.${_name} convert_stl.${_name} stl_written.${_name}
            PROPERTIES LABELS large TIMEOUT 7200)
    endif()
endforeach()
