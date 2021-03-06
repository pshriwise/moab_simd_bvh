

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/common)

CONFIGURE_FILE(test_files.h.in ${CMAKE_CURRENT_SOURCE_DIR}/test_files.h)

FOREACH(TEST_NAME IN LISTS TEST_FILES)
  ADD_EXECUTABLE(test_${TEST_NAME} test_${TEST_NAME}.cpp ${SRC_FILES})
  ADD_TEST(test_${TEST_NAME} test_${TEST_NAME} )
ENDFOREACH()

# Tests that rely on MOAB
TARGET_LINK_LIBRARIES(test_moab_mesh ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(test_moab_old ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(test_MBVH ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(test_manager ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(test_filter_funcs ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(test_distant_rays ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(test_closest_to_location ${MOAB_LIBRARIES} MBVH)

IF(TEST_COVERAGE)
  
  IF(CMAKE_COMPILER_IS_GNUCXX)
    include(CodeCoverage)
    FOREACH(TEST_NAME IN LISTS TEST_FILES)
      ADD_TEST(NAME ${TEST_NAME}_test COMMAND test_${TEST_NAME})
      setup_target_for_coverage( NAME test_${TEST_NAME}_coverage EXECUTABLE test_${TEST_NAME} )
      LIST(APPEND TEST_NAMES test_${TEST_NAME}_coverage)
      LIST(APPEND LCOV_MERGE_NAMES -a test_${TEST_NAME}_coverage.info)
    ENDFOREACH()
  ENDIF()
  
  ADD_CUSTOM_TARGET(test_coverage DEPENDS ${TEST_NAMES})
  ADD_CUSTOM_COMMAND(
    TARGET test_coverage
    COMMAND ${LCOV_PATH} ${LCOV_MERGE_NAMES} --output test_coverage.info
    COMMAND ${GENHTML_PATH} test_coverage.info -o test_coverage
    VERBATIM )
  
ENDIF()

ENABLE_TESTING()

ADD_SUBDIRECTORY(test_files)
