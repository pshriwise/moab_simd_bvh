

SET(TEST_FILES_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "Test File Location")


EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/sphere.h5m ${CMAKE_CURRENT_BINARY_DIR}/sphere.h5m)
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/small_sphere.h5m ${CMAKE_CURRENT_BINARY_DIR}/small_sphere.h5m)
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/cube.h5m ${CMAKE_CURRENT_BINARY_DIR}/cube.h5m)
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/3k-tri-cube.h5m ${CMAKE_CURRENT_BINARY_DIR}/3k-tri-cube.h5m)
EXECUTE_PROCESS(COMMAND ${CMAKE_COMMAND} -E create_symlink ${CMAKE_CURRENT_SOURCE_DIR}/cube_cyl.h5m ${CMAKE_CURRENT_BINARY_DIR}/cube_cyl.h5m)
