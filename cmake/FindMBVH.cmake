# Try to find MOAB
#
# Once done this will define
#
#  MBVH_INCLUDE - the MOAB include directory

find_path(MBVH_CMAKE_CONFIG NAMES MBVHConfig.cmake
          HINTS ${MBVH_ROOT}
          PATHS ENV LD_LIBRARY_PATH
          PATH_SUFFIXES lib Lib cmake cmake/MBVH
          NO_DEFAULT_PATH)

message(STATUS "Found MBVH in ${MBVH_CMAKE_CONFIG}")

include(${MBVH_CMAKE_CONFIG}/MBVHConfig.cmake)
