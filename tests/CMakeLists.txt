

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)

ADD_EXECUTABLE(test_vec3f test_vec.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_aabb test_aabb.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_ray test_ray.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_intersect test_intersect.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_vfloat test_vfloat.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_vbool test_vbool.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_traverse test_traverse.cpp ${SRC_FILES})
ADD_EXECUTABLE(test_builder test_builder.cpp ${SRC_FILES})


