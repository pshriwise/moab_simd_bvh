PROJECT(MOAB_SIMD_BVH)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

ADD_SUBDIRECTORY(tests)

ADD_TEST(NAME vec3f_test COMMAND test_vec3f)
ADD_TEST(NAME aabb_test COMMAND test_aabb)
ADD_TEST(NAME ray_test COMMAND test_ray)
ADD_TEST(NAME intersect_test COMMAND test_intersect)
ADD_TEST(NAME vfloat_test COMMAND test_vfloat)
ADD_TEST(NAME vbool_test COMMAND test_vbool)
ADD_TEST(NAME traverse_test COMMAND test_traverse)

ENABLE_TESTING()
