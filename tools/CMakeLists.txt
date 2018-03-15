


INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/common)
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/tests)

ADD_EXECUTABLE(ray_fire ray_fire.cpp ${SRC_FILES})
ADD_EXECUTABLE(rand_ray_gen rand_ray_gen.cpp ${SRC_FILES})
ADD_EXECUTABLE(leaf_writer leafwriter.cpp ${SRC_FILES})
ADD_EXECUTABLE(traversal_writer travwriter.cpp ${SRC_FILES})
ADD_EXECUTABLE(bvh_validator validator.cpp ${SRC_FILES})
ADD_EXECUTABLE(performance_report performance_report.cpp ${SRC_FILES})

TARGET_LINK_LIBRARIES(ray_fire  ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(rand_ray_gen  ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(leaf_writer ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(traversal_writer ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(bvh_validator ${MOAB_LIBRARIES} MBVH)
TARGET_LINK_LIBRARIES(performance_report ${MOAB_LIBRARIES} MBVH)

INSTALL( TARGETS ray_fire
                 rand_ray_gen
		 leaf_writer
		 traversal_writer
		 bvh_validator
		 performance_report
         DESTINATION ${TOOLS_INSTALL_DIR})

INSTALL(FILES WriteVisitor.hpp
        DESTINATION ${CMAKE_INSTALL_PREFIX}/includes)
INSTALL(FILES BaseVisitor.hpp
	DESTINATION ${CMAKE_INSTALL_PREFIX}/includes)
