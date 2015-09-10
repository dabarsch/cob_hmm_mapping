add_library(scanmatcher scanmatcher.cpp smmap.cpp)
target_link_libraries(scanmatcher sensor_range utils)

install(TARGETS scanmatcher DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
