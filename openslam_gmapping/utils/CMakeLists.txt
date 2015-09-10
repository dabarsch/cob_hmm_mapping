include_directories(./)
add_library(utils stat.cpp)
install(TARGETS utils DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
