cmake_minimum_required(VERSION 2.8)
project(gmapping)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
  nav_msgs
  openslam_gmapping
  roscpp
  rostest
  tf
  rosbag_storage
  server_slam
)

find_package(Boost REQUIRED
  signals
)

catkin_package()

include_directories(
  src
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(slam_gmapping
  src/slam_gmapping.cpp
  src/main.cpp
)

target_link_libraries(slam_gmapping
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

if(catkin_EXPORTED_TARGETS)
  add_dependencies(slam_gmapping
    ${catkin_EXPORTED_TARGETS}
  )
endif()

install(TARGETS slam_gmapping
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
