cmake_minimum_required(VERSION 3.0.2)
project(gps_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rostest
)

# Find Google Test
find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main executable for GPS simulator
add_executable(gps_simulator_node src/gps_simulator_node.cpp src/gps_simulator.cpp)
target_link_libraries(gps_simulator_node
  ${catkin_LIBRARIES}
)

# Test executable for GPS simulator
add_executable(test_gps_simulator test/test_gps_simulator.cpp src/gps_simulator.cpp)
target_compile_definitions(test_gps_simulator PRIVATE GPS_SIMULATOR_TEST)
target_link_libraries(test_gps_simulator
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}  # Link Google Test libraries
  pthread             # Link pthread for multithreading support
)

# Rostest integration
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_gps_simulator_rostest test/test_gps_simulator.launch test/test_gps_simulator.cpp)
  target_compile_definitions(test_gps_simulator_rostest PRIVATE GPS_SIMULATOR_TEST)
  target_link_libraries(test_gps_simulator_rostest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  target_sources(test_gps_simulator_rostest PRIVATE src/gps_simulator.cpp)
endif()
