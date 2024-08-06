cmake_minimum_required(VERSION 3.0.2)
project(temperature_simulation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
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

# Main executable for temperature simulator
add_executable(temperature_simulator_node src/temperature_simulator_node.cpp src/temperature_simulator.cpp)
target_link_libraries(temperature_simulator_node
  ${catkin_LIBRARIES}
)

# Test executable for temperature simulator
add_executable(test_temperature_simulator test/test_temperature_simulator.cpp src/temperature_simulator.cpp)
target_compile_definitions(test_temperature_simulator PRIVATE TEMPERATURE_SIMULATOR_TEST)
target_link_libraries(test_temperature_simulator
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}  # Link Google Test libraries
  pthread             # Link pthread for multithreading support
)

# Rostest integration
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_temperature_simulator_rostest test/test_temperature_simulator.launch test/test_temperature_simulator.cpp)
  target_compile_definitions(test_temperature_simulator_rostest PRIVATE TEMPERATURE_SIMULATOR_TEST)
  target_link_libraries(test_temperature_simulator_rostest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  target_sources(test_temperature_simulator_rostest PRIVATE src/temperature_simulator.cpp)
endif()
