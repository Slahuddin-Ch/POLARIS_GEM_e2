cmake_minimum_required(VERSION 3.0.2)
project(signal_simulation)

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

# Main executable for signal simulator
add_executable(signal_simulator_node src/signal_simulator_node.cpp src/signal_simulator.cpp)
target_link_libraries(signal_simulator_node
  ${catkin_LIBRARIES}
)

# Test executable for signal simulator
add_executable(test_signal_simulator test/test_signal_simulator.cpp src/signal_simulator.cpp)
target_compile_definitions(test_signal_simulator PRIVATE SIGNAL_SIMULATOR_TEST)
target_link_libraries(test_signal_simulator
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}  # Link Google Test libraries
  pthread             # Link pthread for multithreading support
)

# Rostest integration
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_signal_simulator_rostest test/test_signal_simulator.launch test/test_signal_simulator.cpp)
  target_compile_definitions(test_signal_simulator_rostest PRIVATE SIGNAL_SIMULATOR_TEST)
  target_link_libraries(test_signal_simulator_rostest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  target_sources(test_signal_simulator_rostest PRIVATE src/signal_simulator.cpp)
endif()
