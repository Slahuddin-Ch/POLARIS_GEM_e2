cmake_minimum_required(VERSION 3.0.2)
project(battery_simulation)

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

# Main executable for battery simulator
add_executable(battery_simulator_node src/battery_simulator.cpp)
target_link_libraries(battery_simulator_node
  ${catkin_LIBRARIES}
)

# Test executable for battery simulator
add_executable(test_battery_simulator test/test_battery_simulator.cpp src/battery_simulator.cpp)
target_compile_definitions(test_battery_simulator PRIVATE BATTERY_SIMULATOR_TEST)
target_link_libraries(test_battery_simulator
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}  # Link Google Test libraries
  pthread             # Link pthread for multithreading support
)

# Rostest integration
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_battery_simulator_rostest test/test_battery_simulator.launch test/test_battery_simulator.cpp)
  target_compile_definitions(test_battery_simulator_rostest PRIVATE BATTERY_SIMULATOR_TEST)
  target_link_libraries(test_battery_simulator_rostest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  target_sources(test_battery_simulator_rostest PRIVATE src/battery_simulator.cpp)
endif()
