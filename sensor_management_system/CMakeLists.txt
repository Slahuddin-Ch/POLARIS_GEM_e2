cmake_minimum_required(VERSION 3.0.2)
project(sensor_management_system)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  rostest
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# Main executable for sensor manager
add_executable(sensor_manager_node src/sensor_manager_node.cpp src/sensor_manager.cpp)
target_link_libraries(sensor_manager_node
  ${catkin_LIBRARIES}
)

# Test executable for sensor manager
add_executable(test_sensor_manager test/test_sensor_manager.cpp src/sensor_manager.cpp)
target_compile_definitions(test_sensor_manager PRIVATE SENSOR_MANAGER_TEST)
target_link_libraries(test_sensor_manager
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
  pthread
)

# Rostest integration
if (CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_sensor_manager_rostest test/test_sensor_manager.launch test/test_sensor_manager.cpp)
  target_compile_definitions(test_sensor_manager_rostest PRIVATE SENSOR_MANAGER_TEST)
  target_link_libraries(test_sensor_manager_rostest ${catkin_LIBRARIES} ${GTEST_LIBRARIES} pthread)
  target_sources(test_sensor_manager_rostest PRIVATE src/sensor_manager.cpp)
endif()
