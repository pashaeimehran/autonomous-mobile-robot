cmake_minimum_required(VERSION 3.0.2)
project(autonomous_mobile_robot)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)

# pigpio is a Raspberry Pi system library, not a catkin package.
find_path(PIGPIO_INCLUDE_DIR pigpio.h)
find_library(PIGPIO_LIBRARY NAMES pigpio)
if(NOT PIGPIO_INCLUDE_DIR OR NOT PIGPIO_LIBRARY)
  message(FATAL_ERROR "pigpio headers/library not found; install pigpio development files on the Raspberry Pi")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_motor_driver
  CATKIN_DEPENDS roscpp geometry_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${PIGPIO_INCLUDE_DIR})

add_library(${PROJECT_NAME}_motor_driver src/motor_driver.cpp)
target_link_libraries(${PROJECT_NAME}_motor_driver
  ${PIGPIO_LIBRARY} pthread rt
)

add_executable(motor_driver_node src/motor_driver_node.cpp)
target_link_libraries(motor_driver_node
  ${PROJECT_NAME}_motor_driver ${catkin_LIBRARIES}
)
add_dependencies(motor_driver_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_motor_mixing test/test_motor_mixing.cpp)
  # The mixer is header-only: tests do not touch ROS, GPIO, or hardware.
endif()

install(TARGETS ${PROJECT_NAME}_motor_driver motor_driver_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(FILES include/motor_driver.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME}
)
install(DIRECTORY config launch urdf web docs media
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
