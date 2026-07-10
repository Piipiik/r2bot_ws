cmake_minimum_required(VERSION 3.8)
project(r2bot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_path(REALSENSE2_INCLUDE_DIR
  NAMES librealsense2/rs.hpp
  HINTS "$ENV{HOME}/librealsense/include" "/home/yzy/librealsense/include"
)
find_library(REALSENSE2_LIBRARY
  NAMES realsense2 librealsense2
  HINTS
    "$ENV{HOME}/librealsense/build/Release"
    "/home/yzy/librealsense/build/Release"
    "$ENV{HOME}/librealsense/build"
    "/home/yzy/librealsense/build"
)
if(NOT REALSENSE2_INCLUDE_DIR OR NOT REALSENSE2_LIBRARY)
  message(FATAL_ERROR "librealsense2 not found. Build librealsense first or adjust REALSENSE2 paths.")
endif()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(d435_camera_node src/d435_camera_node.cpp)
ament_target_dependencies(d435_camera_node rclcpp sensor_msgs std_msgs)
target_include_directories(d435_camera_node PRIVATE ${REALSENSE2_INCLUDE_DIR})
target_link_libraries(d435_camera_node ${REALSENSE2_LIBRARY})

install(TARGETS
  d435_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

# 末尾加上这段即可
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/odom_calculator.py
  scripts/rgbd_object_approach.py
  scripts/direction.py
  scripts/align_to_obstacles.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
