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

get_filename_component(WORKSPACE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
unset(realsense2_DIR CACHE)
unset(realsense2_DIR)
set(REALSENSE2_INSTALL_DIR "${WORKSPACE_ROOT}/install/librealsense2/lib/cmake/realsense2")
set(REALSENSE2_SOURCE_INCLUDE_DIR "${WORKSPACE_ROOT}/librealsense/include")
set(REALSENSE2_SOURCE_LIBRARY "${WORKSPACE_ROOT}/librealsense/build/Release/librealsense2.so")

if(EXISTS "${REALSENSE2_INSTALL_DIR}/realsense2Config.cmake")
  set(realsense2_DIR "${REALSENSE2_INSTALL_DIR}")
  find_package(realsense2 CONFIG REQUIRED)
elseif(EXISTS "${REALSENSE2_SOURCE_INCLUDE_DIR}/librealsense2/rs.hpp" AND EXISTS "${REALSENSE2_SOURCE_LIBRARY}")
  if(NOT TARGET realsense2::realsense2)
    add_library(realsense2::realsense2 SHARED IMPORTED)
    set_target_properties(realsense2::realsense2 PROPERTIES
      IMPORTED_LOCATION "${REALSENSE2_SOURCE_LIBRARY}"
      INTERFACE_INCLUDE_DIRECTORIES "${REALSENSE2_SOURCE_INCLUDE_DIR}"
    )
  endif()
else()
  message(FATAL_ERROR "realsense2 not found. Build the librealsense2 package or the workspace-local librealsense source tree first.")
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
target_link_libraries(d435_camera_node realsense2::realsense2)

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
  scripts/approach_lidar_box_face.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
