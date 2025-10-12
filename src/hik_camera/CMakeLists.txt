cmake_minimum_required(VERSION 3.8)
project(hik_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include directories
include_directories(
  include
  /opt/MVS/include
)

# link directories
link_directories(/opt/MVS/lib/64)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# executable
add_executable(hik_camera_node
  # src/hik_camera_main.cpp
  src/hik_camera_driver.cpp
  src/hik_camera_node.cpp
)

# link libraries
target_link_libraries(hik_camera_node
  ${OpenCV_LIBS}
  MvCameraControl
)

# ament dependencies
ament_target_dependencies(hik_camera_node
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
)

# install
install(TARGETS hik_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
