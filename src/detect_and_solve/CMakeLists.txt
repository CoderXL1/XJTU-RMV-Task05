cmake_minimum_required(VERSION 3.8)
project(detect_and_solve)

# set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# include directories
include_directories(
  include
)


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# executable
add_executable(detect_and_solve
  src/util.cpp
  src/node.cpp
  src/recognition.cpp
)

# link libraries
target_link_libraries(detect_and_solve
  ${OpenCV_LIBS}
)

# ament dependencies
ament_target_dependencies(detect_and_solve
  rclcpp
  std_msgs
  cv_bridge
  sensor_msgs
  tf2
  tf2_geometry_msgs
)

# install
install(TARGETS detect_and_solve
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
