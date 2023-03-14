cmake_minimum_required(VERSION 3.8)
project(jackal_3d_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL 1.12 REQUIRED)

add_executable(filter src/point_cloud_processing.cpp)

include_directories(include ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(filter ${PCL_LIBRARIES})

ament_export_include_directories(include)

ament_target_dependencies(filter
  rclcpp
  sensor_msgs
  pcl_conversions
)

install(TARGETS
  filter
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  models
  utils
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY
  scripts
  src
  jackal_3d_slam
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)
 
ament_python_install_package(${PROJECT_NAME})
 
install(PROGRAMS
  scripts/object_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
