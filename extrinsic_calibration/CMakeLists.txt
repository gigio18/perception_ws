cmake_minimum_required(VERSION 3.8)
project(extrinsic_calibration)

# Set compiler flags for GNUCXX or Clang
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Set specific CMake policy
cmake_policy(SET CMP0074 NEW)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL COMPONENTS common registration features REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare dependencies
set(dependencies
  rclcpp
  sensor_msgs
  PCL
  pcl_conversions
  pcl_ros
)

# Define executable and it is dependencies
add_executable(extrinsic src/calib.cpp)

target_include_directories(extrinsic PRIVATE ./include/)

ament_export_include_directories(include)

ament_target_dependencies(extrinsic ${dependencies})

target_link_libraries(extrinsic 
  ${PCL_LIBRARIES} 
  ${YAML_CPP_LIBRARIES}
)

# Install executable
install(TARGETS 
  extrinsic 
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()