cmake_minimum_required(VERSION 3.8)
project(camera_calibration)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# 包含目录
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# 创建ICP标定可执行文件
add_executable(icp_calibration_node src/icp_calibration_node.cpp)

ament_target_dependencies(icp_calibration_node
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
  tf2
  tf2_ros
)

target_link_libraries(icp_calibration_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
)

# 安装可执行文件
install(TARGETS
  icp_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装launch和配置文件
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
