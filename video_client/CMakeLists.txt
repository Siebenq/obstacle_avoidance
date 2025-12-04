cmake_minimum_required(VERSION 3.8)
project(video_client)

# 设置C++标准
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(OpenCV REQUIRED)

# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# 创建网络接收节点
add_executable(network_receiver_node 
  src/network_receiver_node.cpp
)

ament_target_dependencies(network_receiver_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  pcl_conversions
)

target_link_libraries(network_receiver_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
)

# 安装节点
install(TARGETS
  network_receiver_node
  DESTINATION lib/${PROJECT_NAME}
)

# 安装配置文件和启动文件
install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

