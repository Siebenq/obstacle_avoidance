cmake_minimum_required(VERSION 3.8)
project(video_client)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# 查找FFmpeg库
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBAV REQUIRED
  libavcodec
  libavutil
  libswscale
)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBAV_INCLUDE_DIRS}
)

# 创建WebSocket视频接收节点
add_executable(network_receiver_node 
  src/network_receiver_node.cpp
)

ament_target_dependencies(network_receiver_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)

target_link_libraries(network_receiver_node
  ${OpenCV_LIBS}
  ${Boost_LIBRARIES}
  ${LIBAV_LIBRARIES}
  pthread
)

target_link_directories(network_receiver_node PUBLIC
  ${LIBAV_LIBRARY_DIRS}
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
