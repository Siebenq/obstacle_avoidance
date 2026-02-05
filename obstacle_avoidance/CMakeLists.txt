cmake_minimum_required(VERSION 3.8)
project(obstacle_avoidance)

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
find_package(std_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters segmentation)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)



# 包含目录
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


# 消息生成
find_package(rosidl_default_generators REQUIRED)

# 生成自定义消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EllipseObstacle.msg"
  "msg/EllipseObstacleArray.msg"
  "msg/TransformMatrix.msg"
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# 链接自定义消息
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME}
  "rosidl_typesupport_cpp")





# 创建点云融合可执行文件
add_executable(pointcloud_fusion_node src/pointcloud_fusion_node.cpp)

ament_target_dependencies(pointcloud_fusion_node
  rclcpp
  sensor_msgs
  pcl_conversions
  tf2
  tf2_ros
  tf2_eigen
  geometry_msgs
  visualization_msgs
)

target_link_libraries(pointcloud_fusion_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)





# 创建障碍物检测可执行文件
add_executable(obstacle_detection_node src/obstacle_detection_node.cpp)

ament_target_dependencies(obstacle_detection_node
  rclcpp
  sensor_msgs
  pcl_conversions
  geometry_msgs
  visualization_msgs
)

# 链接到可执行文件上
target_link_libraries(obstacle_detection_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)






# 创建ICP标定可执行文件
add_executable(icp_calibration_node src/icp_calibration_node.cpp)

ament_target_dependencies(icp_calibration_node
  rclcpp
  sensor_msgs
  pcl_conversions
)

target_link_libraries(icp_calibration_node
  ${PCL_LIBRARIES}
  ${cpp_typesupport_target}
)






# # 查找CasADi
# find_package(casadi QUIET)
# if(NOT casadi_FOUND)
#   message(STATUS "CasADi not found via find_package, trying pkg-config")
#   find_package(PkgConfig REQUIRED)
#   pkg_check_modules(CASADI REQUIRED casadi)
#   set(CASADI_INCLUDE_DIRS ${CASADI_INCLUDEDIR})
#   set(CASADI_LIBRARIES ${CASADI_LIBRARIES})
# else()
#   set(CASADI_INCLUDE_DIRS ${casadi_INCLUDE_DIRS})
#   set(CASADI_LIBRARIES casadi)
# endif()

# # 创建MPC控制器可执行文件
# add_executable(mpc_controller_node src/mpc_controller_node.cpp)

# ament_target_dependencies(mpc_controller_node
#   rclcpp
#   geometry_msgs
#   nav_msgs
#   visualization_msgs
# )


# # 包含CasADi头文件
# target_include_directories(mpc_controller_node PRIVATE ${CASADI_INCLUDE_DIRS})

# # 链接CasADi库和Eigen3
# target_link_libraries(mpc_controller_node 
#   ${CASADI_LIBRARIES}
#   ${cpp_typesupport_target}
#   Eigen3::Eigen
# )




# 创建APF控制器可执行文件
add_executable(apf_controller_node src/apf_controller_node.cpp)

ament_target_dependencies(apf_controller_node
  rclcpp
  geometry_msgs
)

target_link_libraries(apf_controller_node
  ${cpp_typesupport_target}
  Eigen3::Eigen
)




# 安装
install(TARGETS
  pointcloud_fusion_node
  obstacle_detection_node
  icp_calibration_node
# mpc_controller_node
  apf_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# # 安装脚本
# install(PROGRAMS
#   scripts/test_velocity_publisher.py
#   scripts/calibrate_transform.py
#   scripts/diagnose_transform.py
#   scripts/test_obstacle_params.py
#   scripts/diagnose_frame_alignment.py
#   DESTINATION lib/${PROJECT_NAME}
# )

# install(FILES
#   scripts/README_TEST.md
#   DESTINATION share/${PROJECT_NAME}/scripts
# )

# 测试
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

