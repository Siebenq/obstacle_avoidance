#!/bin/bash

# Video Client 快速命令脚本
# 使用方法：chmod +x 快速命令.sh && ./快速命令.sh [命令]

WORKSPACE="/home/zjq/thesis"

case "$1" in
  "build")
    echo "🔨 编译 video_client..."
    cd "$WORKSPACE"
    colcon build --packages-select video_client
    echo "✅ 编译完成"
    ;;
    
  "start")
    echo "🚀 启动 video_client..."
    cd "$WORKSPACE"
    source install/setup.bash
    ros2 launch video_client network_receiver.launch.py
    ;;
    
  "start-node")
    echo "🚀 启动节点（不使用launch）..."
    cd "$WORKSPACE"
    source install/setup.bash
    ros2 run video_client network_receiver_node
    ;;
    
  "test-server")
    echo "🧪 启动测试服务器..."
    cd "$WORKSPACE/src/video_client/test_server"
    python3 video_pointcloud_server.py
    ;;
    
  "check-topic")
    echo "📊 检查 /camera_person 话题..."
    cd "$WORKSPACE"
    source install/setup.bash
    echo ""
    echo "话题列表:"
    ros2 topic list | grep camera_person
    echo ""
    echo "话题信息:"
    ros2 topic info /camera_person -v
    echo ""
    echo "话题频率:"
    timeout 5 ros2 topic hz /camera_person
    ;;
    
  "show-data")
    echo "👁️  显示点云数据..."
    cd "$WORKSPACE"
    source install/setup.bash
    ros2 topic echo /camera_person --once
    ;;
    
  "rviz")
    echo "🎨 启动 RViz..."
    cd "$WORKSPACE"
    source install/setup.bash
    rviz2 &
    sleep 2
    echo ""
    echo "在RViz中："
    echo "1. 点击 Add"
    echo "2. 选择 PointCloud2"
    echo "3. Topic设置为 /camera_person"
    echo "4. Fixed Frame设置为 camera_person_optical_frame"
    ;;
    
  "node-info")
    echo "ℹ️  节点信息..."
    cd "$WORKSPACE"
    source install/setup.bash
    ros2 node list | grep network_receiver
    echo ""
    ros2 node info /network_receiver_node
    ;;
    
  "edit-config")
    echo "✏️  编辑配置文件..."
    nano "$WORKSPACE/src/video_client/config/network_params.yaml"
    ;;
    
  "install-deps")
    echo "📦 安装依赖..."
    echo "安装系统依赖..."
    sudo apt update
    sudo apt install -y libpcl-dev libopencv-dev
    echo ""
    echo "安装Python依赖（测试服务器）..."
    pip3 install opencv-python numpy
    echo "✅ 依赖安装完成"
    ;;
    
  "clean")
    echo "🧹 清理编译文件..."
    cd "$WORKSPACE"
    rm -rf build/video_client install/video_client log/latest_build
    echo "✅ 清理完成"
    ;;
    
  "help"|*)
    echo "======================================================================"
    echo "                   Video Client 快速命令"
    echo "======================================================================"
    echo ""
    echo "使用方法: ./快速命令.sh [命令]"
    echo ""
    echo "可用命令:"
    echo "  build         - 编译video_client包"
    echo "  start         - 启动客户端（使用launch）"
    echo "  start-node    - 启动节点（不使用launch）"
    echo "  test-server   - 启动Python测试服务器"
    echo "  check-topic   - 检查/camera_person话题状态"
    echo "  show-data     - 显示一帧点云数据"
    echo "  rviz          - 启动RViz可视化"
    echo "  node-info     - 显示节点信息"
    echo "  edit-config   - 编辑配置文件"
    echo "  install-deps  - 安装所有依赖"
    echo "  clean         - 清理编译文件"
    echo "  help          - 显示此帮助信息"
    echo ""
    echo "示例:"
    echo "  ./快速命令.sh build              # 编译"
    echo "  ./快速命令.sh test-server        # 启动测试服务器"
    echo "  ./快速命令.sh start              # 启动客户端"
    echo "  ./快速命令.sh check-topic        # 检查话题"
    echo ""
    echo "快速测试流程:"
    echo "  1. ./快速命令.sh edit-config     # 修改server_ip为 127.0.0.1"
    echo "  2. ./快速命令.sh test-server     # 终端1: 启动测试服务器"
    echo "  3. ./快速命令.sh start           # 终端2: 启动客户端"
    echo "  4. ./快速命令.sh check-topic     # 终端3: 验证"
    echo ""
    echo "======================================================================"
    ;;
esac

