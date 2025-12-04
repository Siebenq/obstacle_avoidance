# Video Client - 快速开始指南

## 🎯 5分钟快速测试

### 步骤1: 编译包

```bash
cd /home/zjq/thesis
colcon build --packages-select video_client
source install/setup.bash
```

### 步骤2: 启动测试服务器

在**第一个终端**中启动测试服务器：

```bash
cd /home/zjq/thesis/src/video_client/test_server
chmod +x video_pointcloud_server.py

# 启动服务器（需要Python3）
python3 video_pointcloud_server.py

# 如果没有安装OpenCV，先安装：
# pip3 install opencv-python numpy
```

**预期输出**：

```
============================================================
🚀 视频和点云测试服务器
============================================================
配置:
  🎥 视频: 端口 5000, 30Hz
  ☁️  点云: 端口 5001, 10Hz
============================================================
按 Ctrl+C 停止服务器

☁️  启动点云服务器 - 端口: 5001, 频率: 10Hz
✅ 创建测试PCD文件: /tmp/test_cloud.pcd (1000点)
⏳ 等待客户端连接 (端口 5001)...
🎥 启动视频服务器 - 端口: 5000, 频率: 30Hz
⏳ 等待客户端连接 (端口 5000)...
```

### 步骤3: 修改配置（指定服务器IP）

编辑配置文件 `config/network_params.yaml`：

```bash
# 如果服务器在本机，使用：
server_ip: "127.0.0.1"

# 如果服务器在其他机器，使用实际IP：
server_ip: "192.168.1.100"
```

### 步骤4: 启动客户端

在**第二个终端**中启动客户端：

```bash
cd /home/zjq/thesis
source install/setup.bash

# 启动客户端
ros2 launch video_client network_receiver.launch.py
```

**预期输出**：

```
[network_receiver_node-1] 📡 网络接收节点已启动
[network_receiver_node-1]   服务器IP: 127.0.0.1
[network_receiver_node-1]   视频端口: 5000
[network_receiver_node-1]   点云端口: 5001
[network_receiver_node-1]   点云话题: /camera_person
[network_receiver_node-1] 🎥 启动视频流接收线程...
[network_receiver_node-1] ☁️  启动点云流接收线程...
[network_receiver_node-1] 🔌 正在连接到视频服务器 127.0.0.1:5000...
[network_receiver_node-1] 🔌 正在连接到点云服务器 127.0.0.1:5001...
[network_receiver_node-1] ✅ 视频服务器连接成功
[network_receiver_node-1] ✅ 点云服务器连接成功
[network_receiver_node-1] ☁️  发布点云: 1000 点, 大小: 9.82 KB
```

同时：
- 📺 **视频窗口**会弹出，显示接收到的视频流
- ☁️ **点云数据**会发布到`/camera_person`话题

### 步骤5: 验证点云数据

在**第三个终端**中验证：

```bash
# 查看点云话题
ros2 topic list | grep camera_person

# 查看点云数据
ros2 topic echo /camera_person --once

# 查看点云频率
ros2 topic hz /camera_person

# 预期输出: average rate: 10.000 (根据服务器设置)
```

### 步骤6: 在RViz中可视化

```bash
# 启动RViz
rviz2
```

在RViz中：
1. 点击左下角 **Add**
2. 选择 **PointCloud2**
3. 设置 **Topic** 为 `/camera_person`
4. 设置 **Fixed Frame** 为 `camera_person_optical_frame`
5. 点云应该显示出来！

## 🔧 自定义测试

### 使用自己的PCD文件

```bash
# 假设你有一个PCD文件: my_cloud.pcd
python3 video_pointcloud_server.py --pcd-file my_cloud.pcd
```

### 调整帧率

```bash
# 视频30fps，点云5Hz
python3 video_pointcloud_server.py --video-hz 30 --pointcloud-hz 5
```

### 只测试点云（不发送视频）

```bash
python3 video_pointcloud_server.py --no-video
```

### 只测试视频（不发送点云）

```bash
python3 video_pointcloud_server.py --no-pointcloud
```

### 使用真实摄像头

如果你的测试服务器机器有摄像头，服务器会自动检测并使用：

```bash
# 服务器会自动尝试打开摄像头 (设备ID=0)
python3 video_pointcloud_server.py

# 如果检测到摄像头，你会看到：
# ✅ 视频客户端已连接: ('127.0.0.1', 54321)
# 🎥 已发送视频帧: 30 (大小: 25.34KB)  ← 真实摄像头数据
```

如果没有摄像头，会生成彩色测试图案。

## 🌐 跨机器测试

### 场景：服务器在机器A，客户端在机器B

**机器A（服务器）**：

```bash
# 1. 查看机器A的IP
ip addr show

# 假设是: 192.168.1.100

# 2. 启动服务器（监听所有接口）
python3 video_pointcloud_server.py
# 服务器会绑定到 0.0.0.0，允许外部连接
```

**机器B（客户端）**：

```bash
# 1. 修改配置文件
# config/network_params.yaml:
server_ip: "192.168.1.100"  # 机器A的IP

# 2. 启动客户端
ros2 launch video_client network_receiver.launch.py
```

## 🐛 常见问题

### 问题1: 连接失败

```
❌ 连接视频服务器失败: Connection refused
```

**解决方案**：
- 检查服务器是否已启动
- 检查IP地址是否正确
- 检查防火墙设置：
  ```bash
  # 在服务器机器上
  sudo ufw allow 5000/tcp
  sudo ufw allow 5001/tcp
  ```

### 问题2: 视频窗口不显示

```
⚠️  GStreamer管道打开失败
```

**解决方案**：
- 这是正常的警告，程序会自动降级处理
- 如果完全看不到视频，在配置中禁用显示：
  ```yaml
  display_video: false
  ```

### 问题3: 点云数据不正确

```bash
# 检查PCD文件格式
head /tmp/test_cloud.pcd

# 应该看到PCD文件头：
# # .PCD v0.7 - Point Cloud Data file format
# VERSION 0.7
# ...
```

### 问题4: Python依赖缺失

```bash
# 安装所需的Python包
pip3 install opencv-python numpy
```

## 📊 性能测试

### 检查网络延迟

```bash
# 在客户端机器上
ping <server_ip>

# 预期: < 1ms (本地), < 10ms (局域网)
```

### 检查带宽

```bash
# 安装iperf3
sudo apt install iperf3

# 在服务器上
iperf3 -s

# 在客户端上
iperf3 -c <server_ip>

# 预期: > 100 Mbits/sec (千兆网络)
```

### 监控ROS话题

```bash
# 查看所有话题
ros2 topic list

# 监控话题带宽
ros2 topic bw /camera_person

# 预期输出:
# average: 980.00KB/s (约1MB/s，10Hz点云)
```

## 🎓 下一步

完成快速测试后，你可以：

1. **集成到你的机器人系统**：
   ```bash
   # 将点云数据传递给障碍物检测
   ros2 launch unitree_obstacle_avoidance obstacle_detection.launch.py
   ```

2. **编写自己的服务器**：参考 `test_server/video_pointcloud_server.py`

3. **修改客户端节点**：编辑 `src/network_receiver_node.cpp`

4. **调整参数**：编辑 `config/network_params.yaml`

## 📚 相关文档

- 完整文档: `README.md`
- 服务器协议: `README.md` 的"服务器端协议"部分
- 故障排查: `README.md` 的"故障排查"部分

---

**祝你使用愉快！** 🚀

如有问题，请查看完整README或联系维护者。

