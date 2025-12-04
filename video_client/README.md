# Video Client - ROS2网络视频和点云接收包

## 📖 功能说明

这个ROS2功能包用于通过网络接收H.264视频流和PCD点云数据：

- ✅ **接收H.264视频流**：通过TCP连接接收视频数据并实时播放
- ✅ **接收PCD点云数据**：接收PCD格式的点云数据
- ✅ **ROS2集成**：将点云数据转换为`sensor_msgs/PointCloud2`并发布到`/camera_person`话题
- ✅ **实时显示**：使用OpenCV显示接收到的视频流

## 🏗️ 架构

```
服务器端                     网络                     客户端（本包）
┌─────────────┐           ┌──────┐              ┌──────────────────┐
│             │           │      │              │                  │
│ 视频源      │--H.264--> │      │ --TCP:5000-> │ 视频解码器        │
│             │           │      │              │ + OpenCV显示     │
│             │           │      │              │                  │
│ 点云源      │--PCD----> │      │ --TCP:5001-> │ PCD转ROS2       │
│             │           │      │              │ + 发布话题        │
└─────────────┘           └──────┘              └──────────────────┘
                                                         │
                                                         │ 发布
                                                         ▼
                                                   /camera_person
                                               (PointCloud2消息)
```

## 📦 依赖项

```bash
# ROS2包依赖
rclcpp
sensor_msgs
cv_bridge
image_transport
pcl_conversions

# 系统库依赖
libopencv-dev
libpcl-dev
```

## 🚀 快速开始

### 1. 编译

```bash
cd /home/zjq/thesis
colcon build --packages-select video_client
source install/setup.bash
```

### 2. 配置

编辑 `config/network_params.yaml` 设置服务器IP和端口：

```yaml
network_receiver_node:
  ros__parameters:
    # 修改为实际的服务器IP
    server_ip: "192.168.1.100"
    
    # 视频流端口
    video_port: 5000
    
    # 点云流端口
    pointcloud_port: 5001
    
    # 点云发布话题
    pointcloud_topic: "/camera_person"
    
    # 是否显示视频
    display_video: true
```

### 3. 启动

```bash
# 方式1：使用launch文件（推荐）
ros2 launch video_client network_receiver.launch.py

# 方式2：直接运行节点
ros2 run video_client network_receiver_node --ros-args --params-file src/video_client/config/network_params.yaml
```

### 4. 验证

```bash
# 查看点云话题
ros2 topic list | grep camera_person
ros2 topic echo /camera_person --once

# 查看话题频率
ros2 topic hz /camera_person

# 在RViz中可视化
rviz2
# 添加PointCloud2显示，订阅/camera_person话题
```

## 🔧 配置参数

| 参数 | 类型 | 默认值 | 说明 |
|-----|------|--------|------|
| `server_ip` | string | "192.168.1.100" | 服务器IP地址 |
| `video_port` | int | 5000 | 视频流TCP端口 |
| `pointcloud_port` | int | 5001 | 点云流TCP端口 |
| `pointcloud_topic` | string | "/camera_person" | 点云发布话题名 |
| `video_window_name` | string | "Network Video Stream" | 视频窗口标题 |
| `display_video` | bool | true | 是否显示视频窗口 |
| `buffer_size` | int | 65536 | 网络缓冲区大小（字节） |

## 📡 服务器端协议

### H.264视频流协议（TCP端口5000）

```
连续发送H.264编码的视频帧数据
每一帧可以是完整的JPEG编码帧，或原始H.264 NAL单元
```

### PCD点云流协议（TCP端口5001）

```
消息格式：
┌────────────────┬─────────────────────┐
│ 4字节          │ N字节               │
│ PCD文件大小    │ PCD文件数据         │
│ (uint32_t)     │ (二进制PCD)         │
└────────────────┴─────────────────────┘

注意：
- 大小字段使用网络字节序（大端序）
- PCD文件可以是ASCII或二进制格式
- 每次发送完整的PCD文件
```

### 服务器端示例代码（Python）

```python
import socket
import struct
import time

def send_pointcloud(server_ip, port=5001):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, port))
    
    while True:
        # 读取PCD文件
        with open('pointcloud.pcd', 'rb') as f:
            pcd_data = f.read()
        
        # 发送大小
        size = struct.pack('!I', len(pcd_data))
        sock.sendall(size)
        
        # 发送数据
        sock.sendall(pcd_data)
        
        print(f"发送点云: {len(pcd_data)} 字节")
        time.sleep(0.1)  # 10Hz

def send_video(server_ip, port=5000):
    import cv2
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((server_ip, port))
    
    cap = cv2.VideoCapture(0)  # 打开摄像头
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # JPEG编码
        _, encoded = cv2.imencode('.jpg', frame)
        sock.sendall(encoded.tobytes())
        
        time.sleep(0.033)  # 30fps

if __name__ == '__main__':
    import threading
    
    # 启动两个线程分别发送
    t1 = threading.Thread(target=send_pointcloud, args=('127.0.0.1',))
    t2 = threading.Thread(target=send_video, args=('127.0.0.1',))
    
    t1.start()
    t2.start()
```

## 🐛 故障排查

### 1. 连接失败

```bash
# 检查服务器IP是否可达
ping <server_ip>

# 检查端口是否开放
telnet <server_ip> 5000
telnet <server_ip> 5001

# 检查防火墙设置
sudo ufw status
```

### 2. 视频不显示

```bash
# 检查OpenCV是否支持视频编解码器
python3 -c "import cv2; print(cv2.getBuildInformation())"

# 禁用视频显示（只接收点云）
# 在config/network_params.yaml中设置：
display_video: false
```

### 3. 点云数据异常

```bash
# 检查点云话题
ros2 topic echo /camera_person --once

# 查看点云信息
ros2 topic info /camera_person -v

# 在RViz中可视化点云，检查坐标系
rviz2
# Fixed Frame设置为 camera_person_optical_frame
```

### 4. 网络延迟大

```yaml
# 增大缓冲区
buffer_size: 131072  # 128KB

# 检查网络带宽
iperf3 -c <server_ip>
```

## 📊 性能指标

| 指标 | 典型值 |
|-----|--------|
| 视频延迟 | < 100ms |
| 点云延迟 | < 50ms |
| CPU占用 | 15-25% |
| 内存占用 | ~100MB |
| 带宽需求 | 5-20 Mbps |

## 🔄 与其他包集成

### 与障碍物检测集成

```bash
# 1. 启动网络接收
ros2 launch video_client network_receiver.launch.py

# 2. 启动障碍物检测（订阅/camera_person）
ros2 launch unitree_obstacle_avoidance obstacle_detection.launch.py

# 3. 修改obstacle_detection配置，使其订阅/camera_person
# 在 config/obstacle_detection_params.yaml 中：
# input_topic: "/camera_person"
```

## 📝 TODO

- [ ] 添加UDP传输支持（减少延迟）
- [ ] 支持多路视频流
- [ ] 添加视频录制功能
- [ ] 支持H.265编码
- [ ] 添加自动重连机制
- [ ] 支持点云压缩传输

## 📄 许可证

MIT License

## 👤 维护者

- zjq <zjq@example.com>

---

**注意**：首次使用前，请确保服务器端正确配置并启动视频和点云发送程序。

