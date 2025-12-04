#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>

/**
 * @brief 网络接收节点 - 接收H.264视频流和PCD点云数据
 */
class NetworkReceiverNode : public rclcpp::Node
{
public:
  NetworkReceiverNode() : Node("network_receiver_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("server_ip", "192.168.1.100");
    this->declare_parameter<int>("video_port", 5000);
    this->declare_parameter<int>("pointcloud_port", 5001);
    this->declare_parameter<std::string>("pointcloud_topic", "/camera_person");
    this->declare_parameter<std::string>("video_window_name", "Network Video Stream");
    this->declare_parameter<bool>("display_video", true);
    this->declare_parameter<int>("buffer_size", 65536);
    
    // 获取参数
    server_ip_ = this->get_parameter("server_ip").as_string();
    video_port_ = this->get_parameter("video_port").as_int();
    pointcloud_port_ = this->get_parameter("pointcloud_port").as_int();
    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    video_window_name_ = this->get_parameter("video_window_name").as_string();
    display_video_ = this->get_parameter("display_video").as_bool();
    buffer_size_ = this->get_parameter("buffer_size").as_int();
    
    // 创建点云发布者
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      pointcloud_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "📡 网络接收节点已启动");
    RCLCPP_INFO(this->get_logger(), "  服务器IP: %s", server_ip_.c_str());
    RCLCPP_INFO(this->get_logger(), "  视频端口: %d", video_port_);
    RCLCPP_INFO(this->get_logger(), "  点云端口: %d", pointcloud_port_);
    RCLCPP_INFO(this->get_logger(), "  点云话题: %s", pointcloud_topic_.c_str());
    
    // 启动接收线程
    running_ = true;
    video_thread_ = std::thread(&NetworkReceiverNode::receiveVideoStream, this);
    pointcloud_thread_ = std::thread(&NetworkReceiverNode::receivePointCloudStream, this);
  }
  
  ~NetworkReceiverNode()
  {
    running_ = false;
    
    if (video_thread_.joinable()) {
      video_thread_.join();
    }
    
    if (pointcloud_thread_.joinable()) {
      pointcloud_thread_.join();
    }
    
    if (display_video_) {
      cv::destroyAllWindows();
    }
    
    RCLCPP_INFO(this->get_logger(), "📡 网络接收节点已关闭");
  }

private:
  /**
   * @brief 接收H.264视频流
   */
  void receiveVideoStream()
  {
    RCLCPP_INFO(this->get_logger(), "🎥 启动视频流接收线程...");
    
    // 创建TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(this->get_logger(), "❌ 创建视频socket失败: %s", strerror(errno));
      return;
    }
    
    // 设置服务器地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(video_port_);
    
    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "❌ 无效的服务器IP地址: %s", server_ip_.c_str());
      close(sock);
      return;
    }
    
    // 连接到服务器
    RCLCPP_INFO(this->get_logger(), "🔌 正在连接到视频服务器 %s:%d...", 
                server_ip_.c_str(), video_port_);
    
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "❌ 连接视频服务器失败: %s", strerror(errno));
      close(sock);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ 视频服务器连接成功");
    
    // 使用OpenCV解码H.264流
    cv::VideoCapture cap;
    std::string pipeline = "appsrc ! h264parse ! avdec_h264 ! videoconvert ! appsink";
    
    // 尝试使用GStreamer管道
    if (!cap.open(pipeline, cv::CAP_GSTREAMER)) {
      RCLCPP_WARN(this->get_logger(), "⚠️  GStreamer管道打开失败，尝试直接解码");
      // 如果GStreamer不可用，我们将直接处理原始数据
    }
    
    std::vector<uint8_t> buffer(buffer_size_);
    std::vector<uint8_t> frame_buffer;
    
    while (running_ && rclcpp::ok()) {
      // 接收数据
      int bytes_received = recv(sock, buffer.data(), buffer.size(), 0);
      
      if (bytes_received <= 0) {
        if (bytes_received == 0) {
          RCLCPP_WARN(this->get_logger(), "⚠️  视频服务器连接关闭");
        } else {
          RCLCPP_ERROR(this->get_logger(), "❌ 接收视频数据失败: %s", strerror(errno));
        }
        break;
      }
      
      // 累积帧数据
      frame_buffer.insert(frame_buffer.end(), buffer.begin(), buffer.begin() + bytes_received);
      
      // 尝试解码帧（简单实现：假设每次接收的是完整帧）
      cv::Mat frame = cv::imdecode(frame_buffer, cv::IMREAD_COLOR);
      
      if (!frame.empty()) {
        // 显示视频
        if (display_video_) {
          cv::imshow(video_window_name_, frame);
          cv::waitKey(1);
        }
        
        // 清空缓冲区准备下一帧
        frame_buffer.clear();
        
        RCLCPP_DEBUG(this->get_logger(), "📹 接收视频帧: %dx%d", frame.cols, frame.rows);
      }
    }
    
    close(sock);
    RCLCPP_INFO(this->get_logger(), "🎥 视频流接收线程已退出");
  }
  
  /**
   * @brief 接收PCD点云流
   */
  void receivePointCloudStream()
  {
    RCLCPP_INFO(this->get_logger(), "☁️  启动点云流接收线程...");
    
    // 创建TCP socket
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      RCLCPP_ERROR(this->get_logger(), "❌ 创建点云socket失败: %s", strerror(errno));
      return;
    }
    
    // 设置服务器地址
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(pointcloud_port_);
    
    if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
      RCLCPP_ERROR(this->get_logger(), "❌ 无效的服务器IP地址: %s", server_ip_.c_str());
      close(sock);
      return;
    }
    
    // 连接到服务器
    RCLCPP_INFO(this->get_logger(), "🔌 正在连接到点云服务器 %s:%d...", 
                server_ip_.c_str(), pointcloud_port_);
    
    if (connect(sock, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "❌ 连接点云服务器失败: %s", strerror(errno));
      close(sock);
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ 点云服务器连接成功");
    
    std::vector<uint8_t> buffer(buffer_size_);
    
    while (running_ && rclcpp::ok()) {
      // 先接收PCD文件大小（4字节）
      uint32_t pcd_size = 0;
      int bytes_received = recv(sock, &pcd_size, sizeof(pcd_size), MSG_WAITALL);
      
      if (bytes_received != sizeof(pcd_size)) {
        if (bytes_received == 0) {
          RCLCPP_WARN(this->get_logger(), "⚠️  点云服务器连接关闭");
        } else {
          RCLCPP_ERROR(this->get_logger(), "❌ 接收点云大小失败");
        }
        break;
      }
      
      // 转换字节序（如果需要）
      pcd_size = ntohl(pcd_size);
      
      if (pcd_size == 0 || pcd_size > 100 * 1024 * 1024) { // 限制最大100MB
        RCLCPP_ERROR(this->get_logger(), "❌ 无效的点云大小: %u", pcd_size);
        continue;
      }
      
      // 接收PCD数据
      std::vector<uint8_t> pcd_data(pcd_size);
      uint32_t total_received = 0;
      
      while (total_received < pcd_size) {
        int to_receive = std::min(buffer_size_, (int)(pcd_size - total_received));
        bytes_received = recv(sock, pcd_data.data() + total_received, to_receive, 0);
        
        if (bytes_received <= 0) {
          RCLCPP_ERROR(this->get_logger(), "❌ 接收点云数据失败");
          break;
        }
        
        total_received += bytes_received;
      }
      
      if (total_received != pcd_size) {
        RCLCPP_ERROR(this->get_logger(), "❌ 点云数据接收不完整: %u/%u", 
                     total_received, pcd_size);
        continue;
      }
      
      // 将数据保存为临时PCD文件并加载
      std::string temp_file = "/tmp/temp_pointcloud.pcd";
      std::ofstream ofs(temp_file, std::ios::binary);
      ofs.write(reinterpret_cast<char*>(pcd_data.data()), pcd_size);
      ofs.close();
      
      // 加载PCD文件
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(temp_file, *cloud) == -1) {
        RCLCPP_ERROR(this->get_logger(), "❌ 加载PCD文件失败");
        continue;
      }
      
      // 转换为ROS2消息并发布
      sensor_msgs::msg::PointCloud2 cloud_msg;
      pcl::toROSMsg(*cloud, cloud_msg);
      cloud_msg.header.stamp = this->now();
      cloud_msg.header.frame_id = "camera_person_optical_frame";
      
      pointcloud_pub_->publish(cloud_msg);
      
      RCLCPP_INFO(this->get_logger(), "☁️  发布点云: %zu 点, 大小: %.2f KB", 
                  cloud->points.size(), pcd_size / 1024.0);
      
      // 删除临时文件
      std::remove(temp_file.c_str());
    }
    
    close(sock);
    RCLCPP_INFO(this->get_logger(), "☁️  点云流接收线程已退出");
  }
  
  // ROS相关
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  
  // 网络参数
  std::string server_ip_;
  int video_port_;
  int pointcloud_port_;
  std::string pointcloud_topic_;
  std::string video_window_name_;
  bool display_video_;
  int buffer_size_;
  
  // 线程控制
  std::atomic<bool> running_;
  std::thread video_thread_;
  std::thread pointcloud_thread_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<NetworkReceiverNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("network_receiver"), "异常: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}

