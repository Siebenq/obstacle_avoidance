#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>

#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <queue>
#include <cstring>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace beast = boost::beast;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;

/**
 * @brief WebSocket视频接收节点 - 通过WebSocket接收H.264视频流并实时显示
 */
class WebSocketVideoNode : public rclcpp::Node
{
public:
  WebSocketVideoNode() : Node("websocket_video_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("server_ip", "192.168.1.100");
    this->declare_parameter<int>("video_port", 5000);
    this->declare_parameter<std::string>("ws_path", "/video");
    this->declare_parameter<std::string>("video_window_name", "WebSocket Video Stream");
    this->declare_parameter<bool>("display_video", true);
    this->declare_parameter<std::string>("image_topic", "/camera/image_raw");
    this->declare_parameter<bool>("publish_image", false);
    
    // 获取参数
    server_ip_ = this->get_parameter("server_ip").as_string();
    video_port_ = this->get_parameter("video_port").as_int();
    ws_path_ = this->get_parameter("ws_path").as_string();
    video_window_name_ = this->get_parameter("video_window_name").as_string();
    display_video_ = this->get_parameter("display_video").as_bool();
    image_topic_ = this->get_parameter("image_topic").as_string();
    publish_image_ = this->get_parameter("publish_image").as_bool();
    
    // 创建图像发布者（可选）
    if (publish_image_) {
      image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_topic_, 10);
    }
    
    RCLCPP_INFO(this->get_logger(), "📡 WebSocket视频接收节点已启动");
    RCLCPP_INFO(this->get_logger(), "  服务器: ws://%s:%d%s", 
                server_ip_.c_str(), video_port_, ws_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "  显示视频: %s", display_video_ ? "是" : "否");
    if (publish_image_) {
      RCLCPP_INFO(this->get_logger(), "  图像话题: %s", image_topic_.c_str());
    }
    
    // 初始化H.264解码器
    if (!initDecoder()) {
      RCLCPP_ERROR(this->get_logger(), "❌ 初始化H.264解码器失败");
      return;
    }
    
    // 启动WebSocket接收线程
    running_ = true;
    ws_thread_ = std::thread(&WebSocketVideoNode::websocketThread, this);
    
    // 启动显示线程
    if (display_video_) {
      display_thread_ = std::thread(&WebSocketVideoNode::displayThread, this);
    }
  }
  
  ~WebSocketVideoNode()
  {
    running_ = false;
    
    if (ws_thread_.joinable()) {
      ws_thread_.join();
    }
    
    if (display_thread_.joinable()) {
      display_thread_.join();
    }
    
    // 清理解码器
    cleanupDecoder();
    
    if (display_video_) {
      cv::destroyAllWindows();
    }
    
    RCLCPP_INFO(this->get_logger(), "📡 WebSocket视频接收节点已关闭");
  }

private:
  /**
   * @brief 初始化FFmpeg H.264解码器
   */
  bool initDecoder()
  {
    // 查找H.264解码器
    const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
    if (!codec) {
      RCLCPP_ERROR(this->get_logger(), "找不到H.264解码器");
      return false;
    }
    
    // 分配解码器上下文
    codec_ctx_ = avcodec_alloc_context3(codec);
    if (!codec_ctx_) {
      RCLCPP_ERROR(this->get_logger(), "无法分配解码器上下文");
      return false;
    }
    
    // 设置解码参数
    codec_ctx_->thread_count = 4;  // 多线程解码
    
    // 打开解码器
    if (avcodec_open2(codec_ctx_, codec, nullptr) < 0) {
      RCLCPP_ERROR(this->get_logger(), "无法打开H.264解码器");
      return false;
    }
    
    // 分配帧
    frame_ = av_frame_alloc();
    frame_bgr_ = av_frame_alloc();
    
    if (!frame_ || !frame_bgr_) {
      RCLCPP_ERROR(this->get_logger(), "无法分配AVFrame");
      return false;
    }
    
    // 分配解析器
    parser_ = av_parser_init(AV_CODEC_ID_H264);
    if (!parser_) {
      RCLCPP_ERROR(this->get_logger(), "无法初始化H.264解析器");
      return false;
    }
    
    RCLCPP_INFO(this->get_logger(), "✅ H.264解码器初始化成功");
    return true;
  }
  
  /**
   * @brief 清理解码器资源
   */
  void cleanupDecoder()
  {
    if (sws_ctx_) {
      sws_freeContext(sws_ctx_);
      sws_ctx_ = nullptr;
    }
    
    if (frame_bgr_) {
      if (frame_bgr_->data[0]) {
        av_freep(&frame_bgr_->data[0]);
      }
      av_frame_free(&frame_bgr_);
    }
    
    if (frame_) {
      av_frame_free(&frame_);
    }
    
    if (parser_) {
      av_parser_close(parser_);
      parser_ = nullptr;
    }
    
    if (codec_ctx_) {
      avcodec_free_context(&codec_ctx_);
    }
  }
  
  /**
   * @brief 处理接收到的H.264数据
   */
  void onH264DataReceived(const uint8_t* data, size_t len)
  {
    std::lock_guard<std::mutex> lock(h264_mutex_);
    h264_buffer_.insert(h264_buffer_.end(), data, data + len);
    total_bytes_received_ += len;
    
    // 尝试解码
    decodeH264();
  }
  
  /**
   * @brief 解码H.264数据
   */
  void decodeH264()
  {
    if (h264_buffer_.empty()) return;
    
    AVPacket* pkt = av_packet_alloc();
    if (!pkt) return;
    
    const uint8_t* data = h264_buffer_.data();
    size_t data_size = h264_buffer_.size();
    
    while (data_size > 0) {
      // 解析H.264 NAL单元
      int ret = av_parser_parse2(parser_, codec_ctx_, 
                                  &pkt->data, &pkt->size,
                                  data, data_size,
                                  AV_NOPTS_VALUE, AV_NOPTS_VALUE, 0);
      
      if (ret < 0) {
        RCLCPP_WARN(this->get_logger(), "解析H.264数据失败");
        break;
      }
      
      data += ret;
      data_size -= ret;
      
      if (pkt->size > 0) {
        // 发送数据包到解码器
        ret = avcodec_send_packet(codec_ctx_, pkt);
        if (ret < 0) {
          if (ret != AVERROR(EAGAIN) && ret != AVERROR_EOF) {
            RCLCPP_WARN(this->get_logger(), "发送数据包失败: %d", ret);
          }
          continue;
        }
        
        // 接收解码后的帧
        while (ret >= 0) {
          ret = avcodec_receive_frame(codec_ctx_, frame_);
          if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
          } else if (ret < 0) {
            RCLCPP_WARN(this->get_logger(), "接收帧失败");
            break;
          }
          
          // 转换为BGR格式用于OpenCV显示
          convertFrameToBGR();
        }
      }
    }
    
    // 清除已处理的数据
    size_t consumed = h264_buffer_.size() - data_size;
    if (consumed > 0) {
      h264_buffer_.erase(h264_buffer_.begin(), h264_buffer_.begin() + consumed);
    }
    
    av_packet_free(&pkt);
  }
  
  /**
   * @brief 将解码后的帧转换为BGR格式
   */
  void convertFrameToBGR()
  {
    if (!frame_->data[0]) return;
    
    int width = frame_->width;
    int height = frame_->height;
    
    // 初始化或更新色彩空间转换器
    if (!sws_ctx_ || last_width_ != width || last_height_ != height) {
      if (sws_ctx_) {
        sws_freeContext(sws_ctx_);
      }
      
      sws_ctx_ = sws_getContext(width, height, (AVPixelFormat)frame_->format,
                                 width, height, AV_PIX_FMT_BGR24,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
      
      if (!sws_ctx_) {
        RCLCPP_ERROR(this->get_logger(), "无法创建色彩空间转换器");
        return;
      }
      
      // 重新分配BGR帧缓冲区
      if (frame_bgr_->data[0]) {
        av_freep(&frame_bgr_->data[0]);
      }
      
      int buf_size = av_image_alloc(frame_bgr_->data, frame_bgr_->linesize,
                                     width, height, AV_PIX_FMT_BGR24, 1);
      if (buf_size < 0) {
        RCLCPP_ERROR(this->get_logger(), "无法分配BGR缓冲区");
        return;
      }
      
      last_width_ = width;
      last_height_ = height;
      
      RCLCPP_INFO(this->get_logger(), "🎬 视频分辨率: %dx%d", width, height);
    }
    
    // 执行色彩空间转换
    sws_scale(sws_ctx_, frame_->data, frame_->linesize, 0, height,
              frame_bgr_->data, frame_bgr_->linesize);
    
    // 创建OpenCV Mat
    cv::Mat bgr_frame(height, width, CV_8UC3, frame_bgr_->data[0], 
                      frame_bgr_->linesize[0]);
    
    // 将帧放入显示队列
    {
      std::lock_guard<std::mutex> lock(frame_mutex_);
      if (frame_queue_.size() > 5) {
        frame_queue_.pop();  // 丢弃旧帧，防止延迟累积
      }
      frame_queue_.push(bgr_frame.clone());
    }
    
    frame_count_++;
    
    // 发布ROS图像消息（可选）
    if (publish_image_ && image_pub_) {
      try {
        std_msgs::msg::Header header;
        header.stamp = this->now();
        header.frame_id = "camera_frame";
        
        cv_bridge::CvImage cv_image(header, "bgr8", bgr_frame);
        image_pub_->publish(*cv_image.toImageMsg());
      } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "发布图像失败: %s", e.what());
      }
    }
  }
  
  /**
   * @brief WebSocket接收线程
   */
  void websocketThread()
  {
    RCLCPP_INFO(this->get_logger(), "🔌 启动WebSocket连接线程...");
    
    while (running_) {
      try {
        // 创建IO上下文
        net::io_context ioc;
        
        // 解析地址
        tcp::resolver resolver(ioc);
        auto const results = resolver.resolve(server_ip_, std::to_string(video_port_));
        
        // 创建WebSocket流
        websocket::stream<tcp::socket> ws(ioc);
        
        // 连接到服务器
        RCLCPP_INFO(this->get_logger(), "🔌 正在连接到 ws://%s:%d%s...",
                    server_ip_.c_str(), video_port_, ws_path_.c_str());
        
        net::connect(ws.next_layer(), results);
        
        // 设置Host头
        std::string host = server_ip_ + ":" + std::to_string(video_port_);
        
        // WebSocket握手
        ws.handshake(host, ws_path_);
        
        RCLCPP_INFO(this->get_logger(), "✅ WebSocket连接已建立");
        std::cerr << "[INFO] ✅ WebSocket连接已建立" << std::endl;
        
        // 设置为二进制模式
        ws.binary(true);
        
        // 接收循环
        beast::flat_buffer buffer;
        
        while (running_) {
          buffer.clear();
          
          // 读取数据
          beast::error_code ec;
          ws.read(buffer, ec);
          
          if (ec == websocket::error::closed) {
            RCLCPP_WARN(this->get_logger(), "🔌 WebSocket连接已关闭");
            break;
          }
          
          if (ec) {
            RCLCPP_ERROR(this->get_logger(), "❌ WebSocket读取错误: %s", ec.message().c_str());
            break;
          }
          
          // 处理H.264数据
          auto data = buffer.data();
          const uint8_t* ptr = static_cast<const uint8_t*>(data.data());
          size_t len = data.size();
          
          if (len > 0) {
            onH264DataReceived(ptr, len);
          }
        }
        
        // 优雅关闭
        if (running_) {
          beast::error_code ec;
          ws.close(websocket::close_code::normal, ec);
        }
        
      } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "❌ WebSocket异常: %s", e.what());
        std::cerr << "[ERROR] ❌ WebSocket异常: " << e.what() << std::endl;
        
        // 重连延迟
        if (running_) {
          RCLCPP_INFO(this->get_logger(), "⏳ 3秒后重试连接...");
          std::this_thread::sleep_for(std::chrono::seconds(3));
        }
      }
    }
    
    RCLCPP_INFO(this->get_logger(), "🔌 WebSocket连接线程已退出");
  }
  
  /**
   * @brief 显示线程
   */
  void displayThread()
  {
    RCLCPP_INFO(this->get_logger(), "🎥 启动视频显示线程...");
    
    cv::namedWindow(video_window_name_, cv::WINDOW_AUTOSIZE);
    
    auto last_log_time = std::chrono::steady_clock::now();
    int displayed_frames = 0;
    
    while (running_) {
      cv::Mat frame;
      
      {
        std::lock_guard<std::mutex> lock(frame_mutex_);
        if (!frame_queue_.empty()) {
          frame = frame_queue_.front();
          frame_queue_.pop();
        }
      }
      
      if (!frame.empty()) {
        cv::imshow(video_window_name_, frame);
        displayed_frames++;
      }
      
      // 按键检测（必须在显示线程中调用）
      int key = cv::waitKey(1);
      if (key == 27 || key == 'q') {  // ESC或q退出
        running_ = false;
        break;
      }
      
      // 每秒打印统计信息
      auto now = std::chrono::steady_clock::now();
      if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time).count() >= 1) {
        double fps = displayed_frames;
        std::cerr << "[INFO] 📹 FPS: " << fps 
                  << ", 总帧数: " << frame_count_
                  << ", 接收: " << (total_bytes_received_ / 1024.0) << " KB" << std::endl;
        
        displayed_frames = 0;
        last_log_time = now;
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    
    cv::destroyWindow(video_window_name_);
    RCLCPP_INFO(this->get_logger(), "🎥 视频显示线程已退出");
  }
  
  // ROS发布者
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  
  // 参数
  std::string server_ip_;
  int video_port_;
  std::string ws_path_;
  std::string video_window_name_;
  bool display_video_;
  std::string image_topic_;
  bool publish_image_;
  
  // H.264解码器
  AVCodecContext* codec_ctx_ = nullptr;
  AVCodecParserContext* parser_ = nullptr;
  AVFrame* frame_ = nullptr;
  AVFrame* frame_bgr_ = nullptr;
  SwsContext* sws_ctx_ = nullptr;
  int last_width_ = 0;
  int last_height_ = 0;
  
  // H.264缓冲区
  std::vector<uint8_t> h264_buffer_;
  std::mutex h264_mutex_;
  
  // 显示帧队列
  std::queue<cv::Mat> frame_queue_;
  std::mutex frame_mutex_;
  
  // 统计
  std::atomic<int> frame_count_{0};
  std::atomic<size_t> total_bytes_received_{0};
  
  // 线程控制
  std::atomic<bool> running_{false};
  std::thread ws_thread_;
  std::thread display_thread_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<WebSocketVideoNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("websocket_video"), "异常: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}
