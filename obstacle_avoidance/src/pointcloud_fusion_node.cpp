#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "obstacle_avoidance/msg/ellipse_obstacle.hpp"
#include "obstacle_avoidance/msg/ellipse_obstacle_array.hpp"
#include "obstacle_avoidance/msg/transform_matrix.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <Eigen/Dense>

using namespace std::chrono_literals;

/**
 * @brief 障碍物融合节点
 * 
 * 功能：
 * 1. 订阅两个障碍物话题和ICP变换矩阵
 * 2. 将障碍物2的信息通过变换矩阵转换到障碍物1的坐标系
 * 3. 合并两组障碍物
 * 4. 将合并后的障碍物转换到camera_link坐标系（与话题1坐标系相对位置固定）
 */
class ObstacleFusionNode : public rclcpp::Node
{
public:
  ObstacleFusionNode()
  : Node("obstacle_fusion_node")
  {
    // 声明参数 - 输入话题
    this->declare_parameter<std::string>("obstacle_topic1", "/obstacles_dog");
    this->declare_parameter<std::string>("obstacle_topic2", "/obstacles_person");
    this->declare_parameter<std::string>("transform_topic", "/icp_transform");
    
    // 声明参数 - 输出话题
    this->declare_parameter<std::string>("output_obstacle_topic", "/obstacles_fused");
    this->declare_parameter<std::string>("output_marker_topic", "/obstacles_markers_fused");
    this->declare_parameter<std::string>("output_marker_topic_dog", "/obstacles_markers_dog_frame");
    this->declare_parameter<std::string>("target_frame", "camera_link");
    this->declare_parameter<bool>("is_optical_frame", true);  // 是否是光学坐标系
    
    // 声明参数 - 话题1坐标系到camera_link的固定变换
    // 这个变换描述了camera_dog_optical_frame到camera_link的相对位置
    this->declare_parameter<double>("frame1_to_camera_link_x", 0.0);
    this->declare_parameter<double>("frame1_to_camera_link_y", 0.0);
    this->declare_parameter<double>("frame1_to_camera_link_z", 0.0);
    this->declare_parameter<double>("frame1_to_camera_link_roll", 0.0);
    this->declare_parameter<double>("frame1_to_camera_link_pitch", 0.0);
    this->declare_parameter<double>("frame1_to_camera_link_yaw", 0.0);
    
    // 声明参数 - 地面对齐
    // 融合后将所有障碍物底面对齐到同一平面
    this->declare_parameter<bool>("align_to_ground", true);
    this->declare_parameter<double>("ground_plane_height", 0.0);  // 地面高度（在目标坐标系中）
    this->declare_parameter<bool>("use_auto_ground", true);       // 自动检测地面（使用最低障碍物底面）
    
    // 获取参数
    obstacle_topic1_ = this->get_parameter("obstacle_topic1").as_string();
    obstacle_topic2_ = this->get_parameter("obstacle_topic2").as_string();
    transform_topic_ = this->get_parameter("transform_topic").as_string();
    output_obstacle_topic_ = this->get_parameter("output_obstacle_topic").as_string();
    output_marker_topic_ = this->get_parameter("output_marker_topic").as_string();
    output_marker_topic_dog_ = this->get_parameter("output_marker_topic_dog").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    is_optical_frame_ = this->get_parameter("is_optical_frame").as_bool();
    
    align_to_ground_ = this->get_parameter("align_to_ground").as_bool();
    ground_plane_height_ = this->get_parameter("ground_plane_height").as_double();
    use_auto_ground_ = this->get_parameter("use_auto_ground").as_bool();
    
    // 构建frame1到camera_link的固定变换矩阵
    double tx = this->get_parameter("frame1_to_camera_link_x").as_double();
    double ty = this->get_parameter("frame1_to_camera_link_y").as_double();
    double tz = this->get_parameter("frame1_to_camera_link_z").as_double();
    
    // 直接构建optical_frame到camera_link的变换矩阵
    // optical_frame: X=右, Y=下, Z=前
    // camera_link:   X=前, Y=左, Z=上
    // 
    // 变换关系:
    //   camera_link.X = optical.Z  (前 = 前)
    //   camera_link.Y = -optical.X (左 = -右)  
    //   camera_link.Z = -optical.Y (上 = -下)
    //
    // 旋转矩阵 R，使得 camera_link = R * optical:
    //   [X_cl]   [ 0   0   1] [X_opt]
    //   [Y_cl] = [-1   0   0] [Y_opt]
    //   [Z_cl]   [ 0  -1   0] [Z_opt]
    
    frame1_to_camera_link_ = Eigen::Matrix4f::Identity();
    
    // 第一行: camera_link.X = optical.Z
    frame1_to_camera_link_(0, 0) = 0.0f;
    frame1_to_camera_link_(0, 1) = 0.0f;
    frame1_to_camera_link_(0, 2) = 1.0f;
    
    // 第二行: camera_link.Y = -optical.X
    frame1_to_camera_link_(1, 0) = -1.0f;
    frame1_to_camera_link_(1, 1) = 0.0f;
    frame1_to_camera_link_(1, 2) = 0.0f;
    
    // 第三行: camera_link.Z = -optical.Y
    frame1_to_camera_link_(2, 0) = 0.0f;
    frame1_to_camera_link_(2, 1) = -1.0f;
    frame1_to_camera_link_(2, 2) = 0.0f;
    
    // 平移部分
    frame1_to_camera_link_(0, 3) = tx;
    frame1_to_camera_link_(1, 3) = ty;
    frame1_to_camera_link_(2, 3) = tz;
    
    RCLCPP_INFO(this->get_logger(), "optical->camera_link 变换矩阵:");
    RCLCPP_INFO(this->get_logger(), "  [%.1f, %.1f, %.1f, %.1f]", 
                frame1_to_camera_link_(0,0), frame1_to_camera_link_(0,1),
                frame1_to_camera_link_(0,2), frame1_to_camera_link_(0,3));
    RCLCPP_INFO(this->get_logger(), "  [%.1f, %.1f, %.1f, %.1f]",
                frame1_to_camera_link_(1,0), frame1_to_camera_link_(1,1),
                frame1_to_camera_link_(1,2), frame1_to_camera_link_(1,3));
    RCLCPP_INFO(this->get_logger(), "  [%.1f, %.1f, %.1f, %.1f]",
                frame1_to_camera_link_(2,0), frame1_to_camera_link_(2,1),
                frame1_to_camera_link_(2,2), frame1_to_camera_link_(2,3));
    
    // 创建订阅者
    sub_obstacles1_ = this->create_subscription<obstacle_avoidance::msg::EllipseObstacleArray>(
      obstacle_topic1_, 10,
      std::bind(&ObstacleFusionNode::obstacles1Callback, this, std::placeholders::_1));
    
    sub_obstacles2_ = this->create_subscription<obstacle_avoidance::msg::EllipseObstacleArray>(
      obstacle_topic2_, 10,
      std::bind(&ObstacleFusionNode::obstacles2Callback, this, std::placeholders::_1));
    
    sub_transform_ = this->create_subscription<obstacle_avoidance::msg::TransformMatrix>(
      transform_topic_, 10,
      std::bind(&ObstacleFusionNode::transformCallback, this, std::placeholders::_1));
    
    // 创建发布者
    obstacle_pub_ = this->create_publisher<obstacle_avoidance::msg::EllipseObstacleArray>(
      output_obstacle_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      output_marker_topic_, 10);
    marker_pub_dog_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      output_marker_topic_dog_, 10);
    
    // 创建定时器进行融合
    timer_ = this->create_wall_timer(
      100ms, std::bind(&ObstacleFusionNode::fusionTimerCallback, this));
    
    // 初始化变换矩阵
    icp_transform_ = Eigen::Matrix4f::Identity();
    
    RCLCPP_INFO(this->get_logger(), "障碍物融合节点已启动");
    RCLCPP_INFO(this->get_logger(), "  订阅话题1: %s", obstacle_topic1_.c_str());
    RCLCPP_INFO(this->get_logger(), "  订阅话题2: %s", obstacle_topic2_.c_str());
    RCLCPP_INFO(this->get_logger(), "  变换矩阵话题: %s", transform_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "  输出话题: %s (坐标系: %s)", 
                output_obstacle_topic_.c_str(), target_frame_.c_str());
  }

private:
  /**
   * @brief 障碍物1回调（来自topic1坐标系）
   */
  void obstacles1Callback(const obstacle_avoidance::msg::EllipseObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles1_ = msg;
    received_obstacles1_ = true;
    frame1_id_ = msg->header.frame_id;
    RCLCPP_DEBUG(this->get_logger(), "接收到障碍物1: %zu 个", msg->obstacles.size());
  }
  
  /**
   * @brief 障碍物2回调（来自topic2坐标系）
   */
  void obstacles2Callback(const obstacle_avoidance::msg::EllipseObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles2_ = msg;
    received_obstacles2_ = true;
    frame2_id_ = msg->header.frame_id;
    RCLCPP_DEBUG(this->get_logger(), "接收到障碍物2: %zu 个", msg->obstacles.size());
  }
  
  /**
   * @brief 变换矩阵回调
   */
  void transformCallback(const obstacle_avoidance::msg::TransformMatrix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 从消息中提取4x4变换矩阵
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        icp_transform_(i, j) = msg->matrix[i * 4 + j];
      }
    }
    
    received_transform_ = true;
    transform_converged_ = msg->converged;
    transform_fitness_ = msg->fitness_score;
    
    RCLCPP_DEBUG(this->get_logger(), "接收到ICP变换矩阵 (收敛: %s, 适配度: %.4f)",
                 transform_converged_ ? "是" : "否", transform_fitness_);
  }
  
  /**
   * @brief 融合定时器回调
   */
  void fusionTimerCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查数据是否完整
    if (!received_obstacles1_ || !received_obstacles2_ || !received_transform_) {
      if (!received_obstacles1_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "等待障碍物1数据...");
      }
      if (!received_obstacles2_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "等待障碍物2数据...");
      }
      if (!received_transform_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "等待ICP变换矩阵...");
      }
      return;
    }
    
    try {
      // ========== 步骤1: 在dog坐标系(frame1)下融合障碍物 ==========
      std::vector<obstacle_avoidance::msg::EllipseObstacle> fused_in_dog_frame;
      
      // 1.1 将障碍物1直接添加（已在frame1坐标系）
      int id = 0;
      for (const auto& obs : obstacles1_->obstacles) {
        auto obs_copy = obs;
        obs_copy.id = id++;
        fused_in_dog_frame.push_back(obs_copy);
      }
      
      // 1.2 将障碍物2通过ICP变换矩阵转换到frame1坐标系
      for (const auto& obs : obstacles2_->obstacles) {
        auto transformed_obs = transformObstacle(obs, icp_transform_);
        transformed_obs.id = id++;
        fused_in_dog_frame.push_back(transformed_obs);
      }
      
      // ========== 步骤2: 地面对齐 - 将所有障碍物底面对齐到同一平面 ==========
      if (align_to_ground_ && !fused_in_dog_frame.empty()) {
        alignObstaclesToGround(fused_in_dog_frame);
      }
      
      // ========== 步骤3: 发布dog坐标系下的融合障碍物可视化 ==========
      publishMarkersInFrame(fused_in_dog_frame, frame1_id_, marker_pub_dog_);
      
      // ========== 步骤4: 转换到camera_link坐标系 ==========
      std::vector<obstacle_avoidance::msg::EllipseObstacle> final_obstacles;
      for (const auto& obs : fused_in_dog_frame) {
        auto final_obs = transformObstacle(obs, frame1_to_camera_link_);
        final_obstacles.push_back(final_obs);
      }
      
      // ========== 步骤4.5: 投影到XY平面并调整椭圆方向 ==========
      // camera_link坐标系：X=前, Y=左, Z=上
      // 对于2D导航，将障碍物投影到Z=0平面（地面）
      for (auto& obs : final_obstacles) {
        // 将障碍物投影到XY平面，底部在Z=0
        obs.center.z = obs.height / 2.0;
        obs.z_min = 0.0;
        obs.z_max = obs.height;
        
        // 交换长短轴：从optical frame到camera_link时坐标轴映射发生变化
        // optical: 长轴可能沿Z（前后），短轴沿X（左右）
        // camera_link: 长轴应沿X（前后），短轴沿Y（左右）
        // 但由于CYLINDER标记的scale.x/y定义方式，需要交换
        std::swap(obs.semi_major_axis, obs.semi_minor_axis);
      }
      
      // 打印第一个障碍物的详细信息（用于调试）
      if (!final_obstacles.empty()) {
        const auto& first = final_obstacles[0];
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                    "示例障碍物[0] camera_link坐标: (x=%.2f前, y=%.2f左, z=%.2f高), 尺寸=%.2fx%.2fx%.2f",
                    first.center.x, first.center.y, first.center.z,
                    first.semi_major_axis * 2, first.semi_minor_axis * 2, first.height);
      }
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                  "障碍物融合: %zu (dog) + %zu (person) = %zu 个 -> camera_link (地面对齐: %s)",
                  obstacles1_->obstacles.size(), 
                  obstacles2_->obstacles.size(),
                  final_obstacles.size(),
                  align_to_ground_ ? "是" : "否");
      
      // ========== 步骤5: 发布camera_link坐标系下的障碍物 ==========
      publishObstacles(final_obstacles);
      
      // ========== 步骤6: 发布camera_link坐标系下的可视化标记 ==========
      publishMarkersInFrame(final_obstacles, target_frame_, marker_pub_);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "障碍物融合失败: %s", e.what());
    }
  }
  
  /**
   * @brief 对单个障碍物应用变换
   */
  obstacle_avoidance::msg::EllipseObstacle transformObstacle(
    const obstacle_avoidance::msg::EllipseObstacle& obs,
    const Eigen::Matrix4f& transform)
  {
    obstacle_avoidance::msg::EllipseObstacle transformed;
    transformed = obs;  // 复制原始数据
    
    // 变换中心点
    Eigen::Vector4f center(obs.center.x, obs.center.y, obs.center.z, 1.0f);
    Eigen::Vector4f transformed_center = transform * center;
    transformed.center.x = transformed_center(0);
    transformed.center.y = transformed_center(1);
    transformed.center.z = transformed_center(2);
    
    // 变换z_min和z_max（考虑高度方向）
    Eigen::Vector4f z_min_pt(obs.center.x, obs.center.y, obs.z_min, 1.0f);
    Eigen::Vector4f z_max_pt(obs.center.x, obs.center.y, obs.z_max, 1.0f);
    Eigen::Vector4f transformed_z_min = transform * z_min_pt;
    Eigen::Vector4f transformed_z_max = transform * z_max_pt;
    transformed.z_min = transformed_z_min(2);
    transformed.z_max = transformed_z_max(2);
    
    // 提取旋转部分来变换椭圆方向
    Eigen::Matrix3f rotation = transform.block<3,3>(0,0);
    
    // 变换旋转角度（只考虑绕Z轴的旋转）
    // 从旋转矩阵中提取yaw角
    float yaw = std::atan2(rotation(1,0), rotation(0,0));
    transformed.rotation_angle = obs.rotation_angle + yaw;
    
    // 规范化角度到 [-π, π]
    while (transformed.rotation_angle > M_PI) transformed.rotation_angle -= 2.0 * M_PI;
    while (transformed.rotation_angle < -M_PI) transformed.rotation_angle += 2.0 * M_PI;
    
    // 重新计算距离（到新坐标系原点的距离）
    transformed.distance = std::sqrt(
      transformed.center.x * transformed.center.x + 
      transformed.center.y * transformed.center.y);
    
    return transformed;
  }
  
  /**
   * @brief 将所有障碍物底面对齐到同一平面
   * 
   * 在光学坐标系中：Y轴向下，所以障碍物底面是 center.y + height/2
   * 在标准坐标系中：Z轴向上，所以障碍物底面是 center.z - height/2
   */
  void alignObstaclesToGround(std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles)
  {
    if (obstacles.empty()) return;
    
    double ground_y;  // 目标地面高度
    
    if (use_auto_ground_) {
      // 自动模式：找到所有障碍物中最低的底面作为地面
      if (is_optical_frame_) {
        // 光学坐标系：Y向下，底面 = center.y + height/2，最大Y是最低点
        ground_y = -std::numeric_limits<double>::infinity();
        for (const auto& obs : obstacles) {
          double bottom_y = obs.center.y + obs.height / 2.0;
          if (bottom_y > ground_y) {
            ground_y = bottom_y;
          }
        }
      } else {
        // 标准坐标系：Z向上，底面 = center.z - height/2，最小Z是最低点
        ground_y = std::numeric_limits<double>::infinity();
        for (const auto& obs : obstacles) {
          double bottom_z = obs.center.z - obs.height / 2.0;
          if (bottom_z < ground_y) {
            ground_y = bottom_z;
          }
        }
      }
      
      RCLCPP_DEBUG(this->get_logger(), "自动检测地面高度: %.3f", ground_y);
    } else {
      // 手动模式：使用参数指定的地面高度
      ground_y = ground_plane_height_;
    }
    
    // 调整每个障碍物的位置，使其底面对齐到地面
    for (auto& obs : obstacles) {
      if (is_optical_frame_) {
        // 光学坐标系：调整center.y使得 center.y + height/2 = ground_y
        double current_bottom = obs.center.y + obs.height / 2.0;
        double adjustment = ground_y - current_bottom;
        obs.center.y += adjustment;
        
        // 同步更新z_min和z_max（如果它们存储的是Y方向的范围）
        // 注意：在某些实现中z_min/z_max可能指的是前后方向(Z)，这里假设是高度方向
        if (std::abs(obs.z_max - obs.z_min - obs.height) < 0.01) {
          // z_min/z_max看起来是高度范围
          obs.z_min += adjustment;
          obs.z_max += adjustment;
        }
      } else {
        // 标准坐标系：调整center.z使得 center.z - height/2 = ground_y
        double current_bottom = obs.center.z - obs.height / 2.0;
        double adjustment = ground_y - current_bottom;
        obs.center.z += adjustment;
        obs.z_min += adjustment;
        obs.z_max += adjustment;
      }
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                "障碍物底面已对齐到地面 (高度=%.3f, 模式=%s)", 
                ground_y, use_auto_ground_ ? "自动" : "手动");
  }
  
  /**
   * @brief 发布融合后的障碍物
   */
  void publishObstacles(const std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles)
  {
    obstacle_avoidance::msg::EllipseObstacleArray msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = target_frame_;
    msg.obstacles = obstacles;
    obstacle_pub_->publish(msg);
  }
  
  /**
   * @brief 发布可视化标记到指定坐标系
   * @param obstacles 障碍物列表
   * @param frame_id 坐标系ID
   * @param publisher 发布者
   */
  void publishMarkersInFrame(
    const std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles,
    const std::string& frame_id,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    auto timestamp = this->now();
    
    // 先删除所有旧标记
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = timestamp;
    delete_marker.ns = "fused_obstacles";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // 区分dog端和person端障碍物的数量（用于颜色区分）
    size_t dog_count = obstacles1_ ? obstacles1_->obstacles.size() : 0;
    
    // 为每个障碍物创建标记
    for (const auto& obs : obstacles) {
      // 椭圆柱体标记（垂直于地面）
      visualization_msgs::msg::Marker cylinder_marker;
      cylinder_marker.header.frame_id = frame_id;
      cylinder_marker.header.stamp = timestamp;
      cylinder_marker.ns = "fused_obstacles";
      cylinder_marker.id = obs.id;
      cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      cylinder_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 位置：质心坐标
      cylinder_marker.pose.position.x = obs.center.x;
      cylinder_marker.pose.position.y = obs.center.y;
      cylinder_marker.pose.position.z = obs.center.z;
      
      // 旋转：使圆柱体垂直于地面
      // 根据目标坐标系决定旋转方式：
      // - 光学坐标系（如frame1_id_）：Y轴向下，需要绕X轴旋转90度
      // - 标准坐标系（如camera_link）：Z轴向上，无需旋转
      bool is_optical = (frame_id == frame1_id_);  // 判断是否发布到光学坐标系
      if (is_optical) {
        // 光学坐标系：圆柱体默认沿Z轴，需绕X轴旋转90°使其沿Y轴（垂直方向）
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        cylinder_marker.pose.orientation.x = q.x();
        cylinder_marker.pose.orientation.y = q.y();
        cylinder_marker.pose.orientation.z = q.z();
        cylinder_marker.pose.orientation.w = q.w();
      } else {
        // 标准坐标系（camera_link）：Z轴向上，圆柱体默认方向正确，无需旋转
        cylinder_marker.pose.orientation.x = 0.0;
        cylinder_marker.pose.orientation.y = 0.0;
        cylinder_marker.pose.orientation.z = 0.0;
        cylinder_marker.pose.orientation.w = 1.0;
      }
      
      // 尺寸
      cylinder_marker.scale.x = std::max(0.1, static_cast<double>(obs.semi_major_axis * 2.0));
      cylinder_marker.scale.y = std::max(0.1, static_cast<double>(obs.semi_minor_axis * 2.0));
      cylinder_marker.scale.z = std::max(0.1, static_cast<double>(obs.height));
      
      // 根据来源区分颜色（前dog_count个来自dog，其余来自person）
      if (static_cast<size_t>(obs.id) < dog_count) {
        // 来自dog - 蓝色
        cylinder_marker.color.r = 0.2;
        cylinder_marker.color.g = 0.5;
        cylinder_marker.color.b = 1.0;
      } else {
        // 来自person - 绿色
        cylinder_marker.color.r = 0.2;
        cylinder_marker.color.g = 1.0;
        cylinder_marker.color.b = 0.5;
      }
      cylinder_marker.color.a = 0.7;
      
      cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      marker_array.markers.push_back(cylinder_marker);
      
      // 文本标记
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;
      text_marker.header.stamp = timestamp;
      text_marker.ns = "fused_text";
      text_marker.id = obs.id + 1000;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 文本位置：在障碍物上方
      text_marker.pose.position.x = obs.center.x;
      text_marker.pose.position.y = obs.center.y - 0.3;  // 光学坐标系上方是-Y
      text_marker.pose.position.z = obs.center.z;
      text_marker.pose.orientation.w = 1.0;
      
      text_marker.scale.z = 0.15;
      
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      
      char text[64];
      snprintf(text, sizeof(text), "%d:%.1fm", obs.id, obs.distance);
      text_marker.text = text;
      
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.3);
      marker_array.markers.push_back(text_marker);
    }
    
    publisher->publish(marker_array);
  }

  // 订阅者
  rclcpp::Subscription<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr sub_obstacles1_;
  rclcpp::Subscription<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr sub_obstacles2_;
  rclcpp::Subscription<obstacle_avoidance::msg::TransformMatrix>::SharedPtr sub_transform_;
  
  // 发布者
  rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_dog_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 数据缓存
  obstacle_avoidance::msg::EllipseObstacleArray::SharedPtr obstacles1_;
  obstacle_avoidance::msg::EllipseObstacleArray::SharedPtr obstacles2_;
  
  // 状态标志
  bool received_obstacles1_ = false;
  bool received_obstacles2_ = false;
  bool received_transform_ = false;
  bool transform_converged_ = false;
  float transform_fitness_ = 0.0f;
  
  // 参数
  std::string obstacle_topic1_;
  std::string obstacle_topic2_;
  std::string transform_topic_;
  std::string output_obstacle_topic_;
  std::string output_marker_topic_;
  std::string output_marker_topic_dog_;
  std::string target_frame_;
  std::string frame1_id_;
  std::string frame2_id_;
  bool is_optical_frame_;
  
  // 地面对齐参数
  bool align_to_ground_;
  double ground_plane_height_;
  bool use_auto_ground_;
  
  // 变换矩阵
  Eigen::Matrix4f icp_transform_;           // ICP计算的topic2到topic1的变换
  Eigen::Matrix4f frame1_to_camera_link_;   // topic1坐标系到camera_link的固定变换
  
  // 互斥锁
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
