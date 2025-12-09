#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "obstacle_avoidance/msg/ellipse_obstacle_array.hpp"
#include <Eigen/Dense>

using namespace std::chrono_literals;

/**
 * @brief 人工势场法（APF）避障控制器节点
 * 
 * 实现原理：
 * 1. 目标位置产生引力：F_att = k_att * (goal - robot_pos)
 * 2. 障碍物产生斥力：F_rep = k_rep * (1/d - 1/d0) * 1/d^2 * direction
 * 3. 合力决定运动方向：F_total = F_att + F_rep
 */
class APFControllerNode : public rclcpp::Node
{
public:
  APFControllerNode()
  : Node("apf_controller_node"),
    has_goal_(false),
    has_obstacles_(false)
  {
    // 声明参数
    // APF算法参数
    this->declare_parameter<double>("k_att", 1.0);              // 引力增益系数
    this->declare_parameter<double>("k_rep", 5.0);              // 斥力增益系数（增强避障）
    this->declare_parameter<double>("d0", 3.0);                 // 障碍物影响距离（米，增加提前避障距离）
    this->declare_parameter<double>("goal_threshold", 0.2);     // 到达目标的距离阈值（米）
    this->declare_parameter<double>("min_safe_distance", 0.3);  // 最小安全距离（米）
    
    // 机器人模型参数
    this->declare_parameter<double>("robot_radius", 0.5);       // 机器人半径（米）
    this->declare_parameter<double>("max_velocity", 1.0);       // 最大线速度（米/秒）
    this->declare_parameter<double>("max_angular_vel", 1.0);    // 最大角速度（弧度/秒）
    this->declare_parameter<double>("min_velocity", 0.1);       // 最小线速度（米/秒）
    this->declare_parameter<double>("max_reverse_velocity", 0.5); // 最大倒退速度（米/秒）
    this->declare_parameter<bool>("enable_reverse", true);      // 是否允许倒退
    
    // 控制参数
    this->declare_parameter<double>("control_frequency", 10.0); // 控制频率（Hz）
    this->declare_parameter<double>("velocity_scale", 0.5);     // 速度缩放因子
    
    // 话题名称参数
    this->declare_parameter<std::string>("goal_topic", "/goal_pose");
    this->declare_parameter<std::string>("obstacles_topic", "/obstacles");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    
    // 获取参数
    k_att_ = this->get_parameter("k_att").as_double();
    k_rep_ = this->get_parameter("k_rep").as_double();
    d0_ = this->get_parameter("d0").as_double();
    goal_threshold_ = this->get_parameter("goal_threshold").as_double();
    min_safe_distance_ = this->get_parameter("min_safe_distance").as_double();
    
    robot_radius_ = this->get_parameter("robot_radius").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    min_velocity_ = this->get_parameter("min_velocity").as_double();
    max_reverse_velocity_ = this->get_parameter("max_reverse_velocity").as_double();
    enable_reverse_ = this->get_parameter("enable_reverse").as_bool();
    
    double control_frequency = this->get_parameter("control_frequency").as_double();
    velocity_scale_ = this->get_parameter("velocity_scale").as_double();
    
    std::string goal_topic = this->get_parameter("goal_topic").as_string();
    std::string obstacles_topic = this->get_parameter("obstacles_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    
    // 创建订阅者
    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      std::bind(&APFControllerNode::goalCallback, this, std::placeholders::_1));
    
    obstacles_sub_ = this->create_subscription<obstacle_avoidance::msg::EllipseObstacleArray>(
      obstacles_topic, 10,
      std::bind(&APFControllerNode::obstaclesCallback, this, std::placeholders::_1));
    
    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    
    // 创建定时器
    auto timer_period = std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency));
    control_timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&APFControllerNode::controlLoop, this));
    
    RCLCPP_INFO(this->get_logger(), "人工势场法控制器节点已启动");
    RCLCPP_INFO(this->get_logger(), "  引力增益: %.2f", k_att_);
    RCLCPP_INFO(this->get_logger(), "  斥力增益: %.2f (增强)", k_rep_);
    RCLCPP_INFO(this->get_logger(), "  影响距离: %.2f m", d0_);
    RCLCPP_INFO(this->get_logger(), "  最小安全距离: %.2f m", min_safe_distance_);
    RCLCPP_INFO(this->get_logger(), "  机器人半径: %.2f m", robot_radius_);
    RCLCPP_INFO(this->get_logger(), "  最大速度: %.2f m/s (允许倒退: %s)", 
                max_velocity_, enable_reverse_ ? "是" : "否");
    RCLCPP_INFO(this->get_logger(), "  控制频率: %.2f Hz", control_frequency);
  }

private:
  /**
   * @brief 目标位置回调函数
   */
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    goal_pose_ = *msg;
    has_goal_ = true;
    
    RCLCPP_INFO(this->get_logger(), "收到新目标: (%.2f, %.2f)",
                goal_pose_.pose.position.x, goal_pose_.pose.position.y);
  }
  
  /**
   * @brief 障碍物信息回调函数
   */
  void obstaclesCallback(const obstacle_avoidance::msg::EllipseObstacleArray::SharedPtr msg)
  {
    obstacles_ = *msg;
    has_obstacles_ = true;
  }
  
  /**
   * @brief 主控制循环
   */
  void controlLoop()
  {
    // 检查是否有目标
    if (!has_goal_) {
      publishZeroVelocity();
      return;
    }
    
    // 假设机器人始终在坐标原点（与MPC控制器一致）
    Eigen::Vector2d robot_pos(0.0, 0.0);
    Eigen::Vector2d goal_pos(goal_pose_.pose.position.x, goal_pose_.pose.position.y);
    
    // 检查是否到达目标
    double distance_to_goal = (goal_pos - robot_pos).norm();
    if (distance_to_goal < goal_threshold_) {
      RCLCPP_INFO(this->get_logger(), "已到达目标！");
      publishZeroVelocity();
      has_goal_ = false;  // 清除目标
      return;
    }
    
    // 检查最近障碍物距离
    double min_obstacle_dist = getMinObstacleDistance(robot_pos);
    
    // 计算引力
    Eigen::Vector2d f_att = computeAttractiveForce(robot_pos, goal_pos);
    
    // 计算斥力
    Eigen::Vector2d f_rep = computeRepulsiveForce(robot_pos);
    
    // 计算合力
    Eigen::Vector2d f_total = f_att + f_rep;
    
    // 转换为速度命令
    geometry_msgs::msg::Twist cmd_vel = forceToVelocity(f_total, robot_pos, goal_pos, min_obstacle_dist);
    
    // 发布速度命令
    cmd_vel_pub_->publish(cmd_vel);
    
    // 打印调试信息
    static int counter = 0;
    if (counter++ % 10 == 0) {  // 每秒打印一次（假设10Hz）
      std::string warning = (min_obstacle_dist < min_safe_distance_) ? " ⚠️危险！" : "";
      RCLCPP_INFO(this->get_logger(), 
                  "到目标: %.2f m | 最近障碍: %.2f m%s | 引力: (%.2f, %.2f) | 斥力: (%.2f, %.2f) | 速度: v=%.2f, w=%.2f",
                  distance_to_goal, min_obstacle_dist, warning.c_str(),
                  f_att.x(), f_att.y(), f_rep.x(), f_rep.y(),
                  cmd_vel.linear.x, cmd_vel.angular.z);
    }
  }
  
  /**
   * @brief 计算引力（吸引力）
   * F_att = k_att * (goal - robot_pos)
   */
  Eigen::Vector2d computeAttractiveForce(const Eigen::Vector2d& robot_pos,
                                         const Eigen::Vector2d& goal_pos)
  {
    Eigen::Vector2d direction = goal_pos - robot_pos;
    return k_att_ * direction;
  }
  
  /**
   * @brief 获取最近障碍物距离
   */
  double getMinObstacleDistance(const Eigen::Vector2d& robot_pos)
  {
    if (!has_obstacles_ || obstacles_.obstacles.empty()) {
      return std::numeric_limits<double>::max();
    }
    
    double min_dist = std::numeric_limits<double>::max();
    
    for (const auto& obstacle : obstacles_.obstacles) {
      Eigen::Vector2d obs_pos(obstacle.center.x, obstacle.center.y);
      double distance = (robot_pos - obs_pos).norm();
      
      // 考虑机器人半径和障碍物半径
      double effective_distance = distance - robot_radius_ - 
                                 std::max(obstacle.semi_major_axis, obstacle.semi_minor_axis);
      
      min_dist = std::min(min_dist, effective_distance);
    }
    
    return min_dist;
  }
  
  /**
   * @brief 计算斥力（排斥力）- 改进版，增强避障能力
   * F_rep = k_rep * (1/d - 1/d0) * 1/d^2 * direction
   * 
   * 改进：
   * 1. 增加紧急避障阈值
   * 2. 分段斥力函数，避免数值爆炸
   * 3. 增强近距离斥力
   */
  Eigen::Vector2d computeRepulsiveForce(const Eigen::Vector2d& robot_pos)
  {
    Eigen::Vector2d f_rep_total(0.0, 0.0);
    
    if (!has_obstacles_) {
      return f_rep_total;
    }
    
    // 遍历所有障碍物
    for (const auto& obstacle : obstacles_.obstacles) {
      Eigen::Vector2d obs_pos(obstacle.center.x, obstacle.center.y);
      Eigen::Vector2d diff = robot_pos - obs_pos;
      double distance = diff.norm();
      
      if (distance < 0.001) continue;  // 避免除零
      
      // 考虑机器人半径和障碍物半径（障碍物使用最大半径作为保守估计）
      double obstacle_radius = std::max(obstacle.semi_major_axis, obstacle.semi_minor_axis);
      double effective_distance = distance - robot_radius_ - obstacle_radius;
      
      // 计算斥力方向（从障碍物指向机器人）
      Eigen::Vector2d direction = diff / distance;
      
      // 分段计算斥力，避免数值不稳定
      double force_magnitude = 0.0;
      
      // 非常危险：距离 < 最小安全距离的一半
      if (effective_distance < min_safe_distance_ * 0.5) {
        // 极强斥力，指数增长
        force_magnitude = k_rep_ * 100.0 / std::max(effective_distance, 0.01);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "⚠️⚠️ 极度危险！距离障碍物仅 %.3f m", effective_distance);
      }
      // 危险：距离 < 最小安全距离
      else if (effective_distance < min_safe_distance_) {
        // 强斥力
        force_magnitude = k_rep_ * 10.0 / std::max(effective_distance, 0.05);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "⚠️ 危险！距离障碍物 %.3f m", effective_distance);
      }
      // 警告：距离 < 影响距离
      else if (effective_distance < d0_) {
        // 标准APF斥力公式
        double eta = 1.0 / effective_distance - 1.0 / d0_;
        force_magnitude = k_rep_ * eta / (effective_distance * effective_distance);
      }
      // 超出影响范围，不产生斥力
      else {
        continue;
      }
      
      // 累加斥力
      f_rep_total += force_magnitude * direction;
    }
    
    return f_rep_total;
  }
  
  /**
   * @brief 将合力转换为速度命令 - 改进版，支持倒退避障
   * 
   * 策略：
   * 1. 检查是否需要紧急避障（倒退）
   * 2. 线速度方向沿着合力方向
   * 3. 线速度大小与合力大小成正比，但限制在最大速度内
   * 4. 角速度用于调整机器人朝向，使其对准合力方向
   */
  geometry_msgs::msg::Twist forceToVelocity(const Eigen::Vector2d& force,
                                            const Eigen::Vector2d& robot_pos,
                                            const Eigen::Vector2d& goal_pos,
                                            double min_obstacle_dist)
  {
    geometry_msgs::msg::Twist cmd_vel;
    
    // 如果合力太小，停止
    double force_magnitude = force.norm();
    if (force_magnitude < 0.01) {
      return cmd_vel;  // 返回零速度
    }
    
    // 计算期望的运动方向（合力方向）
    double desired_theta = std::atan2(force.y(), force.x());
    
    // 假设机器人当前朝向为目标方向（简化处理）
    // 在实际应用中，应该从里程计获取当前朝向
    Eigen::Vector2d to_goal = goal_pos - robot_pos;
    double current_theta = std::atan2(to_goal.y(), to_goal.x());
    
    // 计算角度差
    double angle_diff = normalizeAngle(desired_theta - current_theta);
    
    // === 关键改进：检查是否需要倒退 ===
    bool should_reverse = false;
    
    // 条件1：障碍物非常近
    if (min_obstacle_dist < min_safe_distance_) {
      // 条件2：合力指向后方（与目标方向相反超过90度）
      if (std::abs(angle_diff) > M_PI / 2) {
        should_reverse = true;
      }
    }
    
    // 计算线速度
    double linear_velocity = velocity_scale_ * force_magnitude;
    
    // 紧急避障：倒退
    if (should_reverse && enable_reverse_) {
      // 倒退速度
      linear_velocity = -std::min(linear_velocity, max_reverse_velocity_);
      
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "🔙 紧急倒退避障！距离: %.2f m", min_obstacle_dist);
    }
    else {
      // 正常前进
      
      // 如果角度差较大，降低线速度，增加旋转
      if (std::abs(angle_diff) > M_PI / 4) {  // 45度
        linear_velocity *= 0.5;
      }
      
      // 如果障碍物很近，大幅降低速度
      if (min_obstacle_dist < min_safe_distance_ * 1.5) {
        double speed_ratio = min_obstacle_dist / (min_safe_distance_ * 1.5);
        linear_velocity *= std::max(0.2, speed_ratio);  // 至少保持20%速度以免卡住
      }
      
      // 限制线速度范围
      linear_velocity = std::clamp(linear_velocity, 0.0, max_velocity_);
      
      // 如果速度太小但还没到目标，给一个最小速度
      if (linear_velocity > 0.0 && linear_velocity < min_velocity_) {
        linear_velocity = min_velocity_;
      }
    }
    
    // 计算角速度：用于调整朝向
    double angular_velocity = 2.0 * angle_diff;  // 比例控制
    
    // 如果障碍物很近，增强转向能力
    if (min_obstacle_dist < min_safe_distance_ * 2.0) {
      angular_velocity *= 1.5;  // 增强转向
    }
    
    angular_velocity = std::clamp(angular_velocity, -max_angular_vel_, max_angular_vel_);
    
    cmd_vel.linear.x = linear_velocity;
    cmd_vel.angular.z = angular_velocity;
    
    return cmd_vel;
  }
  
  /**
   * @brief 发布零速度命令
   */
  void publishZeroVelocity()
  {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd_vel);
  }
  
  /**
   * @brief 归一化角度到 [-π, π]
   */
  double normalizeAngle(double angle)
  {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
  }
  
  // 订阅者
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr obstacles_sub_;
  
  // 发布者
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  // 数据存储
  geometry_msgs::msg::PoseStamped goal_pose_;
  obstacle_avoidance::msg::EllipseObstacleArray obstacles_;
  
  bool has_goal_;
  bool has_obstacles_;
  
  // APF参数
  double k_att_;              // 引力增益
  double k_rep_;              // 斥力增益
  double d0_;                 // 障碍物影响距离
  double goal_threshold_;     // 到达目标阈值
  double min_safe_distance_;  // 最小安全距离
  
  // 机器人参数
  double robot_radius_;
  double max_velocity_;
  double max_angular_vel_;
  double min_velocity_;
  double max_reverse_velocity_;  // 最大倒退速度
  bool enable_reverse_;          // 是否允许倒退
  
  // 控制参数
  double velocity_scale_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<APFControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

