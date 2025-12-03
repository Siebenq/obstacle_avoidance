#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "unitree_obstacle_avoidance/msg/ellipse_obstacle_array.hpp"

#include <casadi/casadi.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using namespace casadi;

class MPCControllerNode : public rclcpp::Node
{
public:
  MPCControllerNode()
  : Node("mpc_controller_node")
  {
    // 声明参数
    // MPC参数
    this->declare_parameter<int>("prediction_horizon", 10);      // 预测时域
    this->declare_parameter<double>("dt", 0.1);                  // 采样时间（秒）

    // 机器人模型参数
    this->declare_parameter<double>("robot_radius", 0.5);       // 机器人半径（米）

    this->declare_parameter<double>("max_velocity", 2.0);        // 最大速度（米/秒）
    this->declare_parameter<double>("min_velocity", 0.0);        // 最小速度（米/秒）
    this->declare_parameter<double>("max_angular_vel", 1.0);     // 最大角速度（弧度/秒）
    // this->declare_parameter<double>("max_acceleration", 1.0);    // 最大加速度
    // this->declare_parameter<double>("max_angular_acc", 0.5);     // 最大角加速度
    
    // 代价函数权重
    this->declare_parameter<double>("weight_position", 10.0);    // 位置跟踪权重
    this->declare_parameter<double>("weight_orientation", 5.0);   // 方向跟踪权重

    this->declare_parameter<double>("weight_velocity", 0.1);     // 速度平滑权重
    this->declare_parameter<double>("weight_angular_vel", 0.1);  // 角速度平滑权重

    this->declare_parameter<double>("weight_obstacle", 100.0);   // 障碍物避障权重

    // 安全参数
    this->declare_parameter<double>("safety_distance", 0.6);     // 安全距离（米）
    this->declare_parameter<double>("decay_rate", 4.0);           // 衰减率
    
    // 话题名称
    this->declare_parameter<std::string>("goal_pose_topic", "/goal_pose");
    this->declare_parameter<std::string>("obstacle_topic", "/obstacles");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("marker_topic", "/mpc_markers");
    // this->declare_parameter<std::string>("speed_topic", "/odom");
    



    // 获取参数
    N_ = this->get_parameter("prediction_horizon").as_int();
    dt_ = this->get_parameter("dt").as_double();

    robot_radius_ = this->get_parameter("robot_radius").as_double();

    max_velocity_ = this->get_parameter("max_velocity").as_double();
    min_velocity_ = this->get_parameter("min_velocity").as_double();
    max_angular_vel_ = this->get_parameter("max_angular_vel").as_double();
    // max_acceleration_ = this->get_parameter("max_acceleration").as_double();
    // max_angular_acc_ = this->get_parameter("max_angular_acc").as_double();
    
    weight_pos_ = this->get_parameter("weight_position").as_double();
    weight_ori_ = this->get_parameter("weight_orientation").as_double();

    weight_vel_ = this->get_parameter("weight_velocity").as_double();
    weight_ang_ = this->get_parameter("weight_angular_vel").as_double();

    weight_obs_ = this->get_parameter("weight_obstacle").as_double();

    safety_distance_ = this->get_parameter("safety_distance").as_double();
    decay_rate_ = this->get_parameter("decay_rate").as_double();

    std::string goal_topic = this->get_parameter("goal_pose_topic").as_string();
    std::string obs_topic = this->get_parameter("obstacle_topic").as_string();
    std::string cmd_topic = this->get_parameter("cmd_vel_topic").as_string();
    std::string marker_topic = this->get_parameter("marker_topic").as_string();
    // std::string speed_topic = this->get_parameter("speed_topic").as_string();
    




    // 初始化状态
    current_state_.resize(3);  // [x, y, theta]
    current_state_.setZero();
    
    // 初始化目标位姿（默认为原点）
    goal_pose_.position.x = 0.0;
    goal_pose_.position.y = 0.0;
    goal_pose_.position.z = 0.0;
    goal_pose_.orientation.w = 1.0;
    goal_pose_.orientation.x = 0.0;
    goal_pose_.orientation.y = 0.0;
    goal_pose_.orientation.z = 0.0;
    


    // 创建订阅者
    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      std::bind(&MPCControllerNode::goal_pose_callback, this, std::placeholders::_1));
    
    obstacle_sub_ = this->create_subscription<unitree_obstacle_avoidance::msg::EllipseObstacleArray>(
      obs_topic, 10,
      std::bind(&MPCControllerNode::obstacle_callback, this, std::placeholders::_1));
    
    // speed_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    //   speed_topic, 10,
    //   std::bind(&MPCControllerNode::speed_callback, this, std::placeholders::_1));

    
    // 创建发布者
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic, 10);
    


    // 初始化MPC求解器
    setupMPCSolver();
    
    // 创建控制定时器（10Hz）
    timer_ = this->create_wall_timer(
      100ms, std::bind(&MPCControllerNode::control_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "MPC控制器节点已启动");
  }



private:
  // 设置MPC求解器
  void setupMPCSolver()
  {
    RCLCPP_INFO(this->get_logger(), "初始化CasADi MPC求解器...");
    
    // 状态变量: [x, y, theta]
    SX x = SX::sym("x");
    SX y = SX::sym("y");
    SX theta = SX::sym("theta");
    SX state = vertcat(SXVector{x, y, theta});
    
    // 控制输入: [v, omega] (速度, 角速度)
    SX v = SX::sym("v");
    SX omega = SX::sym("omega");
    SX control = vertcat(SXVector{v, omega});
    
    // 差速小车运动学模型
    SX x_dot = v * cos(theta);
    SX y_dot = v * sin(theta);
    SX theta_dot = omega;
    SX state_dot = vertcat(SXVector{x_dot, y_dot, theta_dot});
    
    // 创建函数
    dynamics_func_ = Function("dynamics", {state, control}, {state_dot});
    
    RCLCPP_INFO(this->get_logger(), "MPC求解器初始化完成");
  }
  
  // 目标位姿回调
  void goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_pose_ = msg->pose;
    has_goal_ = true;
    
    RCLCPP_INFO(this->get_logger(), "接收到目标位姿: (%.2f, %.2f)", 
                goal_pose_.position.x, goal_pose_.position.y);
  }

  // void speed_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  //   std::lock_guard<std::mutex> lock(mutex_);
  //   current_state_[3]=msg->
  // }
  
  // 障碍物回调
  void obstacle_callback(const unitree_obstacle_avoidance::msg::EllipseObstacleArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = msg->obstacles;
    has_obstacles_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "接收到障碍物数据，数量: %zu", obstacles_.size());
    
    // 打印障碍物信息（调试用）
    if (!obstacles_.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "障碍物详情:");
      for (size_t i = 0; i < obstacles_.size() && i < 5; ++i) {
        const auto& obs = obstacles_[i];
        RCLCPP_DEBUG(this->get_logger(), 
                     "  [%zu] 位置:(%.2f, %.2f), 长轴:%.2f, 短轴:%.2f, 距离:%.2f",
                     i, obs.center.x, obs.center.y, 
                     obs.semi_major_axis, obs.semi_minor_axis, obs.distance);
      }
    }
  }
  
  // 控制定时器回调
  void control_timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_goal_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "等待接收目标位姿...");
      publishZeroVelocity();
      return;
    }
    
    // 求解MPC
    auto solution = solveMPC();
    
    if (solution.first) {
      // 发布控制命令
      publishControlCommand(solution.second);
      
      // 发布可视化
      publishVisualization();
    } else {
      RCLCPP_WARN(this->get_logger(), "MPC求解失败，发布零速度");
      publishZeroVelocity();
    }
  }
  
  // 求解MPC优化问题
  std::pair<bool, Eigen::Vector2d> solveMPC()
  {
    try {
      // 构建优化问题
      Opti opti;
      
      // 决策变量
      auto X = opti.variable(3, N_ + 1);  // 状态轨迹
      auto U = opti.variable(2, N_);      // 控制输入
      
      // 目标函数
      MX cost = 0;
      
      // 跟踪代价 + 障碍物代价
      for (int k = 0; k < N_; ++k) {
        // 获取参考点
        auto ref = getReferenceState();
        
        // 位置跟踪代价
        MX pos_error_x = X(0, k) - ref[0];
        MX pos_error_y = X(1, k) - ref[1];
        cost += weight_pos_ * (pos_error_x * pos_error_x + pos_error_y * pos_error_y);
        
        // 方向跟踪代价
        MX ori_error = X(2, k) - ref[2];
        cost += weight_ori_ * ori_error * ori_error;
        
        // 控制平滑代价
        cost += weight_vel_ * U(0, k) * U(0, k);
        cost += weight_ang_ * U(1, k) * U(1, k);
        
        // 障碍物避障代价
        if (has_obstacles_) {
          // 关键：传入第k步的预测位置 X(0,k)=预测的x, X(1,k)=预测的y
          // 即使当前在(0,0)，预测的k=5步可能是(0.5, 0.2)
          // 我们要检查"如果走到(0.5, 0.2)，会不会撞障碍物"
          cost += weight_obs_ * computeObstacleCost(X(0, k), X(1, k));
        }
      }

      // // 获取参考点
      // auto ref = getReferenceState();

      // // 位置跟踪代价
      // MX pos_error_x = X(0, N_) - ref[0];
      // MX pos_error_y = X(1, N_) - ref[1];
      // cost += weight_pos_ * (pos_error_x * pos_error_x + pos_error_y * pos_error_y);
              
      // // 方向跟踪代价
      // MX ori_error = X(2, N_) - ref[2];
      // cost += weight_ori_ * ori_error * ori_error;
      
      // 系统动力学约束
      for (int k = 0; k < N_; ++k) {
        MX state_k = X(Slice(), k);
        MX control_k = U(Slice(), k);
        
        // 使用欧拉前向积分
        std::vector<MX> dynamics_input = {state_k, control_k};
        std::vector<MX> dynamics_output = dynamics_func_(dynamics_input);
        MX state_dot = dynamics_output[0];
        
        MX state_next = state_k + dt_ * state_dot;
        opti.subject_to(X(Slice(), k + 1) == state_next);
      }
      
      // 初始状态约束
      opti.subject_to(X(Slice(), 0) == current_state_vector());
      
      // // 控制输入约束
      // opti.subject_to(opti.bounded(-max_acceleration_, U(0, Slice()), max_acceleration_));
      // opti.subject_to(opti.bounded(-max_angular_acc_, U(1, Slice()), max_angular_acc_));
      
      // 控制输入约束
      opti.subject_to(opti.bounded(-max_velocity_, U(0, Slice()), max_velocity_));
      opti.subject_to(opti.bounded(min_velocity_, U(0, Slice()), max_velocity_));
      opti.subject_to(opti.bounded(-max_angular_vel_, U(1, Slice()), max_angular_vel_));
      
      // 设置目标
      opti.minimize(cost);
      
      // 求解器选项
// QP solver options
// 1. 设置 QP 子求解器选项 (静默)
Dict qp_opts;
qp_opts["print_iter"] = false;
qp_opts["print_header"] = false;

// 2. 设置 SQP 主求解器选项 (静默)
Dict opts;
opts["qpsol"] = "qrqp";              // 使用 qrqp
opts["qpsol_options"] = qp_opts;     // 传入 QP 选项
opts["print_time"] = false;          // 不打印时间
opts["print_header"] = false;        // 不打印头部
opts["print_iteration"] = false;     // 不打印迭代过程
opts["print_status"] = false;     // 不打印 "Convergence achieved"

// 初始化 solver
opti.solver("sqpmethod", opts);

      // Dict opts;
      // opts["verbose"] = false;
      // opts["print_time"] = 0;
      // opts["max_iter"] = 50;
      // opts["tol_du"] = 1e-4;
      // opts["tol_pr"] = 1e-4;
      // opts["qpsol"] = "qrqp";
      
      // // opti.solver("ipopt", opts);
      // opti.solver("sqpmethod", opts);
      
      // 求解
      OptiSol sol = opti.solve();
      
      // 提取控制命令
      DM u_opt = sol.value(U);
      Eigen::Vector2d control;
      control << double(u_opt(0, 0)), double(u_opt(1, 0));
      
      // // 更新状态（简单的积分）
      // current_state_[3] += control[0] * dt_;  // v
      // current_state_[4] += control[1] * dt_;  // omega
      
      // // 限制速度
      // current_state_[3] = std::max(-max_velocity_, std::min(max_velocity_, current_state_[3]));
      // current_state_[4] = std::max(-max_angular_vel_, std::min(max_angular_vel_, current_state_[4]));
      
      return {true, control};
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "MPC求解异常: %s", e.what());
      return {false, Eigen::Vector2d::Zero()};
    }
  }
  
  // 计算障碍物代价（使用椭圆归一化距离）
  // 关键：必须接收MPC预测的机器人位置
  MX computeObstacleCost(const MX& robot_x, const MX& robot_y)
  {
    MX total_cost = 0;
    
    for (const auto& obs : obstacles_) {
      // 计算：如果机器人走到(robot_x, robot_y)，距离障碍物有多远？
      MX dist = ellipseCircleDistance(
        robot_x, robot_y,  // MPC第k步预测的机器人位置
        robot_radius_,
        obs.center.x, obs.center.y,
        obs.semi_major_axis, obs.semi_minor_axis, obs.rotation_angle);
      
      // 障碍物惩罚（指数衰减）
      // dist < 1: 如果走到这，会撞障碍物 → 惩罚很大
      // dist >= 1: 如果走到这，是安全的 → 惩罚衰减
      MX penalty = exp(-decay_rate_ * (dist - 1.0));
      
      total_cost += penalty;
    }
    
    return total_cost;
  }
  
  // 计算椭圆到圆的归一化距离
  // 关键：必须接收机器人的预测位置作为参数
  MX ellipseCircleDistance(const MX& robot_x, const MX& robot_y, double radius,
                           double ex, double ey, double a, double b, double angle)
  {
    // 机器人预测位置相对于障碍物中心的位置
    // 即使机器人当前在(0,0)，预测位置会是(0.1,0), (0.2,0.1)等
    MX dx = robot_x - ex;  // 预测位置X - 障碍物X
    MX dy = robot_y - ey;  // 预测位置Y - 障碍物Y
    
    // 旋转到椭圆局部坐标系
    double cos_a = std::cos(angle);
    double sin_a = std::sin(angle);

    MX lx = dx * cos_a + dy * sin_a;
    MX ly = -dx * sin_a + dy * cos_a;

    // 扩展椭圆，考虑机器人半径
    double A = a + radius;
    double B = b + radius;
    
    // 椭圆归一化距离
    // norm_dist < 1: 预测位置会撞到障碍物
    // norm_dist >= 1: 预测位置安全
    MX norm_dist = sqrt((lx * lx) / (A * A) + (ly * ly) / (B * B));
    
    return norm_dist;
  }
  
  // 获取参考状态
  std::vector<double> getReferenceState()
  {
    if (!has_goal_) {
      return {0, 0, 0};
    }
    
    // 所有预测步都使用相同的目标位姿
    const auto& pose = goal_pose_;
    
    // 提取位置和方向
    double x = pose.position.x;
    double y = pose.position.y;
    
    // 从四元数提取yaw角
    double qw = pose.orientation.w;
    double qz = pose.orientation.z;
    double theta = std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz);
    
    return {x, y, theta};
  }
  
  // 当前状态转为向量
  std::vector<double> current_state_vector()
  {
    return {current_state_[0], current_state_[1], current_state_[2]};
  }
  
  // 发布控制命令
  void publishControlCommand(const Eigen::Vector2d& control)
  {

    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = control[0];   // 当前线速度
    cmd.angular.z = control[1];  // 当前角速度
    
    cmd_vel_pub_->publish(cmd);
    
    RCLCPP_DEBUG(this->get_logger(), "发布控制命令: v=%.3f, omega=%.3f",
                 cmd.linear.x, cmd.angular.z);
  }
  
  // 发布零速度
  void publishZeroVelocity()
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = 0.0;
    cmd.angular.z = 0.0;
    cmd_vel_pub_->publish(cmd);
  }
  
  // 发布可视化
  void publishVisualization()
  {
    visualization_msgs::msg::MarkerArray markers;
    
    // 差速小车圆形标记
    visualization_msgs::msg::Marker robot_marker;
    robot_marker.header.frame_id = "base_link";
    robot_marker.header.stamp = this->now();
    robot_marker.ns = "mpc_robot";
    robot_marker.id = 0;
    robot_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    robot_marker.action = visualization_msgs::msg::Marker::ADD;
    
    robot_marker.pose.position.x = current_state_[0];
    robot_marker.pose.position.y = current_state_[1];
    robot_marker.pose.position.z = 0.0;
    
    // 设置方向
    double yaw = current_state_[2];
    robot_marker.pose.orientation.x = 0.0;
    robot_marker.pose.orientation.y = 0.0;
    robot_marker.pose.orientation.z = std::sin(yaw / 2.0);
    robot_marker.pose.orientation.w = std::cos(yaw / 2.0);
    
    robot_marker.scale.x = robot_radius_ * 2.0;
    robot_marker.scale.y = robot_radius_ * 2.0;
    robot_marker.scale.z = 0.3;
    
    robot_marker.color.r = 0.0;
    robot_marker.color.g = 0.8;
    robot_marker.color.b = 0.0;
    robot_marker.color.a = 0.7;
    
    markers.markers.push_back(robot_marker);
    
    marker_pub_->publish(markers);
  }

  // ROS相关
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<unitree_obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr obstacle_sub_;
  // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr speed_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 数据
  geometry_msgs::msg::Pose goal_pose_;
  Eigen::VectorXd current_state_;

  std::vector<unitree_obstacle_avoidance::msg::EllipseObstacle> obstacles_;
  
  bool has_goal_ = false;
  bool has_obstacles_ = false;
  
  // 参数
  int N_;
  double dt_;

  double robot_radius_;

  double max_velocity_;
  double min_velocity_;
  double max_angular_vel_;
  // double max_acceleration_;
  // double max_angular_acc_;
  
  double weight_pos_;
  double weight_ori_;
  double weight_vel_;
  double weight_ang_;
  double weight_obs_;

  double safety_distance_;
  double decay_rate_;

  
  // CasADi函数
  Function dynamics_func_;
  
  // 互斥锁
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MPCControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

