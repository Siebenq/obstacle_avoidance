#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Dense>

#include "obstacle_avoidance/msg/transform_matrix.hpp"

/**
 * @brief ICP点云标定节点
 * 
 * 功能：
 * 1. 订阅两个点云话题 topic1 和 topic2
 * 2. 使用PCL的ICP算法计算两个点云坐标系之间的外参
 * 3. 将变换矩阵（Eigen::Matrix4f）发布到话题
 */
class ICPCalibrationNode : public rclcpp::Node
{
public:
  ICPCalibrationNode() : Node("icp_calibration_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("topic1", "/camera1/points");
    this->declare_parameter<std::string>("topic2", "/camera2/points");
    this->declare_parameter<std::string>("output_topic", "/icp_transform");
    
    // ICP参数
    this->declare_parameter<double>("max_correspondence_distance", 0.3);
    this->declare_parameter<int>("max_iterations", 50);
    this->declare_parameter<double>("transformation_epsilon", 1e-9);
    this->declare_parameter<double>("euclidean_fitness_epsilon", 1e-6);
    
    // 点云预处理参数
    this->declare_parameter<bool>("use_voxel_filter", true);
    this->declare_parameter<double>("voxel_leaf_size", 0.01);
    this->declare_parameter<bool>("use_outlier_removal", true);
    this->declare_parameter<int>("outlier_mean_k", 50);
    this->declare_parameter<double>("outlier_stddev_mul", 1.0);
    
    // 标定触发参数
    this->declare_parameter<bool>("auto_calibrate", true);
    this->declare_parameter<double>("calibration_interval", 5.0);  // 秒
    this->declare_parameter<int>("min_points_required", 100);
    
    // 获取参数
    topic1_ = this->get_parameter("topic1").as_string();
    topic2_ = this->get_parameter("topic2").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    
    max_correspondence_distance_ = this->get_parameter("max_correspondence_distance").as_double();
    max_iterations_ = this->get_parameter("max_iterations").as_int();
    transformation_epsilon_ = this->get_parameter("transformation_epsilon").as_double();
    euclidean_fitness_epsilon_ = this->get_parameter("euclidean_fitness_epsilon").as_double();
    
    use_voxel_filter_ = this->get_parameter("use_voxel_filter").as_bool();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    use_outlier_removal_ = this->get_parameter("use_outlier_removal").as_bool();
    outlier_mean_k_ = this->get_parameter("outlier_mean_k").as_int();
    outlier_stddev_mul_ = this->get_parameter("outlier_stddev_mul").as_double();
    
    auto_calibrate_ = this->get_parameter("auto_calibrate").as_bool();
    calibration_interval_ = this->get_parameter("calibration_interval").as_double();
    min_points_required_ = this->get_parameter("min_points_required").as_int();
    


    // 创建订阅者
    cloud1_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic1_, 10,
      std::bind(&ICPCalibrationNode::cloud1_callback, this, std::placeholders::_1));
    
    cloud2_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic2_, 10,
      std::bind(&ICPCalibrationNode::cloud2_callback, this, std::placeholders::_1));
    
    // 创建发布者
    transform_pub_ = this->create_publisher<obstacle_avoidance::msg::TransformMatrix>(
      output_topic_, 10);
    


    // 自动标定定时器
    if (auto_calibrate_) {
      calibration_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(calibration_interval_),
        std::bind(&ICPCalibrationNode::calibration_timer_callback, this));
    }
    
    RCLCPP_INFO(this->get_logger(), "ICP标定节点已启动");
  }

private:
  /**
   * @brief 点云1回调函数
   */
  void cloud1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud1_msg_ = msg;
    has_cloud1_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "收到点云1: %d 点", 
                 msg->width * msg->height);
  }
  
  /**
   * @brief 点云2回调函数
   */
  void cloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud2_msg_ = msg;
    has_cloud2_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "收到点云2: %d 点", 
                 msg->width * msg->height);
  }
  
  /**
   * @brief 自动标定定时器回调
   */
  void calibration_timer_callback()
  {
    if (!has_cloud1_ || !has_cloud2_) {
      RCLCPP_WARN(this->get_logger(), "等待点云数据... (cloud1: %s, cloud2: %s)",
                  has_cloud1_ ? "✓" : "✗", has_cloud2_ ? "✓" : "✗");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "开始ICP标定...");
    performCalibration();
  }
  
  /**
   * @brief 执行ICP标定
   */
  void performCalibration()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!has_cloud1_ || !has_cloud2_) {
      RCLCPP_ERROR(this->get_logger(), "点云数据不完整，无法标定");
      return;
    }
    
    // 转换为PCL格式
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::fromROSMsg(*cloud1_msg_, *cloud1);
    pcl::fromROSMsg(*cloud2_msg_, *cloud2);
    
    RCLCPP_INFO(this->get_logger(), "  点云1: %zu 点 (frame: %s)", 
                cloud1->points.size(), cloud1_msg_->header.frame_id.c_str());
    RCLCPP_INFO(this->get_logger(), "  点云2: %zu 点 (frame: %s)", 
                cloud2->points.size(), cloud2_msg_->header.frame_id.c_str());
    
    // 检查点数
    if (cloud1->points.size() < (size_t)min_points_required_ ||
        cloud2->points.size() < (size_t)min_points_required_) {
      RCLCPP_ERROR(this->get_logger(), 
                   "点云点数不足（需要至少 %d 点）", min_points_required_);
      return;
    }
    
    // 预处理点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_processed = preprocessPointCloud(cloud1);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_processed = preprocessPointCloud(cloud2);
    
    RCLCPP_INFO(this->get_logger(), "  预处理后: cloud1=%zu, cloud2=%zu", 
                cloud1_processed->points.size(), cloud2_processed->points.size());
    
    // 配置ICP
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(cloud2_processed);  // cloud2作为源（要变换的点云）
    icp.setInputTarget(cloud1_processed);  // cloud1作为目标（参考点云）
    
    icp.setMaxCorrespondenceDistance(max_correspondence_distance_);
    icp.setMaximumIterations(max_iterations_);
    icp.setTransformationEpsilon(transformation_epsilon_);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon_);
    
    // 执行ICP
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    icp.align(*aligned_cloud);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    
    // 获取结果
    bool converged = icp.hasConverged();
    double fitness_score = icp.getFitnessScore();
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    
    RCLCPP_INFO(this->get_logger(), "ICP标定完成:");

    if (!converged) {
      RCLCPP_WARN(this->get_logger(), "ICP未收敛，结果可能不准确");
    }
    
    // 打印变换矩阵
    RCLCPP_INFO(this->get_logger(), "  变换矩阵 (cloud2 -> cloud1):");
    for (int i = 0; i < 4; ++i) {
      RCLCPP_INFO(this->get_logger(), "    [%7.4f %7.4f %7.4f %7.4f]",
                  transformation(i, 0), transformation(i, 1),
                  transformation(i, 2), transformation(i, 3));
    }
    
    // 提取平移和旋转
    Eigen::Vector3f translation = transformation.block<3, 1>(0, 3);
    Eigen::Matrix3f rotation = transformation.block<3, 3>(0, 0);
    Eigen::Vector3f euler = rotation.eulerAngles(2, 1, 0);  // ZYX顺序
    
    RCLCPP_INFO(this->get_logger(), "  平移 (x, y, z): (%.4f, %.4f, %.4f) m",
                translation[0], translation[1], translation[2]);
    RCLCPP_INFO(this->get_logger(), "  旋转 (roll, pitch, yaw): (%.2f°, %.2f°, %.2f°)",
                euler[2] * 180.0 / M_PI, euler[1] * 180.0 / M_PI, euler[0] * 180.0 / M_PI);
    
    // 计算RMSE
    double rmse = std::sqrt(fitness_score);
    
    // 发布结果
    publishTransform(transformation, fitness_score, rmse, converged,
                     cloud1_msg_->header.frame_id, cloud2_msg_->header.frame_id,
                     cloud1->points.size(), cloud2->points.size());
  }
  
  /**
   * @brief 预处理点云（降采样和离群点去除）
   */
  pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr processed_cloud = cloud;
    
    // 体素滤波降采样
    if (use_voxel_filter_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
      voxel_grid.setInputCloud(processed_cloud);
      voxel_grid.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
      voxel_grid.filter(*voxel_filtered);
      processed_cloud = voxel_filtered;
    }
    
    // 统计离群点去除
    if (use_outlier_removal_) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
      sor.setInputCloud(processed_cloud);
      sor.setMeanK(outlier_mean_k_);
      sor.setStddevMulThresh(outlier_stddev_mul_);
      sor.filter(*outlier_removed);
      processed_cloud = outlier_removed;
    }
    
    return processed_cloud;
  }
  
  /**
   * @brief 发布变换矩阵
   */
  void publishTransform(const Eigen::Matrix4f& transform, 
                       double fitness_score, double rmse, bool converged,
                       const std::string& source_frame, const std::string& target_frame,
                       int source_points, int target_points)
  {
    obstacle_avoidance::msg::TransformMatrix msg;
    
    // 设置header
    msg.header.stamp = this->now();
    msg.header.frame_id = "icp_calibration";
    
    // // 转换矩阵为行优先数组
    // for (int i = 0; i < 4; ++i) {
    //   for (int j = 0; j < 4; ++j) {
    //     msg.matrix[i * 4 + j] = transform(i, j);
    //   }
    // }
    for(int i=0;i<16;i++){
      msg.matrix[i]=transform(i);
    }
    
    // 设置质量指标
    msg.fitness_score = fitness_score;
    msg.num_iterations = max_iterations_;  // PCL的ICP不直接提供实际迭代次数
    msg.converged = converged;
    msg.rmse = rmse;
    
    // 设置坐标系信息
    msg.source_frame_id = source_frame;
    msg.target_frame_id = target_frame;
    msg.source_points = source_points;
    msg.target_points = target_points;
    
    // 发布
    transform_pub_->publish(msg);
    
    RCLCPP_INFO(this->get_logger(), "变换矩阵已发布到 %s", output_topic_.c_str());
  }
  




  // ROS相关
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud1_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud2_sub_;
  rclcpp::Publisher<obstacle_avoidance::msg::TransformMatrix>::SharedPtr transform_pub_;
  rclcpp::TimerBase::SharedPtr calibration_timer_;
  
  // 点云数据
  sensor_msgs::msg::PointCloud2::SharedPtr cloud1_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2_msg_;
  bool has_cloud1_ = false;
  bool has_cloud2_ = false;
  
  // 参数
  std::string topic1_;
  std::string topic2_;
  std::string output_topic_;
  
  // ICP参数
  double max_correspondence_distance_;
  int max_iterations_;
  double transformation_epsilon_;
  double euclidean_fitness_epsilon_;
  
  // 预处理参数
  bool use_voxel_filter_;
  double voxel_leaf_size_;
  bool use_outlier_removal_;
  int outlier_mean_k_;
  double outlier_stddev_mul_;
  
  // 标定触发参数
  bool auto_calibrate_;
  double calibration_interval_;
  int min_points_required_;
  
  // 线程安全
  std::mutex mutex_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<ICPCalibrationNode>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("icp_calibration"), "异常: %s", e.what());
  }
  
  rclcpp::shutdown();
  return 0;
}

