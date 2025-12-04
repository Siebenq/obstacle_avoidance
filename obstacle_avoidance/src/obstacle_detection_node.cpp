#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "obstacle_avoidance/msg/ellipse_obstacle.hpp"
#include "obstacle_avoidance/msg/ellipse_obstacle_array.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/common/common.h"
#include "pcl/common/centroid.h"
#include "pcl/common/pca.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace std::chrono_literals;

class ObstacleDetectionNode : public rclcpp::Node
{
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("input_topic", "/pointcloud_fused");
    this->declare_parameter<std::string>("output_obstacle_topic", "/obstacles");
    this->declare_parameter<std::string>("output_marker_topic", "/obstacles_markers");
    this->declare_parameter<std::string>("frame_id", "base_link");
    
    // 点云滤波参数
    this->declare_parameter<double>("voxel_leaf_size", 0.05);
    this->declare_parameter<double>("z_filter_min", -1.5);
    this->declare_parameter<double>("z_filter_max", 2.0);
    this->declare_parameter<double>("y_filter_min", -5.0);
    this->declare_parameter<double>("y_filter_max", 5.0);
    this->declare_parameter<double>("x_filter_min", 0.0);
    this->declare_parameter<double>("x_filter_max", 10.0);
    
    // 地面分割参数
    this->declare_parameter<bool>("remove_ground", true);
    this->declare_parameter<double>("ground_threshold", 0.2);
    
    // 聚类参数
    this->declare_parameter<double>("cluster_tolerance", 0.5);
    this->declare_parameter<int>("min_cluster_size", 10);
    this->declare_parameter<int>("max_cluster_size", 10000);
    
    // 障碍物过滤参数
    this->declare_parameter<double>("min_obstacle_height", 0.1);
    this->declare_parameter<double>("max_obstacle_height", 3.0);
    this->declare_parameter<double>("min_obstacle_width", 0.05);
    this->declare_parameter<double>("max_obstacle_width", 5.0);
    
    // 椭圆简化参数
    this->declare_parameter<double>("ellipse_scale_factor", 1.2);  // 椭圆放大系数
    
    // 获取参数
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_obstacle_topic_ = this->get_parameter("output_obstacle_topic").as_string();
    output_marker_topic_ = this->get_parameter("output_marker_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    z_filter_min_ = this->get_parameter("z_filter_min").as_double();
    z_filter_max_ = this->get_parameter("z_filter_max").as_double();
    y_filter_min_ = this->get_parameter("y_filter_min").as_double();
    y_filter_max_ = this->get_parameter("y_filter_max").as_double();
    x_filter_min_ = this->get_parameter("x_filter_min").as_double();
    x_filter_max_ = this->get_parameter("x_filter_max").as_double();
    
    remove_ground_ = this->get_parameter("remove_ground").as_bool();
    ground_threshold_ = this->get_parameter("ground_threshold").as_double();
    
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    
    min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
    max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();
    min_obstacle_width_ = this->get_parameter("min_obstacle_width").as_double();
    max_obstacle_width_ = this->get_parameter("max_obstacle_width").as_double();
    
    ellipse_scale_factor_ = this->get_parameter("ellipse_scale_factor").as_double();
    
    // 创建订阅者
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, 10,
      std::bind(&ObstacleDetectionNode::pointcloud_callback, this, std::placeholders::_1));
    
    // 创建发布者
    obstacle_pub_ = this->create_publisher<obstacle_avoidance::msg::EllipseObstacleArray>(
      output_obstacle_topic_, 10);
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      output_marker_topic_, 10);
    
    RCLCPP_INFO(this->get_logger(), "障碍物检测节点已启动");

  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    try {
      // 保存输入点云的坐标系，用于输出
      std::string input_frame_id = msg->header.frame_id;
      
      // 转换ROS消息到PCL点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      
      if (cloud->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "接收到空点云数据");
        return;
      }
      
      RCLCPP_DEBUG(this->get_logger(), "接收到点云，点数: %zu", cloud->points.size());
      
      // 1. 降采样
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
      downsample(cloud, cloud_filtered);
      
      // 2. 范围滤波
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
      applyROIFilter(cloud_filtered, cloud_roi);
      
      if (cloud_roi->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "ROI滤波后无点云数据");
        publishEmptyObstacles(msg->header.stamp, input_frame_id);
        return;
      }
      
      // 3. 地面移除
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);
      if (remove_ground_) {
        removeGround(cloud_roi, cloud_no_ground);
      } else {
        cloud_no_ground = cloud_roi;
      }
      
      if (cloud_no_ground->points.empty()) {
        // RCLCPP_WARN(this->get_logger(), "地面移除后无点云数据");
        publishEmptyObstacles(msg->header.stamp, input_frame_id);
        return;
      }
      
      // 4. 聚类提取障碍物
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
      clusterExtraction(cloud_no_ground, clusters);
      
      // 5. 提取椭圆障碍物信息
      std::vector<obstacle_avoidance::msg::EllipseObstacle> obstacles;
      int id = 0;
      for (const auto& cluster : clusters) {
        auto obs = extractEllipseObstacle(cluster, id);
        
        // 过滤不合理的障碍物
        if (obs.height >= min_obstacle_height_ && obs.height <= max_obstacle_height_ &&
            obs.semi_major_axis * 2 >= min_obstacle_width_ && 
            obs.semi_major_axis * 2 <= max_obstacle_width_) {
          obstacles.push_back(obs);
          id++;
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "检测到有效障碍物数量: %zu", obstacles.size());
      
      // 6. 发布障碍物数组（使用输入点云的坐标系）
      publishObstacles(obstacles, msg->header.stamp, input_frame_id);
      
      // 7. 发布可视化标记（使用输入点云的坐标系）
      publishObstacleMarkers(obstacles, msg->header.stamp, input_frame_id);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "点云处理失败: %s", e.what());
    }
  }
  
  // 1.降采样
  void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_in);
    voxel_filter.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
    voxel_filter.filter(*cloud_out);
  }
  
  // 2.ROI范围滤波
  void applyROIFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    
    // X轴滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp1(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(x_filter_min_, x_filter_max_);
    pass.filter(*cloud_temp1);
    
    // Y轴滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp2(new pcl::PointCloud<pcl::PointXYZ>);
    pass.setInputCloud(cloud_temp1);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(y_filter_min_, y_filter_max_);
    pass.filter(*cloud_temp2);
    
    // Z轴滤波
    pass.setInputCloud(cloud_temp2);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(z_filter_min_, z_filter_max_);
    pass.filter(*cloud_out);
  }
  
  // 3.地面移除
  void removeGround(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out)
  {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(ground_threshold_);
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.empty()) {
      RCLCPP_WARN(this->get_logger(), "未检测到地面平面");
      *cloud_out = *cloud_in;
      return;
    }
    
    // 提取非地面点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloud_out);
  }
  
  // 4.聚类提取
  void clusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_in);
    ec.extract(cluster_indices);
    
    for (const auto& indices : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto& idx : indices.indices) {
        cluster->points.push_back(cloud_in->points[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;
      clusters.push_back(cluster);
    }
  }
  
  // 5.提取椭圆障碍物信息（使用PCA计算椭圆参数）
  obstacle_avoidance::msg::EllipseObstacle extractEllipseObstacle(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster, int id)
  {
    obstacle_avoidance::msg::EllipseObstacle obs;
    obs.id = id;
    obs.point_count = cluster->points.size();
    
    // 获取Z轴范围
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    obs.z_min = min_pt.z;
    obs.z_max = max_pt.z;
    obs.height = max_pt.z - min_pt.z;
    
    // 计算3D中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    
    // 椭圆中心使用XY平面投影，Z使用障碍物底部（最低点）
    // 这样椭圆就在障碍物的底部平面，与点云对齐
    obs.center.x = centroid[0];
    obs.center.y = centroid[1];
    obs.center.z = min_pt.z;  // 使用障碍物的最低点作为底部
    
    // 计算到原点的距离（使用地面投影的xy坐标）
    obs.distance = std::sqrt(obs.center.x * obs.center.x + 
                             obs.center.y * obs.center.y);
    
    // 只使用XY平面的点进行PCA分析
    Eigen::MatrixXf points_xy(2, cluster->points.size());
    for (size_t i = 0; i < cluster->points.size(); ++i) {
      points_xy(0, i) = cluster->points[i].x - centroid[0];
      points_xy(1, i) = cluster->points[i].y - centroid[1];
    }
    
    // 计算协方差矩阵
    Eigen::Matrix2f covariance = points_xy * points_xy.transpose() / cluster->points.size();
    
    // 特征值分解
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
    Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
    Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
    
    // 按特征值降序排列（大的对应长半轴）
    if (eigenvalues(0) > eigenvalues(1)) {
      std::swap(eigenvalues(0), eigenvalues(1));
      eigenvectors.col(0).swap(eigenvectors.col(1));
    }
    
    // 椭圆半轴长度（使用标准差的2倍，并乘以缩放因子）
    obs.semi_major_axis = std::sqrt(eigenvalues(1)) * 2.0 * ellipse_scale_factor_;
    obs.semi_minor_axis = std::sqrt(eigenvalues(0)) * 2.0 * ellipse_scale_factor_;
    
    // 确保最小尺寸
    obs.semi_major_axis = std::max(obs.semi_major_axis, min_obstacle_width_ / 2.0);
    obs.semi_minor_axis = std::max(obs.semi_minor_axis, min_obstacle_width_ / 2.0);
    
    // 旋转角度（长轴相对于X轴的角度）
    obs.rotation_angle = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));
    
    RCLCPP_DEBUG(this->get_logger(), 
      "障碍物 %d: 底部(%.2f, %.2f, %.2f), 长轴=%.2f, 短轴=%.2f, 角度=%.2f°, 高度=%.2f",
      id, obs.center.x, obs.center.y, obs.center.z,
      obs.semi_major_axis, obs.semi_minor_axis, 
      obs.rotation_angle * 180.0 / M_PI, obs.height);
    
    return obs;
  }
  
  // 6.发布障碍物数组
  void publishObstacles(const std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles,
                        const rclcpp::Time& timestamp,
                        const std::string& frame_id)
  {
    obstacle_avoidance::msg::EllipseObstacleArray msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;  // 使用输入点云的坐标系
    msg.obstacles = obstacles;
    
    obstacle_pub_->publish(msg);
  }
  
  // 发布空障碍物消息
  void publishEmptyObstacles(const rclcpp::Time& timestamp, const std::string& frame_id = "base_link")
  {
    obstacle_avoidance::msg::EllipseObstacleArray msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;  // 使用输入点云的坐标系
    obstacle_pub_->publish(msg);
    
    // 同时清除可视化标记
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;  // 使用输入点云的坐标系
    delete_marker.header.stamp = timestamp;
    delete_marker.ns = "ellipse_obstacles";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    marker_pub_->publish(marker_array);
  }
  
  // 7.发布障碍物可视化标记（椭圆柱体）
  void publishObstacleMarkers(
    const std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles,
    const rclcpp::Time& timestamp,
    const std::string& frame_id)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // 先删除所有旧标记
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;  // 使用输入点云的坐标系
    delete_marker.header.stamp = timestamp;
    delete_marker.ns = "ellipse_obstacles";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    // 为每个障碍物创建椭圆柱体标记
    for (const auto& obs : obstacles) {
      // 椭圆柱体标记 - 从地面向上延伸
      visualization_msgs::msg::Marker cylinder_marker;
      cylinder_marker.header.frame_id = frame_id;  // 使用输入点云的坐标系
      cylinder_marker.header.stamp = timestamp;
      cylinder_marker.ns = "ellipse_obstacles";
      cylinder_marker.id = obs.id * 2;
      cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      cylinder_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 位置：椭圆柱体的中心在障碍物中间
      // 底部在obs.center.z（障碍物最低点），顶部在obs.center.z + obs.height
      cylinder_marker.pose.position.x = obs.center.x;
      cylinder_marker.pose.position.y = obs.center.y;
      cylinder_marker.pose.position.z = obs.center.z + obs.height / 2.0;  // 中心 = 底部 + 高度/2
      
      // 旋转（椭圆的方向）
      Eigen::Quaterniond q(Eigen::AngleAxisd(obs.rotation_angle, Eigen::Vector3d::UnitZ()));
      cylinder_marker.pose.orientation.x = q.x();
      cylinder_marker.pose.orientation.y = q.y();
      cylinder_marker.pose.orientation.z = q.z();
      cylinder_marker.pose.orientation.w = q.w();
      
      // 尺寸（椭圆柱体）
      cylinder_marker.scale.x = obs.semi_major_axis * 2.0;  // 长轴直径
      cylinder_marker.scale.y = obs.semi_minor_axis * 2.0;  // 短轴直径
      cylinder_marker.scale.z = obs.height;  // 从地面到顶部的高度
      
      // 颜色（根据距离）
      cylinder_marker.color.r = std::min(1.0, 5.0 / (obs.distance + 1.0));
      cylinder_marker.color.g = std::min(1.0, obs.distance / 5.0);
      cylinder_marker.color.b = 0.2;
      cylinder_marker.color.a = 0.6;
      
      cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(cylinder_marker);
      
      // 地面椭圆标记（LINE_STRIP画椭圆轮廓）
      visualization_msgs::msg::Marker ground_ellipse;
      ground_ellipse.header.frame_id = frame_id;  // 使用输入点云的坐标系
      ground_ellipse.header.stamp = timestamp;
      ground_ellipse.ns = "ground_projection";
      ground_ellipse.id = obs.id * 3;
      ground_ellipse.type = visualization_msgs::msg::Marker::LINE_STRIP;
      ground_ellipse.action = visualization_msgs::msg::Marker::ADD;
      
      // 在障碍物底部画椭圆（稍微高一点避免z-fighting）
      ground_ellipse.pose.position.x = obs.center.x;
      ground_ellipse.pose.position.y = obs.center.y;
      ground_ellipse.pose.position.z = obs.center.z + 0.01;  // 在障碍物底部上方1cm
      ground_ellipse.pose.orientation.w = 1.0;
      
      ground_ellipse.scale.x = 0.05;  // 线宽
      
      ground_ellipse.color.r = 1.0;
      ground_ellipse.color.g = 0.5;
      ground_ellipse.color.b = 0.0;
      ground_ellipse.color.a = 0.9;
      
      // 生成椭圆轮廓点
      int num_points = 36;  // 36个点画椭圆
      for (int i = 0; i <= num_points; ++i) {
        double angle = 2.0 * M_PI * i / num_points;
        
        // 椭圆参数方程（局部坐标系）
        double x_local = obs.semi_major_axis * std::cos(angle);
        double y_local = obs.semi_minor_axis * std::sin(angle);
        
        // 旋转到全局坐标系
        double cos_rot = std::cos(obs.rotation_angle);
        double sin_rot = std::sin(obs.rotation_angle);
        double x_global = x_local * cos_rot - y_local * sin_rot;
        double y_global = x_local * sin_rot + y_local * cos_rot;
        
        geometry_msgs::msg::Point p;
        p.x = x_global;
        p.y = y_global;
        p.z = 0.0;
        ground_ellipse.points.push_back(p);
      }
      
      ground_ellipse.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(ground_ellipse);
      
      // 文本标记
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;  // 使用输入点云的坐标系
      text_marker.header.stamp = timestamp;
      text_marker.ns = "ellipse_text";
      text_marker.id = obs.id * 3 + 1;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 文本在障碍物顶部上方
      text_marker.pose.position.x = obs.center.x;
      text_marker.pose.position.y = obs.center.y;
      text_marker.pose.position.z = obs.center.z + obs.height + 0.3;  // 底部 + 高度 + 0.3m
      text_marker.pose.orientation.w = 1.0;
      
      text_marker.scale.z = 0.3;
      
      text_marker.color.r = 1.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 1.0;
      text_marker.color.a = 1.0;
      
      char text[256];
      snprintf(text, sizeof(text), 
               "ID:%d\nDist:%.2fm\nSize:%.2f×%.2fm\nH:%.2fm",
               obs.id, obs.distance, 
               obs.semi_major_axis * 2.0, obs.semi_minor_axis * 2.0,
               obs.height);
      text_marker.text = text;
      
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.2);
      marker_array.markers.push_back(text_marker);
    }
    
    marker_pub_->publish(marker_array);
  }

  // 订阅者和发布者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr obstacle_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  
  // 参数
  std::string input_topic_;
  std::string output_obstacle_topic_;
  std::string output_marker_topic_;
  std::string frame_id_;
  
  double voxel_leaf_size_;
  double z_filter_min_, z_filter_max_;
  double y_filter_min_, y_filter_max_;
  double x_filter_min_, x_filter_max_;
  
  bool remove_ground_;
  double ground_threshold_;
  
  double cluster_tolerance_;
  int min_cluster_size_;
  int max_cluster_size_;
  
  double min_obstacle_height_, max_obstacle_height_;
  double min_obstacle_width_, max_obstacle_width_;
  
  double ellipse_scale_factor_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
