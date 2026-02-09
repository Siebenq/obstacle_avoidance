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
#include "pcl/search/kdtree.h"

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>

using namespace std::chrono_literals;

class ObstacleDetectionNode : public rclcpp::Node
{
  // 相机参数结构体（需要在public之前定义）
  struct CameraParams {
    double voxel_leaf_size;
    double range_min_forward, range_max_forward;
    double range_min_lateral, range_max_lateral;
    bool remove_ground;
    double ground_distance;
    bool remove_ceiling;
    double ceiling_distance;
    bool use_ransac_ground;         // 是否使用RANSAC去地面（适合倾斜相机）
    double ransac_ground_threshold; // RANSAC平面距离阈值
    double cluster_tolerance;
    int min_cluster_size;
    int max_cluster_size;
    double ellipse_scale_factor;  // 椭圆放大系数
  };
  
public:
  ObstacleDetectionNode()
  : Node("obstacle_detection_node")
  {
    // 声明参数 - 双相机点云话题
    this->declare_parameter<std::string>("topic_person", "/camera_person/points");
    this->declare_parameter<std::string>("topic_dog", "/camera_dog/points");
    this->declare_parameter<std::string>("output_obstacle_topic_person", "/obstacles_person");
    this->declare_parameter<std::string>("output_obstacle_topic_dog", "/obstacles_dog");
    this->declare_parameter<std::string>("output_marker_topic_person", "/obstacles_markers_person");
    this->declare_parameter<std::string>("output_marker_topic_dog", "/obstacles_markers_dog");
    
    // 坐标系类型：optical（Y向下，Z向前）或 standard（Z向上，X向前）
    this->declare_parameter<bool>("is_optical_frame", true);
    
    // ============ Person 相机参数 ============
    this->declare_parameter<double>("person.voxel_leaf_size", 0.05);
    this->declare_parameter<double>("person.range_min_forward", 0.3);
    this->declare_parameter<double>("person.range_max_forward", 10.0);
    this->declare_parameter<double>("person.range_min_lateral", -5.0);
    this->declare_parameter<double>("person.range_max_lateral", 5.0);
    this->declare_parameter<bool>("person.remove_ground", true);
    this->declare_parameter<double>("person.ground_distance", 1.6);
    this->declare_parameter<bool>("person.remove_ceiling", true);
    this->declare_parameter<double>("person.ceiling_distance", 1.5);
    this->declare_parameter<bool>("person.use_ransac_ground", true);      // person端默认启用RANSAC
    this->declare_parameter<double>("person.ransac_ground_threshold", 0.15);
    this->declare_parameter<double>("person.cluster_tolerance", 0.3);
    this->declare_parameter<int>("person.min_cluster_size", 30);
    this->declare_parameter<int>("person.max_cluster_size", 15000);
    this->declare_parameter<double>("person.ellipse_scale_factor", 1.2);
    
    // ============ Dog 相机参数 ============
    this->declare_parameter<double>("dog.voxel_leaf_size", 0.04);
    this->declare_parameter<double>("dog.range_min_forward", 0.3);
    this->declare_parameter<double>("dog.range_max_forward", 6.0);
    this->declare_parameter<double>("dog.range_min_lateral", -3.0);
    this->declare_parameter<double>("dog.range_max_lateral", 3.0);
    this->declare_parameter<bool>("dog.remove_ground", true);
    this->declare_parameter<double>("dog.ground_distance", 0.4);
    this->declare_parameter<bool>("dog.remove_ceiling", true);
    this->declare_parameter<double>("dog.ceiling_distance", 2.5);
    this->declare_parameter<bool>("dog.use_ransac_ground", false);       // dog端默认不启用
    this->declare_parameter<double>("dog.ransac_ground_threshold", 0.10);
    this->declare_parameter<double>("dog.cluster_tolerance", 0.2);
    this->declare_parameter<int>("dog.min_cluster_size", 30);
    this->declare_parameter<int>("dog.max_cluster_size", 8000);
    this->declare_parameter<double>("dog.ellipse_scale_factor", 1.0);  // dog端不放大
    
    // ============ 通用参数 ============
    this->declare_parameter<double>("ground_threshold", 0.2);
    this->declare_parameter<bool>("use_ransac_ground", false);
    this->declare_parameter<double>("min_obstacle_height", 0.1);
    this->declare_parameter<double>("max_obstacle_height", 3.0);
    this->declare_parameter<double>("min_obstacle_width", 0.05);
    this->declare_parameter<double>("max_obstacle_width", 5.0);
    
    // 获取话题参数
    topic_person_ = this->get_parameter("topic_person").as_string();
    topic_dog_ = this->get_parameter("topic_dog").as_string();
    output_obstacle_topic_person_ = this->get_parameter("output_obstacle_topic_person").as_string();
    output_obstacle_topic_dog_ = this->get_parameter("output_obstacle_topic_dog").as_string();
    output_marker_topic_person_ = this->get_parameter("output_marker_topic_person").as_string();
    output_marker_topic_dog_ = this->get_parameter("output_marker_topic_dog").as_string();
    
    is_optical_frame_ = this->get_parameter("is_optical_frame").as_bool();
    
    // 获取 Person 相机参数
    person_params_.voxel_leaf_size = this->get_parameter("person.voxel_leaf_size").as_double();
    person_params_.range_min_forward = this->get_parameter("person.range_min_forward").as_double();
    person_params_.range_max_forward = this->get_parameter("person.range_max_forward").as_double();
    person_params_.range_min_lateral = this->get_parameter("person.range_min_lateral").as_double();
    person_params_.range_max_lateral = this->get_parameter("person.range_max_lateral").as_double();
    person_params_.remove_ground = this->get_parameter("person.remove_ground").as_bool();
    person_params_.ground_distance = this->get_parameter("person.ground_distance").as_double();
    person_params_.remove_ceiling = this->get_parameter("person.remove_ceiling").as_bool();
    person_params_.ceiling_distance = this->get_parameter("person.ceiling_distance").as_double();
    person_params_.use_ransac_ground = this->get_parameter("person.use_ransac_ground").as_bool();
    person_params_.ransac_ground_threshold = this->get_parameter("person.ransac_ground_threshold").as_double();
    person_params_.cluster_tolerance = this->get_parameter("person.cluster_tolerance").as_double();
    person_params_.min_cluster_size = this->get_parameter("person.min_cluster_size").as_int();
    person_params_.max_cluster_size = this->get_parameter("person.max_cluster_size").as_int();
    person_params_.ellipse_scale_factor = this->get_parameter("person.ellipse_scale_factor").as_double();
    
    // 获取 Dog 相机参数
    dog_params_.voxel_leaf_size = this->get_parameter("dog.voxel_leaf_size").as_double();
    dog_params_.range_min_forward = this->get_parameter("dog.range_min_forward").as_double();
    dog_params_.range_max_forward = this->get_parameter("dog.range_max_forward").as_double();
    dog_params_.range_min_lateral = this->get_parameter("dog.range_min_lateral").as_double();
    dog_params_.range_max_lateral = this->get_parameter("dog.range_max_lateral").as_double();
    dog_params_.remove_ground = this->get_parameter("dog.remove_ground").as_bool();
    dog_params_.ground_distance = this->get_parameter("dog.ground_distance").as_double();
    dog_params_.remove_ceiling = this->get_parameter("dog.remove_ceiling").as_bool();
    dog_params_.ceiling_distance = this->get_parameter("dog.ceiling_distance").as_double();
    dog_params_.use_ransac_ground = this->get_parameter("dog.use_ransac_ground").as_bool();
    dog_params_.ransac_ground_threshold = this->get_parameter("dog.ransac_ground_threshold").as_double();
    dog_params_.cluster_tolerance = this->get_parameter("dog.cluster_tolerance").as_double();
    dog_params_.min_cluster_size = this->get_parameter("dog.min_cluster_size").as_int();
    dog_params_.max_cluster_size = this->get_parameter("dog.max_cluster_size").as_int();
    dog_params_.ellipse_scale_factor = this->get_parameter("dog.ellipse_scale_factor").as_double();
    
    // 获取通用参数
    ground_threshold_ = this->get_parameter("ground_threshold").as_double();
    use_ransac_ground_ = this->get_parameter("use_ransac_ground").as_bool();
    min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
    max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();
    min_obstacle_width_ = this->get_parameter("min_obstacle_width").as_double();
    max_obstacle_width_ = this->get_parameter("max_obstacle_width").as_double();
    
    // 创建订阅者
    sub_person_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_person_, 10,
      std::bind(&ObstacleDetectionNode::personCloudCallback, this, std::placeholders::_1));
    
    sub_dog_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic_dog_, 10,
      std::bind(&ObstacleDetectionNode::dogCloudCallback, this, std::placeholders::_1));
    
    // 创建发布者
    obstacle_pub_person_ = this->create_publisher<obstacle_avoidance::msg::EllipseObstacleArray>(
      output_obstacle_topic_person_, 10);
    obstacle_pub_dog_ = this->create_publisher<obstacle_avoidance::msg::EllipseObstacleArray>(
      output_obstacle_topic_dog_, 10);
    marker_pub_person_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      output_marker_topic_person_, 10);
    marker_pub_dog_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      output_marker_topic_dog_, 10);
    
    RCLCPP_INFO(this->get_logger(), "障碍物检测节点已启动（双相机独立参数模式）");
    RCLCPP_INFO(this->get_logger(), "  坐标系类型: %s", is_optical_frame_ ? "光学坐标系(Y下Z前)" : "标准坐标系(Z上X前)");
    RCLCPP_INFO(this->get_logger(), "  Person相机: 话题=%s, 前方=%.1f~%.1fm, 地面距离=%.1fm, 天花板距离=%.1fm", 
                topic_person_.c_str(), person_params_.range_min_forward, person_params_.range_max_forward,
                person_params_.ground_distance, person_params_.ceiling_distance);
    RCLCPP_INFO(this->get_logger(), "  Dog相机: 话题=%s, 前方=%.1f~%.1fm, 地面距离=%.1fm, 天花板距离=%.1fm", 
                topic_dog_.c_str(), dog_params_.range_min_forward, dog_params_.range_max_forward,
                dog_params_.ground_distance, dog_params_.ceiling_distance);
  }

private:
  void personCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "[person] 首次收到点云数据，frame_id: %s", 
                     msg->header.frame_id.c_str());
    processPointCloud(msg, obstacle_pub_person_, marker_pub_person_, "person", person_params_);
  }
  
  void dogCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_INFO_ONCE(this->get_logger(), "[dog] 首次收到点云数据，frame_id: %s", 
                     msg->header.frame_id.c_str());
    processPointCloud(msg, obstacle_pub_dog_, marker_pub_dog_, "dog", dog_params_);
  }
  
  /**
   * @brief 通用点云处理函数
   * 处理流程：1.ROI裁剪 → 2.去除地面/天花板 → 3.降采样 → 4.聚类 → 5.提取障碍物
   */
  void processPointCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg,
    rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr& obstacle_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_pub,
    const std::string& source_name,
    const CameraParams& params)
  {
    try {
      std::string input_frame_id = msg->header.frame_id;
      
      // 转换ROS消息到PCL点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*msg, *cloud);
      
      if (cloud->points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[%s] 接收到空点云数据", source_name.c_str());
        return;
      }
      
      size_t original_size = cloud->points.size();
      
      // ============ 步骤1: ROI裁剪（减少处理的数据量）============
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZ>);
      applyROIFilter(cloud, cloud_roi, params);
      
      if (cloud_roi->points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[%s] ROI裁剪后无点云 (原始: %zu)", source_name.c_str(), original_size);
        publishEmptyObstacles(obstacle_pub, marker_pub, msg->header.stamp, input_frame_id);
        return;
      }
      
      // ============ 步骤2: 去除地面和天花板 ============
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground_ceiling(new pcl::PointCloud<pcl::PointXYZ>);
      removeGroundAndCeiling(cloud_roi, cloud_no_ground_ceiling, params);
      
      if (cloud_no_ground_ceiling->points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "[%s] 去除地面/天花板后无点云 (ROI后: %zu)", source_name.c_str(), cloud_roi->points.size());
        publishEmptyObstacles(obstacle_pub, marker_pub, msg->header.stamp, input_frame_id);
        return;
      }
      
      // ============ 步骤3: 降采样 ============
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZ>);
      downsample(cloud_no_ground_ceiling, cloud_downsampled, params.voxel_leaf_size);
      
      if (cloud_downsampled->points.empty()) {
        publishEmptyObstacles(obstacle_pub, marker_pub, msg->header.stamp, input_frame_id);
        return;
      }
      
      // ============ 步骤4: 可选RANSAC精确地面去除（按相机配置）============
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_final(new pcl::PointCloud<pcl::PointXYZ>);
      if (params.use_ransac_ground) {
        removeGroundRANSAC(cloud_downsampled, cloud_final, params.ransac_ground_threshold);
      } else {
        cloud_final = cloud_downsampled;
      }
      
      if (cloud_final->points.empty()) {
        publishEmptyObstacles(obstacle_pub, marker_pub, msg->header.stamp, input_frame_id);
        return;
      }
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                   "[%s] 预处理: %zu → ROI %zu → 去地面/天花板 %zu → 降采样 %zu", 
                   source_name.c_str(), original_size, cloud_roi->points.size(),
                   cloud_no_ground_ceiling->points.size(), cloud_final->points.size());
      
      // ============ 步骤5: 聚类提取障碍物 ============
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters;
      clusterExtraction(cloud_final, clusters, params);
      
      if (clusters.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                             "[%s] 聚类后无障碍物 (预处理后点数: %zu)", source_name.c_str(), cloud_final->points.size());
        publishEmptyObstacles(obstacle_pub, marker_pub, msg->header.stamp, input_frame_id);
        return;
      }
      
      // ============ 步骤6: 提取椭圆障碍物信息 ============
      std::vector<obstacle_avoidance::msg::EllipseObstacle> obstacles;
      int id = 0;
      for (const auto& cluster : clusters) {
        auto obs = extractEllipseObstacle(cluster, id, params.ellipse_scale_factor);
        
        // 过滤不合理的障碍物
        if (obs.height >= min_obstacle_height_ && obs.height <= max_obstacle_height_ &&
            obs.semi_major_axis * 2 >= min_obstacle_width_ && 
            obs.semi_major_axis * 2 <= max_obstacle_width_) {
          obstacles.push_back(obs);
          id++;
        }
      }
      
      RCLCPP_INFO(this->get_logger(), "[%s] 原始 %zu → 预处理 %zu → 聚类 %zu → 有效障碍物 %zu", 
                  source_name.c_str(), original_size, cloud_final->points.size(),
                  clusters.size(), obstacles.size());
      
      // ============ 步骤7: 发布结果 ============
      publishObstacles(obstacle_pub, obstacles, msg->header.stamp, input_frame_id);
      publishObstacleMarkers(marker_pub, obstacles, msg->header.stamp, input_frame_id);
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "[%s] 点云处理失败: %s", source_name.c_str(), e.what());
    }
  }
  
  /**
   * @brief ROI范围滤波
   * 对于optical_frame: Z=前方深度, X=左右, Y=上下
   * 对于standard_frame: X=前方, Y=左右, Z=上下
   */
  void applyROIFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                      pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                      const CameraParams& params)
  {
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp1(new pcl::PointCloud<pcl::PointXYZ>);
    
    if (is_optical_frame_) {
      // 光学坐标系: Z=前方, X=右
      // 前方距离过滤（Z轴）
      pass.setInputCloud(cloud_in);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(params.range_min_forward, params.range_max_forward);
      pass.filter(*temp1);
      
      // 左右过滤（X轴）
      pass.setInputCloud(temp1);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(params.range_min_lateral, params.range_max_lateral);
      pass.filter(*cloud_out);
    } else {
      // 标准坐标系: X=前方, Y=左右
      pass.setInputCloud(cloud_in);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(params.range_min_forward, params.range_max_forward);
      pass.filter(*temp1);
      
      pass.setInputCloud(temp1);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(params.range_min_lateral, params.range_max_lateral);
      pass.filter(*cloud_out);
    }
  }
  
  /**
   * @brief 去除地面和天花板点云
   * 
   * 参数含义（相对于相机位置）：
   *   ground_distance: 相机到地面的距离（向下为正），过滤掉接近地面的点
   *   ceiling_distance: 相机到天花板的距离（向上为正），过滤掉接近天花板的点
   *   ground_margin: 地面上方保留的边距（不过滤的区域）
   * 
   * 对于optical_frame: Y轴向下为正
   *   相机在Y=0，地面在Y=+ground_distance，天花板在Y=-ceiling_distance
   *   保留范围: [-ceiling_distance + margin, ground_distance - margin]
   */
  void removeGroundAndCeiling(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                               const CameraParams& params)
  {
    if (!params.remove_ground && !params.remove_ceiling) {
      *cloud_out = *cloud_in;
      return;
    }
    
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_in);
    
    // 地面边距（保留地面上方多少距离的点）
    double ground_margin = 0.1;  // 10cm
    double ceiling_margin = 0.1; // 10cm
    
    // 如果启用了RANSAC地面检测，PassThrough只做粗略过滤（放宽地面边距）
    // 让RANSAC来精确去除倾斜地面
    if (params.use_ransac_ground) {
      ground_margin = -0.1;  // 负值=保留更多地面附近的点，交给RANSAC处理
    }
    
    if (is_optical_frame_) {
      // 光学坐标系: Y轴向下
      // 相机在 Y=0
      // 地面在 Y = +ground_distance（如相机高1.6m，地面在Y=1.6）
      // 天花板在 Y = -ceiling_distance（如天花板在相机上方2m，则Y=-2.0）
      pass.setFilterFieldName("y");
      
      // 保留的Y范围：
      // 最小Y（天花板方向）: -ceiling_distance + margin，或不限制
      // 最大Y（地面方向）: ground_distance - margin，或不限制
      double y_min = params.remove_ceiling ? (-params.ceiling_distance + ceiling_margin) : -100.0;
      double y_max = params.remove_ground ? (params.ground_distance - ground_margin) : 100.0;
      
      pass.setFilterLimits(y_min, y_max);
      
      RCLCPP_INFO_ONCE(this->get_logger(), "[optical] 垂直过滤Y轴: %.2f ~ %.2f (地面距离:%.1f, 天花板距离:%.1f, RANSAC:%s)", 
                       y_min, y_max, params.ground_distance, params.ceiling_distance,
                       params.use_ransac_ground ? "开" : "关");
    } else {
      // 标准坐标系: Z轴向上
      // 相机在某个Z高度，通常设为0
      // 地面在 Z = -ground_distance
      // 天花板在 Z = +ceiling_distance
      pass.setFilterFieldName("z");
      
      double z_min = params.remove_ground ? (-params.ground_distance + ground_margin) : -100.0;
      double z_max = params.remove_ceiling ? (params.ceiling_distance - ceiling_margin) : 100.0;
      
      pass.setFilterLimits(z_min, z_max);
      
      RCLCPP_INFO_ONCE(this->get_logger(), "[standard] 垂直过滤Z轴: %.2f ~ %.2f", z_min, z_max);
    }
    
    pass.filter(*cloud_out);
  }
  
  /**
   * @brief 降采样
   */
  void downsample(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                  pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                  double voxel_leaf_size)
  {
    if (cloud_in->points.size() < 100) {
      // 点太少，不降采样
      *cloud_out = *cloud_in;
      return;
    }
    
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_in);
    voxel_filter.setLeafSize(voxel_leaf_size, voxel_leaf_size, voxel_leaf_size);
    voxel_filter.filter(*cloud_out);
  }
  
  /**
   * @brief 使用RANSAC去除地面（支持倾斜相机）
   * 
   * 使用 SACMODEL_PERPENDICULAR_PLANE 限制法向量方向，
   * 确保只检测接近水平的平面（地面），不会误删墙壁等垂直面。
   * 对于 optical frame: 地面法向量接近 Y 轴方向
   * 对于 standard frame: 地面法向量接近 Z 轴方向
   * 
   * @param max_angle_deg 法向量与期望方向的最大偏差角度（度），
   *        允许一定倾斜（如相机倾斜30°则设为35°左右）
   */
  void removeGroundRANSAC(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                          pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_out,
                          double distance_threshold,
                          double max_angle_deg = 30.0)
  {
    if (cloud_in->points.size() < 100) {
      *cloud_out = *cloud_in;
      return;
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);
    seg.setMaxIterations(200);
    
    // 使用法向量约束，只检测接近水平的平面（地面）
    // 这样即使相机倾斜，也只会找到地面平面，不会误删墙壁
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    
    if (is_optical_frame_) {
      // optical frame: Y 轴是垂直方向（向下）
      seg.setAxis(Eigen::Vector3f(0.0f, 1.0f, 0.0f));
    } else {
      // standard frame: Z 轴是垂直方向（向上）
      seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    }
    // 允许法向量偏离的最大角度（弧度），适应倾斜相机
    seg.setEpsAngle(max_angle_deg * M_PI / 180.0);
    
    seg.setInputCloud(cloud_in);
    seg.segment(*inliers, *coefficients);
    
    if (inliers->indices.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "RANSAC未检测到地面平面");
      *cloud_out = *cloud_in;
      return;
    }
    
    RCLCPP_INFO_ONCE(this->get_logger(), 
      "RANSAC地面检测: 法向量=(%.2f,%.2f,%.2f), d=%.2f, 内点数=%zu/%zu",
      coefficients->values[0], coefficients->values[1], 
      coefficients->values[2], coefficients->values[3],
      inliers->indices.size(), cloud_in->points.size());
    
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_in);
    extract.setIndices(inliers);
    extract.setNegative(true);  // 保留非地面点
    extract.filter(*cloud_out);
  }
  
  /**
   * @brief 聚类提取
   */
  void clusterExtraction(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_in,
                         std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clusters,
                         const CameraParams& params)
  {
    if (cloud_in->points.size() < static_cast<size_t>(params.min_cluster_size)) {
      return;
    }
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_in);
    
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(params.cluster_tolerance);
    ec.setMinClusterSize(params.min_cluster_size);
    ec.setMaxClusterSize(params.max_cluster_size);
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
  
  /**
   * @brief 提取椭圆障碍物信息
   * 保持在原始点云坐标系中，不做坐标变换
   * 对于光学坐标系：X=右, Y=下, Z=前
   */
  obstacle_avoidance::msg::EllipseObstacle extractEllipseObstacle(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cluster, int id, double scale_factor)
  {
    obstacle_avoidance::msg::EllipseObstacle obs;
    obs.id = id;
    obs.point_count = cluster->points.size();
    
    // 获取边界框
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);
    
    // 计算中心点
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);
    
    // 保持原始坐标系，不做变换
    // 中心点就是质心的XYZ
    obs.center.x = centroid[0];
    obs.center.y = centroid[1];
    obs.center.z = centroid[2];
    
    // 记录边界
    obs.z_min = min_pt.z;
    obs.z_max = max_pt.z;
    
    if (is_optical_frame_) {
      // 光学坐标系: X=右, Y=下, Z=前
      // "高度"是Y方向的范围
      obs.height = max_pt.y - min_pt.y;
      
      // 距离：到相机原点的距离（主要看Z方向，即深度）
      obs.distance = centroid[2];  // 深度距离
      
      // PCA分析在XZ平面（水平面，因为Y是垂直方向）
      Eigen::MatrixXf points_xz(2, cluster->points.size());
      for (size_t i = 0; i < cluster->points.size(); ++i) {
        points_xz(0, i) = cluster->points[i].x - centroid[0];  // 左右
        points_xz(1, i) = cluster->points[i].z - centroid[2];  // 前后
      }
      
      Eigen::Matrix2f covariance = points_xz * points_xz.transpose() / cluster->points.size();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
      Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
      Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
      
      if (eigenvalues(0) > eigenvalues(1)) {
        std::swap(eigenvalues(0), eigenvalues(1));
        eigenvectors.col(0).swap(eigenvectors.col(1));
      }
      
      obs.semi_major_axis = std::sqrt(eigenvalues(1)) * 2.0 * scale_factor;
      obs.semi_minor_axis = std::sqrt(eigenvalues(0)) * 2.0 * scale_factor;
      // 旋转角度：在XZ平面内的角度
      obs.rotation_angle = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));
      
    } else {
      // 标准坐标系: X=前, Y=左, Z=上
      // "高度"是Z方向的范围
      obs.height = max_pt.z - min_pt.z;
      
      // 距离：水平面上到原点的距离
      obs.distance = std::sqrt(centroid[0] * centroid[0] + centroid[1] * centroid[1]);
      
      // PCA分析在XY平面（水平面）
      Eigen::MatrixXf points_xy(2, cluster->points.size());
      for (size_t i = 0; i < cluster->points.size(); ++i) {
        points_xy(0, i) = cluster->points[i].x - centroid[0];
        points_xy(1, i) = cluster->points[i].y - centroid[1];
      }
      
      Eigen::Matrix2f covariance = points_xy * points_xy.transpose() / cluster->points.size();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(covariance);
      Eigen::Vector2f eigenvalues = eigen_solver.eigenvalues();
      Eigen::Matrix2f eigenvectors = eigen_solver.eigenvectors();
      
      if (eigenvalues(0) > eigenvalues(1)) {
        std::swap(eigenvalues(0), eigenvalues(1));
        eigenvectors.col(0).swap(eigenvectors.col(1));
      }
      
      obs.semi_major_axis = std::sqrt(eigenvalues(1)) * 2.0 * scale_factor;
      obs.semi_minor_axis = std::sqrt(eigenvalues(0)) * 2.0 * scale_factor;
      obs.rotation_angle = std::atan2(eigenvectors(1, 1), eigenvectors(0, 1));
    }
    
    // 确保最小尺寸
    obs.semi_major_axis = std::max(obs.semi_major_axis, min_obstacle_width_ / 2.0);
    obs.semi_minor_axis = std::max(obs.semi_minor_axis, min_obstacle_width_ / 2.0);
    
    return obs;
  }
  
  void publishObstacles(
    rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr& pub,
    const std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles,
    const rclcpp::Time& timestamp,
    const std::string& frame_id)
  {
    obstacle_avoidance::msg::EllipseObstacleArray msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;
    msg.obstacles = obstacles;
    pub->publish(msg);
  }
  
  void publishEmptyObstacles(
    rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr& obstacle_pub,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& marker_pub,
    const rclcpp::Time& timestamp,
    const std::string& frame_id)
  {
    obstacle_avoidance::msg::EllipseObstacleArray msg;
    msg.header.stamp = timestamp;
    msg.header.frame_id = frame_id;
    obstacle_pub->publish(msg);
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = timestamp;
    delete_marker.ns = "obstacles";  // 与publishObstacleMarkers保持一致
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    visualization_msgs::msg::Marker delete_text;
    delete_text.header.frame_id = frame_id;
    delete_text.header.stamp = timestamp;
    delete_text.ns = "obstacle_text";
    delete_text.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_text);
    
    marker_pub->publish(marker_array);
  }
  
  void publishObstacleMarkers(
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& pub,
    const std::vector<obstacle_avoidance::msg::EllipseObstacle>& obstacles,
    const rclcpp::Time& timestamp,
    const std::string& frame_id)
  {
    visualization_msgs::msg::MarkerArray marker_array;
    
    // 删除旧标记
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = timestamp;
    delete_marker.ns = "obstacles";
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);
    
    for (const auto& obs : obstacles) {
      // 使用圆柱体显示椭圆障碍物
      visualization_msgs::msg::Marker cylinder_marker;
      cylinder_marker.header.frame_id = frame_id;
      cylinder_marker.header.stamp = timestamp;
      cylinder_marker.ns = "obstacles";
      cylinder_marker.id = obs.id;
      cylinder_marker.type = visualization_msgs::msg::Marker::CYLINDER;
      cylinder_marker.action = visualization_msgs::msg::Marker::ADD;
      
      // 位置：质心坐标
      cylinder_marker.pose.position.x = obs.center.x;
      cylinder_marker.pose.position.y = obs.center.y;
      cylinder_marker.pose.position.z = obs.center.z;
      
      // 旋转：使圆柱体垂直于地面
      // RViz中CYLINDER默认沿Z轴
      // 光学坐标系：Y轴向下（垂直方向），需要绕X轴旋转90度
      // 标准坐标系：Z轴向上（垂直方向），无需旋转
      if (is_optical_frame_) {
        // 绕X轴旋转90度，使圆柱体沿Y轴（垂直方向）
        Eigen::Quaterniond q(Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX()));
        cylinder_marker.pose.orientation.x = q.x();
        cylinder_marker.pose.orientation.y = q.y();
        cylinder_marker.pose.orientation.z = q.z();
        cylinder_marker.pose.orientation.w = q.w();
      } else {
        // 标准坐标系：圆柱体默认沿Z轴，已经是垂直的
        cylinder_marker.pose.orientation.x = 0.0;
        cylinder_marker.pose.orientation.y = 0.0;
        cylinder_marker.pose.orientation.z = 0.0;
        cylinder_marker.pose.orientation.w = 1.0;
      }
      
      // 尺寸：x=长轴直径，y=短轴直径，z=高度
      cylinder_marker.scale.x = std::max(0.1, static_cast<double>(obs.semi_major_axis * 2.0));
      cylinder_marker.scale.y = std::max(0.1, static_cast<double>(obs.semi_minor_axis * 2.0));
      cylinder_marker.scale.z = std::max(0.1, static_cast<double>(obs.height));
      
      // 颜色：根据距离变化（近红远绿）
      double dist_ratio = std::min(1.0, obs.distance / 5.0);
      cylinder_marker.color.r = 1.0 - dist_ratio * 0.5;
      cylinder_marker.color.g = dist_ratio * 0.8;
      cylinder_marker.color.b = 0.1;
      cylinder_marker.color.a = 0.7;
      
      cylinder_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(cylinder_marker);
      
      // 文本标记
      visualization_msgs::msg::Marker text_marker;
      text_marker.header.frame_id = frame_id;
      text_marker.header.stamp = timestamp;
      text_marker.ns = "obstacle_text";
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
      snprintf(text, sizeof(text), "ID%d:%.1fm", obs.id, obs.distance);
      text_marker.text = text;
      
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);
      marker_array.markers.push_back(text_marker);
    }
    
    pub->publish(marker_array);
  }

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_person_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_dog_;
  
  // 发布者
  rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr obstacle_pub_person_;
  rclcpp::Publisher<obstacle_avoidance::msg::EllipseObstacleArray>::SharedPtr obstacle_pub_dog_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_person_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_dog_;
  
  // 话题参数
  std::string topic_person_;
  std::string topic_dog_;
  std::string output_obstacle_topic_person_;
  std::string output_obstacle_topic_dog_;
  std::string output_marker_topic_person_;
  std::string output_marker_topic_dog_;
  
  // 坐标系类型
  bool is_optical_frame_;
  
  // 分相机参数
  CameraParams person_params_;
  CameraParams dog_params_;
  
  // 通用参数
  double ground_threshold_;
  bool use_ransac_ground_;
  double min_obstacle_height_, max_obstacle_height_;
  double min_obstacle_width_, max_obstacle_width_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ObstacleDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
