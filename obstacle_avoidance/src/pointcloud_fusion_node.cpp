#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "obstacle_avoidance/msg/transform_matrix.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "pcl/common/centroid.h"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Dense>

using namespace std::chrono_literals;

class PointCloudFusionNode : public rclcpp::Node
{
public:
  PointCloudFusionNode()
  : Node("pointcloud_fusion_node")
  {
    // 声明参数
    this->declare_parameter<std::string>("topic1", "/camera_dog/points");
    this->declare_parameter<std::string>("topic2", "/camera_person/points");
    this->declare_parameter<std::string>("topic3", "/icp_transform");
    this->declare_parameter<std::string>("output_topic", "/pointcloud_fused");
    this->declare_parameter<std::string>("target_frame", "base_link");
    
    // // 声明坐标变换参数（从topic2坐标系到topic1坐标系的变换）
    // this->declare_parameter<double>("transform_x", 0.0);
    // this->declare_parameter<double>("transform_y", 0.0);
    // this->declare_parameter<double>("transform_z", 0.0);
    // this->declare_parameter<double>("transform_roll", 0.0);
    // this->declare_parameter<double>("transform_pitch", 0.0);
    // this->declare_parameter<double>("transform_yaw", 0.0);
    
    // this->declare_parameter<bool>("use_transform", true);
    this->declare_parameter<bool>("use_tf2", false);  // 是否使用TF2自动查询变换
    
    // 获取参数值
    topic1_ = this->get_parameter("topic1").as_string();
    topic2_ = this->get_parameter("topic2").as_string();
    topic3_ = this->get_parameter("topic3").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    target_frame_ = this->get_parameter("target_frame").as_string();
    
    // double tx = this->get_parameter("transform_x").as_double();
    // double ty = this->get_parameter("transform_y").as_double();
    // double tz = this->get_parameter("transform_z").as_double();
    // double roll = this->get_parameter("transform_roll").as_double();
    // double pitch = this->get_parameter("transform_pitch").as_double();
    // double yaw = this->get_parameter("transform_yaw").as_double();
    
    // use_transform_ = this->get_parameter("use_transform").as_bool();
    use_tf2_ = this->get_parameter("use_tf2").as_bool();
    



    // // 构建变换矩阵（从欧拉角）
    // Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    // Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    // Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
    
    // Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    // Eigen::Matrix3d rotation = q.matrix();
    
    // transform_matrix_ = Eigen::Matrix4f::Identity();
    // transform_matrix_.block<3,3>(0,0) = rotation.cast<float>();
    // transform_matrix_(0,3) = tx;
    // transform_matrix_(1,3) = ty;
    // transform_matrix_(2,3) = tz;
    
    // 创建订阅者
    subscription1_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic1_, 10,
      std::bind(&PointCloudFusionNode::pointcloud1_callback, this, std::placeholders::_1));
    
    subscription2_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      topic2_, 10,
      std::bind(&PointCloudFusionNode::pointcloud2_callback, this, std::placeholders::_1));

      
    subscription3_ = this->create_subscription<obstacle_avoidance::msg::TransformMatrix>(
      topic3_,10,
      std::bind(&PointCloudFusionNode::matrix_callback,this,std::placeholders::_1));
    
    // 创建发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, 10);
    
    // 如果使用TF2，创建TF监听器
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
      RCLCPP_INFO(this->get_logger(), "使用TF2自动查询坐标变换");
    
    // 创建定时器进行融合和发布
    timer_ = this->create_wall_timer(
      100ms, std::bind(&PointCloudFusionNode::fusion_timer_callback, this));
    
    RCLCPP_INFO(this->get_logger(), "点云融合节点已启动");

    
    // if (use_transform_) {
    //   RCLCPP_INFO(this->get_logger(), 
    //     "变换参数 - 平移: [%.3f, %.3f, %.3f], 旋转(RPY): [%.3f, %.3f, %.3f]",
    //     tx, ty, tz, roll, pitch, yaw);
      
    //   // 打印变换矩阵
    //   RCLCPP_INFO(this->get_logger(), "变换矩阵:");
    //   for(int i = 0; i < 4; ++i) {
    //     RCLCPP_INFO(this->get_logger(), "[%.3f, %.3f, %.3f, %.3f]",
    //       transform_matrix_(i,0), transform_matrix_(i,1), 
    //       transform_matrix_(i,2), transform_matrix_(i,3));
    //   }
    // }
  }

private:
  void pointcloud1_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud1_msg_ = msg;
    received_cloud1_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "接收到点云1数据，点数: %d", 
                 msg->width * msg->height);
  }

  void pointcloud2_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    cloud2_msg_ = msg;
    received_cloud2_ = true;
    
    RCLCPP_DEBUG(this->get_logger(), "接收到点云2数据，点数: %d", 
                 msg->width * msg->height);
  }

  void matrix_callback(const obstacle_avoidance::msg::TransformMatrix::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    for(int i=0;i<16;i++){
      transform_matrix_(i) = msg->matrix[i];
    }
    received_matrix_ = true;
  }

  void fusion_timer_callback()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    
    // 检查是否收到两个点云数据
    if (!received_cloud1_ || !received_cloud2_||!received_matrix_) {
      if (!received_cloud1_ && !received_cloud2_&&!received_matrix_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "等待");
      } else if (!received_cloud1_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "等待接收点云1数据...");
      } else if(!received_cloud2_){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "等待接收点云2数据...");
      }else if(!received_matrix_){
        RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),5000,
                             "等待接收变换矩阵数据...");
      }
      return;
    }

    try {
      // 转换ROS消息到PCL点云
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      
      pcl::fromROSMsg(*cloud1_msg_, *cloud1);
      pcl::fromROSMsg(*cloud2_msg_, *cloud2);
      
      // 如果启用变换，将两个点云都变换到目标坐标系
      // if (use_transform_) {
        // 如果使用TF2，动态查询变换
        if (use_tf2_) {
          try {
            // 将cloud1变换到目标坐标系
            //点云数据都是在camera_optical_link坐标系下
            //这里将其通过TF转换到base_link坐标系下
            geometry_msgs::msg::TransformStamped transform1 = 
              tf_buffer_->lookupTransform(
                target_frame_,
                cloud1_msg_->header.frame_id,
                tf2::TimePointZero,
                std::chrono::milliseconds(100));
            
            Eigen::Affine3d transform1_eigen = tf2::transformToEigen(transform1.transform);
            Eigen::Matrix4f transform1_matrix = transform1_eigen.matrix().cast<float>();
            pcl::transformPointCloud(*cloud1, *cloud1_transformed, transform1_matrix);
            
            // 将cloud2变换到目标坐标系
            geometry_msgs::msg::TransformStamped transform2 = 
              tf_buffer_->lookupTransform(
                target_frame_,
                cloud2_msg_->header.frame_id,
                tf2::TimePointZero,
                std::chrono::milliseconds(100));
            
            Eigen::Affine3d transform2_eigen = tf2::transformToEigen(transform2.transform);
            Eigen::Matrix4f transform2_matrix = transform2_eigen.matrix().cast<float>();
            pcl::transformPointCloud(*cloud2, *cloud2_transformed, transform2_matrix);
            
            RCLCPP_DEBUG(this->get_logger(), 
              "TF2变换: %s -> %s, %s -> %s",
              cloud1_msg_->header.frame_id.c_str(), target_frame_.c_str(),
              cloud2_msg_->header.frame_id.c_str(), target_frame_.c_str());
              
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN(this->get_logger(), "TF2查询失败: %s，使用原始点云", ex.what());
            cloud1_transformed = cloud1;
            cloud2_transformed = cloud2;
          }
        } else {
          // 使用配置文件中的变换矩阵（cloud2到目标坐标系）
          // 假设cloud1已经在目标坐标系，只变换cloud2
          cloud1_transformed = cloud1;
          pcl::transformPointCloud(*cloud2, *cloud2_transformed, transform_matrix_);

        }

        // RCLCPP_INFO(this->get_logger(), "  变换矩阵 (cloud2 -> cloud1):");
        // for (int i = 0; i < 4; ++i) {
        //   RCLCPP_INFO(this->get_logger(), "    [%7.4f %7.4f %7.4f %7.4f]",
        //   transform_matrix_(i, 0), transform_matrix_(i, 1),
        //               transform_matrix_(i, 2), transform_matrix_(i, 3));
        // }
        
      // } else {
      //   // 不使用变换，直接使用原始点云
      //   cloud1_transformed = cloud1;
      //   cloud2_transformed = cloud2;
      // }
      
      // 融合变换后的点云（都在target_frame坐标系中）
      pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::PointCloud<pcl::PointXYZ>::Ptr fused_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
      *fused_cloud = *cloud1_transformed + *cloud2_transformed;


      //将融合后的点云转换到base_link坐标系下
      geometry_msgs::msg::TransformStamped fused_transform = 
      tf_buffer_->lookupTransform(
        target_frame_,
        fused_cloud->header.frame_id,
        tf2::TimePointZero,
        std::chrono::milliseconds(100));
    
      Eigen::Affine3d transform_eigen = tf2::transformToEigen(fused_transform.transform);
      Eigen::Matrix4f fused_transform_matrix = transform_eigen.matrix().cast<float>();
      pcl::transformPointCloud(*fused_cloud, *fused_cloud_transformed, fused_transform_matrix);
    

      
      // 转换回ROS消息
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*fused_cloud_transformed, output_msg);
      output_msg.header.stamp = this->now();
      output_msg.header.frame_id = target_frame_;
      
      // 发布融合后的点云
      publisher_->publish(output_msg);
      
      // RCLCPP_DEBUG(this->get_logger(), 
      //   "融合完成 - 点云1: %zu点, 点云2: %zu点, 融合后: %zu点",
      //   cloud1->points.size(), cloud2->points.size(), fused_cloud->points.size());
        
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "点云融合处理失败: %s", e.what());
    }
  }

  // 订阅者
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription1_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription2_;
  rclcpp::Subscription<obstacle_avoidance::msg::TransformMatrix>::SharedPtr subscription3_;
  // 发布者
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 点云数据
  sensor_msgs::msg::PointCloud2::SharedPtr cloud1_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr cloud2_msg_;
  
  // 状态标志
  bool received_cloud1_ = false;
  bool received_cloud2_ = false;
  bool received_matrix_ = false;
  // bool use_transform_ = true;
  bool use_tf2_ = false;
  
  // 参数
  std::string topic1_;
  std::string topic2_;
  std::string topic3_;
  std::string output_topic_;
  std::string target_frame_;
  
  // 变换矩阵
  Eigen::Matrix4f transform_matrix_;
  
  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  
  // 互斥锁
  std::mutex mutex_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PointCloudFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

