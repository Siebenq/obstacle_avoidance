#Simulation:

#0.start robot
ros2 launch robot_simulation simulate_gazebo.launch.py

#1.start pointcloud calibration node
ros2 launch obstacle_avoidance icp_calibration.launch.py

#2.start obstacle detection node
ros2 launch obstacle_avoidance obstacle_detection.launch.py

#3.start pointcloud fusion node
ros2 launch obstacle_avoidance pointcloud_fusion.launch.py

#4.start controller node
ros2 launch obstacle_avoidance apf_controller.launch.py

#5.set test goal position
  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  pose: {
    position: {x: 2.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" 

#In addition:show the real-time video of iphone
ros2 launch video_client network_receiver.launch.py


camera_optical_link
auxiliary_depth_camera_optical_link



colcon build --symlink-install


##
ros2 launch unitree_navigation navigation2.launch.py  use_sim_time:=true
ros2 run rqt_tf_tree rqt_tf_tree
ros2 launch camera_calibration icp_calibration.launch.py


# 查看完整TF树
ros2 run tf2_tools view_frames

# 查看特定两帧之间的关系
ros2 run tf2_ros tf2_echo trunk person_camera_depth_optical_frame


ros2 run depthimage_to_laserscan depthimage_to_laserscan_node --ros-args \
-p depth_frame_id:=camera_depth_frame \
-r /depth:=/camera/depth/image_rect_raw \
-r /depth_camera_info:=/camera/depth/camera_info