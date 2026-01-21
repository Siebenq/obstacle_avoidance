Simulation:

0.start robot
ros2 launch robot_simulation simulate_gazebo.launch.py

1.start pointcloud calibration node
ros2 launch obstacle_avoidance icp_calibration.launch.py

2.start pointcloud fusion node
ros2 launch obstacle_avoidance pointcloud_fusion.launch.py

3.start obstacle detection node
ros2 launch obstacle_avoidance obstacle_detection.launch.py

4.start controller node
ros2 launch obstacle_avoidance apf_controller.launch.py

5.set test goal position
  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  pose: {
    position: {x: 4.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}" 


Offical test:
get the pointcloud from iphone:
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

(In addition:show the real-time video of iphone)
ros2 launch video_client network_receiver.launch.py