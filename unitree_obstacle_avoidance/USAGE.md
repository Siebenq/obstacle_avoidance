  ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped "
{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'base_link'
  },
  pose: {
    position: {x: 5.0, y: 0.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"