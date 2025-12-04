# 椭圆障碍物消息定义
# 用于表示简化后的障碍物（投影到地面的椭圆）

# 障碍物ID
int32 id

# 椭圆中心位置（3D坐标）
geometry_msgs/Point center

# 椭圆参数（在XY平面上）
float64 semi_major_axis    # 长半轴（米）
float64 semi_minor_axis    # 短半轴（米）
float64 rotation_angle     # 旋转角度（弧度，相对于X轴）

# 高度信息
float64 height             # 障碍物高度（米）
float64 z_min              # 最低点Z坐标（米）
float64 z_max              # 最高点Z坐标（米）

# 附加信息
int32 point_count          # 点云数量
float64 distance           # 到原点的距离（米）

