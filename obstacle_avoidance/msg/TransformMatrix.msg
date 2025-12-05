# 变换矩阵消息 - 4x4齐次变换矩阵
# 用于发布ICP标定结果

std_msgs/Header header

# 4x4变换矩阵，按行优先顺序存储（row-major）
# [m00 m01 m02 m03]
# [m10 m11 m12 m13]
# [m20 m21 m22 m23]
# [m30 m31 m32 m33]
float32[16] matrix



# 标定质量指标
float32 fitness_score       # ICP适配度分数（越小越好）
int32 num_iterations        # ICP迭代次数
bool converged             # 是否收敛
float32 rmse               # 均方根误差

# 源点云和目标点云的信息
string source_frame_id     # 源点云坐标系
string target_frame_id     # 目标点云坐标系
int32 source_points        # 源点云点数
int32 target_points        # 目标点云点数

