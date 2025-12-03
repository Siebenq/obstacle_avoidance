# MarkerArray和点云平面对齐 - 快速修复说明

## 问题
在RViz中，障碍物椭圆标记和点云显示在不同的高度，无法对齐。

## 原因
```cpp
// 错误：假设地面是z=0
obs.center.z = 0.0;  // ❌

// 但点云的Z范围可能是 [-0.5, 1.5]
// 导致标记在 [0, 2.0]，点云在 [-0.5, 1.5]
// 结果：错位！
```

## 修复
```cpp
// 正确：使用障碍物实际底部
obs.center.z = min_pt.z;  // ✅

// 标记基于实际底部
cylinder_marker.pose.position.z = obs.center.z + obs.height / 2.0;
ground_ellipse.pose.position.z = obs.center.z + 0.01;
text_marker.pose.position.z = obs.center.z + obs.height + 0.3;
```

## 效果对比

### 修复前
```
点云: z ∈ [-0.3, 0.8]
标记: z ∈ [0.0, 1.1]
❌ 标记比点云高0.3m
```

### 修复后
```
点云: z ∈ [-0.3, 0.8]
标记: z ∈ [-0.3, 0.8]
✅ 完美对齐！
```

## 可视化
```
修复前：              修复后：
  ╔═══╗                  [文本]
  ║   ║                  ╔═══╗
  ║   ║ ●●●             ║●●●║  ← 对齐
  ╚═══╝ ●●             ║●●●║
 ═══════               ╚═══╝
      ●●              ═══════
     ●●●
  ❌ 不对齐           ✅ 完美对齐
```

## 使用

```bash
# 已编译完成，直接启动
ros2 launch unitree_obstacle_avoidance full_pipeline.launch.py

# 在RViz中添加
- PointCloud2 → /pointcloud_fused
- MarkerArray → /obstacles_markers

# 验证：椭圆应该完全包围点云中的障碍物
```

## 核心变化

| 项目 | 修复前 | 修复后 |
|-----|--------|--------|
| obs.center.z | 0.0（固定） | min_pt.z（实际底部） |
| 柱体位置Z | height/2 | center.z + height/2 |
| 地面轮廓Z | 0.01 | center.z + 0.01 |
| 文本位置Z | height + 0.3 | center.z + height + 0.3 |

## 注意

- ✅ **不影响**MPC控制器（只用XY）
- ✅ **不影响**避障逻辑（只用XY）
- ✅ **只改善**可视化显示

## 详细文档

`MARKER_PLANE_FIX.md` - 完整技术说明

---

**结果**: 现在障碍物标记与点云在同一高度平面显示！🎯

