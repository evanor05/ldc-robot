#pragma once

#include <cmath>
#include <cstddef>
#include <functional>
#include <vector>
#include <stdexcept>
#include "ldc_navigation/grid_collision.hpp"

namespace ldc_navigation
{

/// 梯度式路径平滑（纯算法，无 ROS 依赖，思路同 nav2 SimpleSmoother）。
///
/// 迭代式把路径点往"邻居平均位置"拉（w_smooth），同时往"原始位置"拉（w_data），
/// 每次更新检查前后完整线段；调用方提供与搜索一致的栅格碰撞检查。
struct PathSmootherConfig
{
  double w_data = 0.3;             ///< 往原路径拉的权重
  double w_smooth = 0.45;          ///< 往平滑位置拉的权重（> w_data 才会收敛变弯为直）
  double tolerance = 1.0e-4;       ///< 整条路径单轮最大位移低于此即收敛
  std::size_t max_its = 1000;      ///< 最大迭代轮数
  bool do_refinement = true;       ///< 收敛后再整体 refinement 一轮
};


/// 平滑一条平面路径。输入/输出均为 (x, y) 序列，长度不变。
/// 端点永不移动（起点/终点语义必须保持）。
/// segment_free 检查闭线段；无地图约束时可传 nullptr。
inline std::vector<std::pair<double, double>> smooth_path(
  const std::vector<std::pair<double, double>> & path,
  const SegmentFreeFn & segment_free,
  const PathSmootherConfig & config = {},
  const std::function<void()> & check_cancel = {},
  const PoseFreeFn & pose_free = {})
{
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (check_cancel) {check_cancel();}
    const auto & p = path[i];
    const auto & prev = path[i == 0 ? 0 : i - 1];
    if (!std::isfinite(p.first) || !std::isfinite(p.second) ||
      (segment_free && !segment_free(prev.first, prev.second, p.first, p.second)))
    {
      throw std::invalid_argument("smoother input contains an invalid segment");
    }
  }
  if (path.size() < 3) {return path;}
  auto cur = path;
  const auto safe_update = [&](std::size_t i, double x, double y) {
      if (!std::isfinite(x) || !std::isfinite(y)) {return;}
      if (segment_free &&
        (!segment_free(cur[i - 1].first, cur[i - 1].second, x, y) ||
        !segment_free(x, y, cur[i + 1].first, cur[i + 1].second)))
      {
        return;
      }
      // The two adjacent segment checks above keep the centerline safe.
      // When a footprint predicate is supplied, also test the candidate pose
      // under both incoming and outgoing headings; this covers the turn at a
      // path vertex without requiring a ROS/costmap dependency here.
      if (pose_free) {
        const double in_yaw = std::atan2(y - cur[i - 1].second, x - cur[i - 1].first);
        const double out_yaw = std::atan2(cur[i + 1].second - y, cur[i + 1].first - x);
        if (!pose_free(x, y, in_yaw) || !pose_free(x, y, out_yaw)) {return;}
      }
      cur[i] = {x, y};
    };
  for (int pass = 0; pass < (config.do_refinement ? 2 : 1); ++pass) {
    double change = config.tolerance + 1.0;
    for (std::size_t it = 0; it < config.max_its && change >= config.tolerance; ++it) {
      change = 0.0;
      for (std::size_t i = 1; i + 1 < cur.size(); ++i) {
        if (check_cancel) {check_cancel();}
        const auto [x, y] = cur[i];
        const double nx = x + config.w_data * (path[i].first - x) +
          config.w_smooth * (cur[i - 1].first + cur[i + 1].first - 2.0 * x);
        const double ny = y + config.w_data * (path[i].second - y) +
          config.w_smooth * (cur[i - 1].second + cur[i + 1].second - 2.0 * y);
        safe_update(i, nx, ny);
        change += std::hypot(cur[i].first - x, cur[i].second - y);
      }
    }
  }
  return cur;
}

/// 折线总长度（诊断用：平滑前后长度对比，验证"平滑不改道"）。
inline double path_length(const std::vector<std::pair<double, double>> & path)
{
  double len = 0.0;
  for (std::size_t i = 1; i < path.size(); ++i) {
    len += std::hypot(path[i].first - path[i - 1].first,
        path[i].second - path[i - 1].second);
  }
  return len;
}

}  // namespace ldc_navigation
