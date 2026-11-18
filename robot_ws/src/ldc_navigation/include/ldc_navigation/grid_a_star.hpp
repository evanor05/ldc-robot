#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <vector>

namespace ldc_navigation
{

/// 栅格 A* 搜索（纯算法，无 ROS 依赖，主机可单测）。
///
/// 设计约束：
///   1. 八连通；对角移动要求两个相邻正交格均可通行，禁止切角；
///   2. 代价 = 移动距离 + weight_cost * 栅格代价(0..254 归一化)，
///      让 A* 天然偏向低膨胀区，而不是只走最短；
///   3. 启发函数用 octile 距离（对角 1.414），且不含栅格代价项 ——
///      保证 h(n) <= 真实代价，可采纳性不被破坏；
///   4. 搜索中每 expand 一批节点检查一次 cancel，取消时不产出残缺路径。
struct GridAStarConfig
{
  bool allow_unknown = false;      ///< unknown(-1) 是否可通行
  double weight_cost = 0.0;        ///< 栅格代价权重（0 = 纯最短路）
  std::size_t max_expand = 200000; ///< 节点扩展上限，防呆
};

/// 搜索结果。failure 时 path 为空，expanded 记录实际扩展数（诊断用）。
struct GridAStarResult
{
  std::vector<std::pair<uint32_t, uint32_t>> path;  ///< (x, y) 栅格序列，含起终点
  std::size_t expanded = 0;
  bool success = false;
  bool cancelled = false;
};

/// 通行性查询回调：返回该格代价 0..254；负值表示 unknown。
/// 注意必须用 int16_t：costmap 的 253/254 超出 int8_t 范围。
using CellCostFn = std::function<int16_t(uint32_t x, uint32_t y)>;
/// 取消检查回调：返回 true 表示外部要求中止搜索。
using CancelFn = std::function<bool()>;

namespace detail
{
/// octile 启发：八连通平面上到目标的最短距离下界（可采纳）。
inline double octile(uint32_t x, uint32_t y, uint32_t gx, uint32_t gy)
{
  const double dx = std::abs(static_cast<double>(x) - gx);
  const double dy = std::abs(static_cast<double>(y) - gy);
  return dx + dy + (1.4142135623730951 - 2.0) * std::min(dx, dy);
}

struct Node
{
  uint32_t x, y;
  double g;      // 起点到此的实际代价
  double f;      // g + h
  // 优先队列默认取最大，故反向比较
  bool operator>(const Node & o) const {return f > o.f;}
};
}  // namespace detail

inline GridAStarResult search_grid_a_star(
  uint32_t start_x, uint32_t start_y,
  uint32_t goal_x, uint32_t goal_y,
  uint32_t width, uint32_t height,
  const CellCostFn & cell_cost,
  const GridAStarConfig & config = {},
  const CancelFn & cancel = {})
{
  GridAStarResult result;
  const auto traversable = [&](uint32_t x, uint32_t y) {
      const int16_t c = cell_cost(x, y);
      if (c < 0) {return config.allow_unknown;}   // unknown（NO_INFORMATION）
      // nav2_costmap_2d 约定：254 = LETHAL_OBSTACLE，253 = INSCRIBED_INFLATED
      // （内切障碍 = 车身已无余量）。其余膨胀代价按权重惩罚，不阻断。
      return c != 254 && c != 253;
    };

  if (start_x >= width || start_y >= height || goal_x >= width || goal_y >= height) {
    return result;
  }
  if (!traversable(start_x, start_y) || !traversable(goal_x, goal_y)) {
    return result;
  }

  const std::size_t cells = static_cast<std::size_t>(width) * height;
  std::vector<double> g_score(cells, std::numeric_limits<double>::infinity());
  std::vector<uint32_t> parent(cells, std::numeric_limits<uint32_t>::max());
  std::vector<bool> closed(cells, false);
  const auto idx = [&](uint32_t x, uint32_t y) {
      return static_cast<std::size_t>(y) * width + x;
    };

  std::priority_queue<detail::Node, std::vector<detail::Node>, std::greater<>> open;
  g_score[idx(start_x, start_y)] = 0.0;
  open.push({start_x, start_y, 0.0,
      detail::octile(start_x, start_y, goal_x, goal_y)});

  // 8 邻域：(dx, dy, 基础距离)。对角 = sqrt(2)。
  static constexpr double kSqrt2 = 1.4142135623730951;
  static constexpr int32_t kNeighbors[8][3] = {
    {1, 0, 1}, {-1, 0, 1}, {0, 1, 1}, {0, -1, 1},
    {1, 1, 2}, {1, -1, 2}, {-1, 1, 2}, {-1, -1, 2}
  };

  while (!open.empty() && result.expanded < config.max_expand) {
    const detail::Node cur = open.top();
    open.pop();
    const std::size_t ci = idx(cur.x, cur.y);
    if (closed[ci]) {continue;}
    closed[ci] = true;
    ++result.expanded;

    if (cur.x == goal_x && cur.y == goal_y) {
      // 回溯路径
      std::size_t i = ci;
      while (i != std::numeric_limits<std::size_t>::max()) {
        result.path.emplace_back(
          static_cast<uint32_t>(i % width), static_cast<uint32_t>(i / width));
        i = parent[i] == std::numeric_limits<uint32_t>::max() ?
          std::numeric_limits<std::size_t>::max() : parent[i];
      }
      std::reverse(result.path.begin(), result.path.end());
      result.success = true;
      return result;
    }

    // 每次扩展都给调用方取消机会（回调开销远小于一次扩展）
    if (cancel && cancel()) {
      result.path.clear();
      result.success = false;
      result.cancelled = true;
      return result;
    }

    const double cg = g_score[ci];
    for (const auto & nb : kNeighbors) {
      const int64_t nx = static_cast<int64_t>(cur.x) + nb[0];
      const int64_t ny = static_cast<int64_t>(cur.y) + nb[1];
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) {continue;}
      const uint32_t ux = static_cast<uint32_t>(nx), uy = static_cast<uint32_t>(ny);
      if (!traversable(ux, uy)) {continue;}

      // 禁止切角：对角移动的两个正交邻格必须都可通行
      if (nb[2] == 2) {
        const bool side1 = traversable(
          static_cast<uint32_t>(cur.x + nb[0]), cur.y);
        const bool side2 = traversable(
          cur.x, static_cast<uint32_t>(cur.y + nb[1]));
        if (!side1 || !side2) {continue;}
      }

      const double step = (nb[2] == 2 ? kSqrt2 : 1.0) *
        (1.0 + config.weight_cost * cell_cost(ux, uy) / 254.0);
      const double ng = cg + step;
      const std::size_t ni = idx(ux, uy);
      if (ng < g_score[ni]) {
        g_score[ni] = ng;
        parent[ni] = ci;
        open.push({ux, uy, ng, ng + detail::octile(ux, uy, goal_x, goal_y)});
      }
    }
  }
  return result;
}

}  // namespace ldc_navigation
