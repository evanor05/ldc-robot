// 纯算法单测：路径平滑 —— 无 ROS 依赖，主机 g++ 直接编译。
// 覆盖：收敛变直、端点不动、碰撞不越界、长度不爆、退化输入。
#include "ldc_navigation/path_smoother.hpp"
#include "ldc_navigation/grid_a_star.hpp"
#include <string>

#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

using ldc_navigation::PathSmootherConfig;
using ldc_navigation::path_length;
using ldc_navigation::smooth_path;

namespace
{
using Path = std::vector<std::pair<double, double>>;
constexpr double kEps = 1e-9;
}  // namespace

int main()
{
  // Refinement must perform every requested iteration, not stop after one
  // accepted sweep. Two plus two sweeps equals four sweeps of the same rule.
  {
    Path path = {{0, 0}, {1, 0}, {1, 2}, {3, 2}, {4, 4}};
    PathSmootherConfig cfg;
    cfg.max_its = 2; cfg.tolerance = 1e-14; cfg.do_refinement = true;
    const auto refined = smooth_path(path, nullptr, cfg);
    cfg.max_its = 4; cfg.do_refinement = false;
    const auto expected = smooth_path(path, nullptr, cfg);
    for (std::size_t i = 0; i < path.size(); ++i) {
      assert(std::hypot(refined[i].first - expected[i].first,
                        refined[i].second - expected[i].second) < 1e-12);
    }
  }
  // ---- 1. L 形折线平滑后：端点不动、路径变短（更直）、无点进障碍 ----
  {
    Path l_shape = {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}};
    const auto smoothed = smooth_path(l_shape, nullptr);
    assert(smoothed.size() == l_shape.size());
    assert(smoothed.front() == l_shape.front());
    assert(smoothed.back() == l_shape.back());
    assert(path_length(smoothed) < path_length(l_shape) + kEps);
  }

  // ---- 2. 碰撞约束：平滑点不得进入禁区 ----
  {
    // 原路径贴着禁区 x>1 绕行；强平滑会想抄近道，但禁区挡住 → 点不得越界
    Path around = {{0, 0}, {1, 0}, {1, 1}, {1, 2}, {0, 2}};
    const auto no_cross = [](double x, double, double ex, double) {
        return x <= 1.0 + kEps && ex <= 1.0 + kEps;
      };
    const auto smoothed = smooth_path(around, no_cross,
      PathSmootherConfig{0.1, 0.6, 1e-4, 500, false});
    for (const auto & [x, y] : smoothed) {
      (void)y;
      assert(x <= 1.0 + 1e-6);   // 没有任何平滑点越进禁区
    }
    assert(smoothed.front() == around.front());
    assert(smoothed.back() == around.back());
  }

  // ---- 3. 直线输入应保持不动（平滑无收益） ----
  {
    Path line;
    for (int i = 0; i <= 10; ++i) {
      line.emplace_back(i * 0.5, 0.0);
    }
    const auto smoothed = smooth_path(line, nullptr);
    for (std::size_t i = 0; i < line.size(); ++i) {
      assert(std::hypot(smoothed[i].first - line[i].first,
        smoothed[i].second - line[i].second) < 1e-6);
    }
  }

  // ---- 4. 退化输入：少于 3 点原样返回 ----
  {
    const Path empty, one = {{1, 2}}, two = {{0, 0}, {1, 1}};
    assert(smooth_path(empty, nullptr).empty());
    assert(smooth_path(one, nullptr).size() == 1);
    assert(smooth_path(two, nullptr).size() == 2);
  }

  // ---- 5. 平滑量随 w_smooth 增大而增大（可调性 sanity check） ----
  {
    Path zigzag;
    for (int i = 0; i <= 6; ++i) {
      zigzag.emplace_back(i, (i % 2) * 1.0);
    }
    const auto weak = smooth_path(zigzag, nullptr, PathSmootherConfig{0.3, 0.1, 1e-4, 100, false});
    const auto strong = smooth_path(zigzag, nullptr,
      PathSmootherConfig{0.3, 0.6, 1e-4, 100, false});
    assert(path_length(strong) < path_length(weak));
  }

  // Regression: free vertices do not imply free connecting segments (5 cm grid).
  {
    const std::vector<std::string> grid = {
      ".......#....", "#.#....##.#.", ".#.#..#...#.", "........#.#.",
      "............", ".#..........", ".#....#.#...", "...#....#...",
      "..#.#......#", ".......###..", "....#......#", "...#.......#"};
    const auto cost = [&](uint32_t x, uint32_t y) -> int16_t {
        return grid[y][x] == '#' ? 254 : 0;
      };
    const auto found = ldc_navigation::search_grid_a_star(1, 1, 10, 10, 12, 12, cost);
    assert(found.success);
    Path raw;
    for (const auto & [x, y] : found.path) {
      raw.emplace_back((x + 0.5) * 0.05, (y + 0.5) * 0.05);
    }
    const auto free = [&](uint32_t x, uint32_t y) {return grid[y][x] != '#';};
    const ldc_navigation::SegmentFreeFn edges = [&](double x, double y, double ex, double ey) {
        return ldc_navigation::grid_segment_is_free(x / 0.05, y / 0.05, ex / 0.05, ey / 0.05, 12,
        12, free);
      };
    const auto output = smooth_path(raw, edges);
    // Independent dense sampling also catches the original corner-cutting counterexample.
    for (std::size_t i = 1; i < output.size(); ++i) {
      for (int j = 0; j <= 1000; ++j) {
        const double t = j / 1000.0;
        const int x = std::floor((output[i - 1].first * (1 - t) + output[i].first * t) / 0.05);
        const int y = std::floor((output[i - 1].second * (1 - t) + output[i].second * t) / 0.05);
        assert(grid[y][x] != '#');
      }
    }
    bool rejected = false;
    try {
      smooth_path({{0.0970392, 0.0521015}, {0.133779, 0.0472708}}, edges);
    } catch (const std::invalid_argument &) {
      rejected = true;
    }
    assert(rejected);  // Invalid short input is not a safe fallback.
  }
  // safe_update must consult the orientation-dependent footprint predicate,
  // not only the centerline segment predicate.
  {
    Path path = {{1.5, 1.5}, {2.5, 2.5}, {3.5, 1.5}, {4.5, 2.5}};
    int pose_calls = 0;
    const auto pose_free = [&](double x, double /*y*/, double) {
        ++pose_calls;
        return !(x > 2.0 && x < 3.0);
      };
    const auto smoothed = smooth_path(
      path, nullptr, PathSmootherConfig{0.3, 0.45, 1e-4, 100, false}, {}, pose_free);
    assert(pose_calls > 0);
    assert(smoothed[1] == path[1]);
  }
  std::puts("path_smoother tests PASSED (8 cases)");
  return 0;
}
