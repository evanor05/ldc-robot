// 纯算法单测：grid A* —— 无 ROS 依赖，主机 g++ 直接编译。
// 覆盖：直路/绕障/禁切角/unknown 拒绝/取消/越界/起终点不可通行。
#include "ldc_navigation/grid_a_star.hpp"

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

using ldc_navigation::CellCostFn;
using ldc_navigation::GridAStarConfig;
using ldc_navigation::GridAStarResult;
using ldc_navigation::search_grid_a_star;

namespace
{

/// 测试栅格：'.'=free(0) '#'=lethal(100) '?'=unknown(-1)
std::vector<int16_t> make_grid(
  const std::vector<std::string> & rows, uint32_t & w, uint32_t & h)
{
  w = static_cast<uint32_t>(rows[0].size());
  h = static_cast<uint32_t>(rows.size());
  std::vector<int16_t> grid(w * h);
  for (uint32_t y = 0; y < h; ++y) {
    for (uint32_t x = 0; x < w; ++x) {
      const char c = rows[y][x];
      grid[y * w + x] = c == '#' ? 254 : (c == '?' ? -1 : 0);
    }
  }
  return grid;
}

CellCostFn cost_fn(const std::vector<int16_t> & g, uint32_t w, uint32_t h)
{
  // 必须按值捕获 w/h：它们是本函数的形参，返回后即销毁；
  // 若按引用捕获，lambda 里的 w/h 会变成悬垂引用（单测曾因此抓出崩溃）。
  return [g, w, h](uint32_t x, uint32_t y) {
           return (x < w && y < h) ? g[y * w + x] : static_cast<int16_t>(-2);
         };
}

bool has_path(const GridAStarResult & r) {return r.success && !r.path.empty();}

}  // namespace

int main()
{
  // ---- 1. 空地图直线：路径长 = 曼哈顿距离 ----
  {
    uint32_t w, h;
    const auto g = make_grid({"........",
          "........",
          "........"}, w, h);
    const auto r = search_grid_a_star(0, 0, 7, 2, w, h, cost_fn(g, w, h));
    assert(has_path(r));
    // 7 直 + 2 对角 → 步数 2 对角 + 5 直；格间长度 = 5*1 + 2*sqrt(2) ≈ 7.828
    const double len = r.path.size() - 1;   // 步数
    assert(r.path.front().first == 0 && r.path.front().second == 0);
    assert(r.path.back().first == 7 && r.path.back().second == 2);
    assert(len == 7.0);   // 5 直 + 2 对角 = 7 步
  }

  // ---- 2. 绕墙：唯一通道必须被找到 ----
  {
    uint32_t w, h;
    const auto g = make_grid({"....#....",
          "....#....",
          "....#....",
          "....#....",
          "........."}, w, h);
    const auto r = search_grid_a_star(0, 0, 8, 0, w, h, cost_fn(g, w, h));
    assert(has_path(r));
    for (const auto & [x, y] : r.path) {
      (void)x;
      assert(g[y * w + x] != 254);   // 每一步都不可落在障碍上
    }
  }

  // ---- 3. 禁切角：对角穿越墙角必须被拒绝 ----
  {
    uint32_t w, h;
    const auto g = make_grid({"...",
          ".#.",
          "..."}, w, h);
    // 从 (0,0) 到 (2,2)：若允许切角，可 (0,0)->(1,1)? 不行，(1,1)是障碍。
    // 关键用例：从 (0,1) 到 (1,2)？(1,2) free，对角邻接的两个正交格
    // (1,1)=障碍、(0,2)=free —— side1 失败，必须绕行或不可达由对角直连。
    const auto r = search_grid_a_star(0, 1, 2, 2, w, h, cost_fn(g, w, h));
    assert(has_path(r));
    // 验证没有任何一步是对角且穿过 (1,1) 的角
    for (std::size_t i = 1; i < r.path.size(); ++i) {
      const auto [x0, y0] = r.path[i - 1];
      const auto [x1, y1] = r.path[i];
      if (x1 != x0 && y1 != y0) {
        // 对角步：两个正交邻格都必须 free
        assert(g[y0 * w + x1] != 254 && g[y1 * w + x0] != 254);
      }
    }
  }

  // ---- 4. unknown 默认不可通行；allow_unknown 可放行 ----
  {
    uint32_t w, h;
    const auto g = make_grid({"..??",
          "..??",
          "..??",
          "...."}, w, h);
    const auto blocked = search_grid_a_star(0, 0, 3, 0, w, h, cost_fn(g, w, h));
    assert(!has_path(blocked));   // 目标在 unknown 里 → 失败
    GridAStarConfig allow;
    allow.allow_unknown = true;
    const auto through = search_grid_a_star(0, 0, 3, 0, w, h, cost_fn(g, w, h), allow);
    assert(has_path(through));
  }

  // ---- 5. 起点或终点在障碍上：立即失败，不产出路径 ----
  {
    uint32_t w, h;
    const auto g = make_grid({".#.",
          "..."}, w, h);
    assert(!has_path(search_grid_a_star(1, 0, 2, 1, w, h, cost_fn(g, w, h))));
    assert(!has_path(search_grid_a_star(0, 0, 1, 0, w, h, cost_fn(g, w, h))));
  }

  // ---- 6. 越界坐标：立即失败 ----
  {
    uint32_t w, h;
    const auto g = make_grid({"...", "..."}, w, h);
    assert(!has_path(search_grid_a_star(0, 0, 3, 0, w, h, cost_fn(g, w, h))));
    assert(!has_path(search_grid_a_star(0, 0, 1, 2, w, h, cost_fn(g, w, h))));
  }

  // ---- 7. 取消回调：搜索应中止并标记 cancelled ----
  {
    uint32_t w, h;
    const auto g = make_grid(std::vector<std::string>(60, std::string(60, '.')), w, h);
    bool cancelled = false;
    const auto r = search_grid_a_star(0, 0, 59, 59, w, h, cost_fn(g, w, h), {},
        [&] {cancelled = true; return true;});
    assert(cancelled);
    assert(!r.success && r.cancelled);
  }

  // ---- 8. 代价权重：低代价通道被优先选择 ----
  {
    uint32_t w, h;
    // 上半 free(0)，下半虽可通但代价高(120)：weight>0 时应走上半
    auto g = make_grid({"........",
          "........",
          "11111111"}, w, h);
    // 用 1..254 表示代价：重新生成 —— '1' 不是障碍
    for (uint32_t x = 0; x < w; ++x) {
      g[2 * w + x] = 120;
    }
    GridAStarConfig weighted;
    weighted.weight_cost = 2.0;
    const auto r = search_grid_a_star(0, 1, 7, 1, w, h, cost_fn(g, w, h), weighted);
    assert(has_path(r));
    for (const auto & [x, y] : r.path) {
      (void)x;
      assert(y != 2);   // 不应穿过高代价区
    }
  }

  std::puts("grid_a_star tests PASSED (8 cases)");
  return 0;
}
