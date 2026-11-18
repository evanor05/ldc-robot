#include "ldc_navigation/grid_collision.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

using ldc_navigation::Footprint;
using ldc_navigation::footprint_pose_is_free;
using ldc_navigation::footprint_segment_is_free;
using ldc_navigation::grid_segment_is_free;
using ldc_navigation::rectangular_footprint;

// Independent oracle: intersect the segment with each closed obstacle rectangle.
bool hits_rectangle(double x, double y, double ex, double ey, int cx, int cy)
{
  double low = 0.0, high = 1.0;
  const double starts[] = {x, y}, deltas[] = {ex - x, ey - y};
  const int cells[] = {cx, cy};
  for (int axis = 0; axis < 2; ++axis) {
    const double a = cells[axis], b = a + 1.0, s = starts[axis], d = deltas[axis];
    if (d == 0.0) {
      if (s < a || s > b) {return false;}
    } else {
      double left = (a - s) / d, right = (b - s) / d;
      if (left > right) {std::swap(left, right);}
      low = std::max(low, left); high = std::min(high, right);
      if (low > high) {return false;}
    }
  }
  return true;
}

int main()
{
  const auto clear = [](uint32_t, uint32_t) {return true;};
  const auto wall = [](uint32_t x, uint32_t y) {return x != 2 || y != 2;};
  assert(grid_segment_is_free(0.5, 0.5, 4.5, 0.5, 5, 5, wall));
  assert(!grid_segment_is_free(0.5, 2.5, 4.5, 2.5, 5, 5, wall));
  assert(!grid_segment_is_free(1.5, 2.5, 2.5, 1.5, 5, 5, wall));  // corner
  assert(!grid_segment_is_free(1.5, 2.0, 3.5, 2.0, 5, 5, wall));  // edge
  assert(!grid_segment_is_free(2.0, 2.0, 2.0, 2.0, 5, 5, wall));
  assert(grid_segment_is_free(1.5, 1.5, 1.5, 1.5, 5, 5, wall));
  assert(!grid_segment_is_free(0.0, 0.5, 1.0, 0.5, 5, 5, clear));
  assert(!grid_segment_is_free(-0.1, 0.5, 1.0, 0.5, 5, 5, clear));
  assert(!grid_segment_is_free(0.5, 0.5, 5.0, 0.5, 5, 5, clear));
  assert(!grid_segment_is_free(std::numeric_limits<double>::quiet_NaN(),
    0.5, 1.0, 0.5, 5, 5, clear));
  assert(!grid_segment_is_free(0.5, 0.5, 1.0, 0.5, 0, 5, clear));

  std::mt19937 random(20261110);
  std::uniform_real_distribution<double> coordinate(0.05, 11.95);
  for (int trial = 0; trial < 800; ++trial) {
    std::vector<bool> occupied(144);
    for (int i = 0; i < 144; ++i) {
      occupied[i] = random() % 100 < 20;
    }
    const double x = coordinate(random), y = coordinate(random);
    const double ex = coordinate(random), ey = coordinate(random);
    bool expected = true;
    for (int cy = 0; cy < 12; ++cy) {
      for (int cx = 0; cx < 12; ++cx) {
        if (occupied[cy * 12 + cx] && hits_rectangle(x, y, ex, ey, cx, cy)) {
          expected = false;
        }
      }
    }
    const auto free = [&](uint32_t cx, uint32_t cy) {return !occupied[cy * 12 + cx];};
    assert(grid_segment_is_free(x, y, ex, ey, 12, 12, free) == expected);
    assert(grid_segment_is_free(ex, ey, x, y, 12, 12, free) == expected);
  }
  // ---- Footprint-aware checks: centerline free but body corner/edge hits ----
  {
    constexpr uint32_t W = 6, H = 6;
    std::vector<bool> blocked(W * H, false);
    blocked[1 * W + 1] = true;  // cell (1,1)
    const auto free = [&](uint32_t x, uint32_t y) {return !blocked[y * W + x];};
    const auto footprint = rectangular_footprint(1.0, 0.6, 0.1);
    // The centerline pose (2.5, 2.5) is free, but a 2.0 x 1.2 cell body
    // reaches into blocked cell (1,1) at yaw 0 / 90 / 45 degrees.
    assert(free(2, 2));
    assert(grid_segment_is_free(2.5, 2.5, 3.5, 2.5, W, H, free));
    assert(!footprint_pose_is_free(2.5, 2.5, 0.0, W, H, free, footprint));
    assert(!footprint_pose_is_free(2.5, 2.5, 1.5707963267948966, W, H, free, footprint));
    assert(!footprint_pose_is_free(2.5, 2.5, 0.7853981633974483, W, H, free, footprint));
    // Away from the obstacle the same body is accepted.
    assert(footprint_pose_is_free(4.0, 4.0, 0.0, W, H, free, footprint));
  }
  {
    // Narrow corridor: free lane x=2 only; a body 1.2 cells wide along x
    // cannot fit, although the centerline is free.
    constexpr uint32_t W = 5, H = 5;
    std::vector<bool> blocked(W * H, false);
    for (uint32_t y = 0; y < H; ++y) {
      blocked[y * W + 1] = true;
      blocked[y * W + 3] = true;
    }
    const auto free = [&](uint32_t x, uint32_t y) {return !blocked[y * W + x];};
    const auto footprint = rectangular_footprint(1.0, 0.6, 0.1);
    assert(free(2, 2));
    assert(footprint_pose_is_free(2.5, 2.5, 1.5707963267948966, W, H, free,
      rectangular_footprint(0.2, 0.2, 0.1)));
    assert(!footprint_pose_is_free(2.5, 2.5, 1.5707963267948966, W, H, free, footprint));
  }
  {
    // Doorway: wall on y=2 except a one-cell gap at x=2. A large body
    // straddles the wall; a small body fits through the gap.
    constexpr uint32_t W = 5, H = 5;
    std::vector<bool> blocked(W * H, false);
    for (uint32_t x = 0; x < W; ++x) {
      if (x != 2) {blocked[2 * W + x] = true;}
    }
    const auto free = [&](uint32_t x, uint32_t y) {return !blocked[y * W + x];};
    assert(free(2, 2));
    assert(!footprint_pose_is_free(2.5, 2.5, 0.0, W, H, free,
      rectangular_footprint(1.0, 0.6, 0.1)));
    assert(footprint_pose_is_free(2.5, 2.5, 0.0, W, H, free,
      rectangular_footprint(0.2, 0.2, 0.1)));
  }
  {
    // A swept body must reject an obstacle that the centerline misses.
    constexpr uint32_t W = 6, H = 5;
    std::vector<bool> blocked(W * H, false);
    blocked[1 * W + 2] = true;  // cell (2,1), just below y=2.0
    const auto free = [&](uint32_t x, uint32_t y) {return !blocked[y * W + x];};
    const auto footprint = rectangular_footprint(0.3, 0.6, 0.1);
    assert(grid_segment_is_free(0.5, 2.5, 4.5, 2.5, W, H, free));
    assert(!footprint_segment_is_free(0.5, 2.5, 4.5, 2.5, 0.0, W, H, free, footprint));
  }

  std::cout << "PASS grid collision: boundaries, 800 rectangle-oracle cases, footprint sweep\n";
}
