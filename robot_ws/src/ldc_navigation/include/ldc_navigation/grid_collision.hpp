#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <utility>
#include <vector>

namespace ldc_navigation
{
using CellFreeFn = std::function<bool(uint32_t, uint32_t)>;
using SegmentFreeFn = std::function<bool(double, double, double, double)>;
/// Footprint pose predicate: (x, y, yaw) in cell units. yaw is radians in the
/// same frame as x/y. Used to check the orientation-dependent vehicle body.
using PoseFreeFn = std::function<bool(double, double, double)>;

/// Check every grid cell touched by a closed segment, including corner/edge contacts.
/// Coordinates are in cell units: (world - origin) / resolution. Contact with
/// a blocked cell or the outside of the grid is rejected. This is a centerline
/// check on an already inflated map, not an orientation-dependent footprint check.
inline bool grid_segment_is_free(
  double x0, double y0, double x1, double y1,
  uint32_t width, uint32_t height, const CellFreeFn & free)
{
  if (!free || width == 0 || height == 0 ||
    !std::isfinite(x0) || !std::isfinite(y0) ||
    !std::isfinite(x1) || !std::isfinite(y1))
  {
    return false;
  }
  const auto inside = [&](double x, double y) {
      return x >= 0.0 && y >= 0.0 && x < width && y < height;
    };
  if (!inside(x0, y0) || !inside(x1, y1)) {return false;}

  // A point exactly on an edge touches both cells; on a corner, all four.
  const auto point_free = [&](double x, double y) {
      constexpr double epsilon = 1e-9;
      const bool edge_x = std::abs(x - std::round(x)) <= epsilon;
      const bool edge_y = std::abs(y - std::round(y)) <= epsilon;
      const int64_t ix = static_cast<int64_t>(edge_x ? std::round(x) : std::floor(x));
      const int64_t iy = static_cast<int64_t>(edge_y ? std::round(y) : std::floor(y));
      for (int dx = 0; dx <= static_cast<int>(edge_x); ++dx) {
        for (int dy = 0; dy <= static_cast<int>(edge_y); ++dy) {
          const int64_t cx = ix - dx, cy = iy - dy;
          if (cx < 0 || cy < 0 || cx >= width || cy >= height ||
            !free(static_cast<uint32_t>(cx), static_cast<uint32_t>(cy)))
          {
            return false;
          }
        }
      }
      return true;
    };
  if (!point_free(x0, y0) || !point_free(x1, y1)) {return false;}
  const double dx = x1 - x0, dy = y1 - y0;
  if (dx == 0.0 && dy == 0.0) {return true;}
  const double inf = std::numeric_limits<double>::infinity();
  const auto first_crossing = [&](double v, double delta) {
      if (delta == 0.0) {return inf;}
      const double boundary = delta > 0.0 ? std::floor(v) + 1.0 : std::ceil(v) - 1.0;
      return (boundary - v) / delta;
    };
  double tx = first_crossing(x0, dx), ty = first_crossing(y0, dy);
  const double dtx = dx == 0.0 ? inf : 1.0 / std::abs(dx);
  const double dty = dy == 0.0 ? inf : 1.0 / std::abs(dy);
  double t = 0.0;
  const uint64_t max_steps = static_cast<uint64_t>(width) + height + 3;
  for (uint64_t step = 0; step < max_steps; ++step) {
    const double next = std::min({tx, ty, 1.0});
    if (next < t) {return false;}
    const double middle = (t + next) * 0.5;
    if (!point_free(x0 + dx * middle, y0 + dy * middle) ||
      !point_free(x0 + dx * next, y0 + dy * next))
    {
      return false;
    }
    if (next == 1.0) {return true;}
    if (tx <= next + 1e-12) {tx += dtx;}
    if (ty <= next + 1e-12) {ty += dty;}
    t = next;
  }
  return false;
}

/// Convex footprint approximation in cell units, relative to the robot origin.
/// `vertices` must be a simple convex polygon (Nav2's footprint contract);
/// `sample_step` is the sweep sampling interval in cells.
struct Footprint
{
  std::vector<std::pair<double, double>> vertices;
  double sample_step = 0.25;
};

/// Build a rectangle footprint centered at the robot origin.
inline Footprint rectangular_footprint(
  double half_length, double half_width, double sample_step = 0.25)
{
  Footprint footprint;
  footprint.sample_step = sample_step;
  if (half_length > 0.0 && half_width > 0.0) {
    footprint.vertices = {
      {-half_length, -half_width}, {half_length, -half_width},
      {half_length, half_width}, {-half_length, half_width}};
  }
  return footprint;
}

namespace detail
{
/// Separating-axis test between a convex polygon and an axis-aligned cell.
inline bool convex_polygon_intersects_cell(
  const std::vector<std::pair<double, double>> & polygon, double cx, double cy)
{
  const std::pair<double, double> cell[4] = {
    {cx, cy}, {cx + 1.0, cy}, {cx + 1.0, cy + 1.0}, {cx, cy + 1.0}};
  const auto separated = [](double min_a, double max_a, double min_b, double max_b) {
      return max_a < min_b || max_b < min_a;
    };
  const auto project = [](const auto & points, double ax, double ay, double & min_v,
    double & max_v) {
      min_v = std::numeric_limits<double>::infinity();
      max_v = -min_v;
      for (const auto & p : points) {
        const double v = p.first * ax + p.second * ay;
        min_v = std::min(min_v, v);
        max_v = std::max(max_v, v);
      }
    };

  double pminx, pmaxx, pminy, pmaxy;
  project(polygon, 1.0, 0.0, pminx, pmaxx);
  project(polygon, 0.0, 1.0, pminy, pmaxy);
  if (separated(pminx, pmaxx, cx, cx + 1.0) ||
    separated(pminy, pmaxy, cy, cy + 1.0))
  {
    return false;
  }

  // For convex polygons, testing every edge normal is sufficient.
  for (std::size_t i = 0; i < polygon.size(); ++i) {
    const auto & a = polygon[i];
    const auto & b = polygon[(i + 1) % polygon.size()];
    const double ax = -(b.second - a.second);
    const double ay = b.first - a.first;
    if (std::abs(ax) + std::abs(ay) <= 1e-15) {continue;}
    double fmin, fmax, cmin, cmax;
    project(polygon, ax, ay, fmin, fmax);
    project(cell, ax, ay, cmin, cmax);
    if (separated(fmin, fmax, cmin, cmax)) {return false;}
  }
  return true;
}
}  // namespace detail

/// Check the oriented footprint at a pose. Coordinates are cell units.
/// The footprint is rejected if it leaves the grid or touches a blocked cell.
inline bool footprint_pose_is_free(
  double x, double y, double yaw,
  uint32_t width, uint32_t height, const CellFreeFn & free,
  const Footprint & footprint)
{
  if (!free || width == 0 || height == 0 ||
    footprint.vertices.size() < 3 || !std::isfinite(x) || !std::isfinite(y) ||
    !std::isfinite(yaw))
  {
    return false;
  }
  const double c = std::cos(yaw), s = std::sin(yaw);
  std::vector<std::pair<double, double>> world;
  world.reserve(footprint.vertices.size());
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = min_x, max_x = -min_x, max_y = -min_x;
  for (const auto & v : footprint.vertices) {
    if (!std::isfinite(v.first) || !std::isfinite(v.second)) {return false;}
    const double wx = x + c * v.first - s * v.second;
    const double wy = y + s * v.first + c * v.second;
    world.emplace_back(wx, wy);
    min_x = std::min(min_x, wx);
    max_x = std::max(max_x, wx);
    min_y = std::min(min_y, wy);
    max_y = std::max(max_y, wy);
  }
  // Reject a footprint that touches the map boundary; outside is unknown.
  if (min_x <= 0.0 || min_y <= 0.0 ||
    max_x >= static_cast<double>(width) || max_y >= static_cast<double>(height))
  {
    return false;
  }
  const int64_t ix0 = static_cast<int64_t>(std::floor(min_x));
  const int64_t iy0 = static_cast<int64_t>(std::floor(min_y));
  const int64_t ix1 = static_cast<int64_t>(std::floor(max_x));
  const int64_t iy1 = static_cast<int64_t>(std::floor(max_y));
  for (int64_t cy = iy0; cy <= iy1; ++cy) {
    for (int64_t cx = ix0; cx <= ix1; ++cx) {
      if (cx < 0 || cy < 0 || cx >= static_cast<int64_t>(width) ||
        cy >= static_cast<int64_t>(height) ||
        detail::convex_polygon_intersects_cell(
          world, static_cast<double>(cx), static_cast<double>(cy)))
      {
        if (cx < 0 || cy < 0 || cx >= static_cast<int64_t>(width) ||
          cy >= static_cast<int64_t>(height))
        {
          return false;
        }
        if (!free(static_cast<uint32_t>(cx), static_cast<uint32_t>(cy))) {
          return false;
        }
      }
    }
  }
  return true;
}

/// Sweep a footprint over a straight segment (cell units). The footprint yaw is
/// the segment direction; zero-length segments use the caller-provided yaw.
inline bool footprint_segment_is_free(
  double x0, double y0, double x1, double y1, double yaw,
  uint32_t width, uint32_t height, const CellFreeFn & free,
  const Footprint & footprint)
{
  if (!std::isfinite(x0) || !std::isfinite(y0) ||
    !std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(yaw))
  {
    return false;
  }
  const double dx = x1 - x0, dy = y1 - y0;
  const double length = std::hypot(dx, dy);
  const double step = footprint.sample_step > 1e-9 ? footprint.sample_step : 0.25;
  const int64_t samples = std::max<int64_t>(
    1, static_cast<int64_t>(std::ceil(length / step)));
  const double movement_yaw = length > 1e-12 ? std::atan2(dy, dx) : yaw;
  for (int64_t i = 0; i <= samples; ++i) {
    const double t = samples == 0 ? 1.0 :
      static_cast<double>(i) / static_cast<double>(samples);
    if (!footprint_pose_is_free(
        x0 + dx * t, y0 + dy * t, movement_yaw, width, height, free, footprint))
    {
      return false;
    }
  }
  return true;
}
}  // namespace ldc_navigation
