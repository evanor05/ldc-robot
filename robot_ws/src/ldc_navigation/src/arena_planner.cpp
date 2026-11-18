// ArenaPlanner —— 自研 A* 全局规划器（nav2_core::GlobalPlanner 插件）。
//
// 职责边界（刻意保持"薄"）：
//   - 本文件只做 ROS 数据搬运：costmap 取数 → 纯算法 → Path 消息；
//   - 全部算法逻辑在 include/ldc_navigation/grid_a_star.hpp
//     与 path_smoother.hpp 里，纯 C++、主机可单测；
//   - 失败一律抛 nav2_core 具体异常（映射到 action error_code），
//     绝不静默返回空路径。
#include "ldc_navigation/arena_planner.hpp"

#include "nav2_core/planner_exceptions.hpp"

#include <chrono>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace ldc_navigation
{

namespace
{
constexpr int kUnknownCost = -1;
constexpr double kDefaultTfTimeoutS = 0.5;
}  // namespace

void ArenaPlanner::configure(
  const nav2::LifecycleNode::WeakPtr & parent,
  std::string name,
  nav2::TransformBuffer::SharedPtr tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  plugin_name_ = name;
  tf_ = tf;
  costmap_ros_ = costmap_ros;
  costmap_ = costmap_ros_->getCostmap();

  auto node = parent.lock();
  if (!node) {
    throw nav2_core::PlannerException("ArenaPlanner: parent lifecycle node expired");
  }
  logger_ = node->get_logger();
  // 参数都挂在 <plugin_name>. 前缀下（与 NavFn 相同的惯例）
  weight_cost_ = node->declare_or_get_parameter(plugin_name_ + ".weight_cost", 0.5);
  allow_unknown_ = node->declare_or_get_parameter(plugin_name_ + ".allow_unknown", false);
  use_smoother_ = node->declare_or_get_parameter(plugin_name_ + ".use_smoother", true);
  const auto limit = node->declare_or_get_parameter(plugin_name_ + ".max_expand", 200000);
  if (limit <= 0) {throw nav2_core::PlannerException("max_expand must be positive");}
  max_expand_ = static_cast<std::size_t>(limit);

  if (!std::isfinite(weight_cost_) || weight_cost_ < 0.0 || weight_cost_ > 10.0) {
    throw nav2_core::PlannerException(
            "ArenaPlanner: weight_cost 必须在 [0, 10]，当前 = " +
            std::to_string(weight_cost_));
  }

  RCLCPP_INFO(
    node->get_logger(),
    "ArenaPlanner configured: weight_cost=%.2f allow_unknown=%s smoother=%s",
    weight_cost_, allow_unknown_ ? "true" : "false", use_smoother_ ? "true" : "false");
}

void ArenaPlanner::cleanup()
{
  // costmap_ 是 Costmap2DROS 托管的裸指针，不拥有所有权，只清引用
  costmap_ = nullptr;
  costmap_ros_.reset();
}

void ArenaPlanner::activate() {}
void ArenaPlanner::deactivate() {}

nav_msgs::msg::Path ArenaPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  const std::vector<geometry_msgs::msg::PoseStamped> & /*viapoints*/,
  std::function<bool()> cancel_checker)
{
  const auto begin = std::chrono::steady_clock::now();
  const auto check_cancel = [&]() {
      if (cancel_checker && cancel_checker()) {
        throw nav2_core::PlannerCancelled("ArenaPlanner: cancelled");
      }
    };
  check_cancel();
  nav_msgs::msg::Path path;
  path.header.stamp = start.header.stamp;
  path.header.frame_id = costmap_ros_->getGlobalFrameID();

  // 起终点统一到全局坐标系（planner_server 通常已保证；防御性转换）
  const auto to_global = [&](geometry_msgs::msg::PoseStamped p) {
      if (p.header.frame_id == path.header.frame_id || p.header.frame_id.empty()) {
        return p;
      }
      geometry_msgs::msg::TransformStamped t;
      try {
        t = tf_->lookupTransform(
          path.header.frame_id, p.header.frame_id, p.header.stamp,
          tf2::durationFromSec(kDefaultTfTimeoutS));
      } catch (const tf2::TransformException & ex) {
        throw nav2_core::PlannerTFError(
                std::string("ArenaPlanner: TF 失败: ") + ex.what());
      }
      geometry_msgs::msg::PoseStamped out;
      tf2::doTransform(p, out, t);
      return out;
    };
  const geometry_msgs::msg::PoseStamped gs = to_global(start);
  const geometry_msgs::msg::PoseStamped gg = to_global(goal);

  for (const auto * p : {&gs, &gg}) {
    const auto & q = p->pose.orientation;
    const double norm = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    if (!std::isfinite(p->pose.position.x) || !std::isfinite(p->pose.position.y) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    {
      throw nav2_core::PlannerException("ArenaPlanner: invalid pose/quaternion");
    }
  }
  // Copy geometry and cells while holding the same lock used by Costmap writers.
  nav2_costmap_2d::Costmap2D snapshot;
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());
    snapshot = *costmap_;
  }
  check_cancel();
  const auto to_cell = [&](const auto & p, unsigned int & x, unsigned int & y, bool start) {
      if (!snapshot.worldToMap(p.pose.position.x, p.pose.position.y, x, y)) {
        if (start) {throw nav2_core::StartOutsideMapBounds("ArenaPlanner: start outside map");}
        throw nav2_core::GoalOutsideMapBounds("ArenaPlanner: goal outside map");
      }
    };
  unsigned int sx, sy, gx, gy;
  to_cell(gs, sx, sy, true);
  to_cell(gg, gx, gy, false);
  const auto cell_cost_raw = [&](uint32_t x, uint32_t y) {return snapshot.getCost(x, y);};

  // 起终点致命性检查：出具体异常而不是让搜索悄悄失败。
  // 注意 255 = NO_INFORMATION（未知）不是致命障碍，单独区分，
  // 否则 action 层会拿到误导性的 GOAL_OCCUPIED 错误码。
  const auto lethal = [](unsigned char c) {
      return c >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE &&
             c != nav2_costmap_2d::NO_INFORMATION;
    };
  const auto no_info = [](unsigned char c) {
      return c == nav2_costmap_2d::NO_INFORMATION;
    };
  if (lethal(cell_cost_raw(sx, sy))) {
    throw nav2_core::StartOccupied("ArenaPlanner: start 落在致命/内切障碍栅格");
  }
  if (lethal(cell_cost_raw(gx, gy))) {
    throw nav2_core::GoalOccupied("ArenaPlanner: goal 落在致命/内切障碍栅格");
  }
  if (!allow_unknown_ &&
    (no_info(cell_cost_raw(sx, sy)) || no_info(cell_cost_raw(gx, gy))))
  {
    throw nav2_core::NoValidPathCouldBeFound(
      "ArenaPlanner: start/goal 落在未知区域且 allow_unknown=false");
  }

  GridAStarConfig cfg;
  cfg.allow_unknown = allow_unknown_;
  cfg.weight_cost = weight_cost_;
  cfg.max_expand = max_expand_;
  const CellCostFn cell_cost = [&](uint32_t x, uint32_t y) -> int16_t {
      const auto raw = snapshot.getCost(x, y);
      return raw == 255 ? kUnknownCost : static_cast<int16_t>(raw);
    };
  const CellFreeFn cell_free = [&](uint32_t x, uint32_t y) {
      const auto raw = snapshot.getCost(x, y);
      return !lethal(raw) && (allow_unknown_ || raw != 255);
    };
  // Copy the costmap-provided footprint once per plan. When the costmap is
  // configured with only robot_radius, approximate that radius as a 16-gon so
  // the same orientation-dependent check still covers the configured body.
  Footprint footprint;
  footprint.sample_step = 0.5;  // cells
  const double snapshot_resolution = snapshot.getResolution();
  if (snapshot_resolution > 0.0) {
    // Use the raw footprint; the costmap's padded footprint can be larger than
    // the inflation layer's inscribed radius and reject otherwise safe paths.
    const auto robot_footprint = costmap_ros_->getUnpaddedRobotFootprint();
    for (const auto & point : robot_footprint) {
      footprint.vertices.emplace_back(
        point.x / snapshot_resolution, point.y / snapshot_resolution);
    }
    if (footprint.vertices.size() < 3) {
      const double radius = costmap_ros_->getRobotRadius();
      if (radius > 0.0) {
        constexpr int kCirclePoints = 16;
        for (int i = 0; i < kCirclePoints; ++i) {
          const double angle = 2.0 * M_PI * static_cast<double>(i) / kCirclePoints;
          footprint.vertices.emplace_back(
            radius * std::cos(angle) / snapshot_resolution,
            radius * std::sin(angle) / snapshot_resolution);
        }
      }
    }
  }
  const bool footprint_enabled = footprint.vertices.size() >= 3;
  const double snapshot_origin_x = snapshot.getOriginX();
  const double snapshot_origin_y = snapshot.getOriginY();
  const uint32_t snapshot_width = static_cast<uint32_t>(snapshot.getSizeInCellsX());
  const uint32_t snapshot_height = static_cast<uint32_t>(snapshot.getSizeInCellsY());
  const auto to_cell_x = [&](double wx) {return (wx - snapshot_origin_x) / snapshot_resolution;};
  const auto to_cell_y = [&](double wy) {return (wy - snapshot_origin_y) / snapshot_resolution;};
  const SegmentFreeFn centerline_segment_free = [&](double x0, double y0, double x1, double y1) {
      return grid_segment_is_free(
        to_cell_x(x0), to_cell_y(y0), to_cell_x(x1), to_cell_y(y1),
        snapshot_width, snapshot_height, cell_free);
    };
  // Validation uses the footprint-aware sweep; smoothing uses the cheaper
  // centerline predicate plus pose_free at each candidate vertex. The raw path
  // and the final smoothed path are both footprint-validated below.
  const SegmentFreeFn segment_free = [&](double x0, double y0, double x1, double y1) {
      if (!footprint_enabled) {return centerline_segment_free(x0, y0, x1, y1);}
      return footprint_segment_is_free(
        to_cell_x(x0), to_cell_y(y0), to_cell_x(x1), to_cell_y(y1),
        std::atan2(y1 - y0, x1 - x0), snapshot_width, snapshot_height,
        cell_free, footprint);
    };
  const PoseFreeFn pose_free = [&](double wx, double wy, double yaw) {
      if (!footprint_enabled) {return true;}
      return footprint_pose_is_free(
        to_cell_x(wx), to_cell_y(wy), yaw, snapshot_width, snapshot_height,
        cell_free, footprint);
    };
  const auto result = search_grid_a_star(
    sx, sy, gx, gy,
    snapshot_width, snapshot_height,
    cell_cost, cfg, cancel_checker);

  if (result.cancelled) {
    throw nav2_core::PlannerCancelled("ArenaPlanner: 搜索被取消");
  }
  if (!result.success || result.path.empty()) {
    throw nav2_core::NoValidPathCouldBeFound(
            "ArenaPlanner: 未找到合法路径 (expanded=" +
            std::to_string(result.expanded) + ")");
  }

  // ---- 栅格 → 世界坐标 ----
  std::vector<std::pair<double, double>> world_path;
  world_path.reserve(result.path.size());
  for (const auto & [cx, cy] : result.path) {
    double wx, wy;
    snapshot.mapToWorld(cx, cy, wx, wy);
    world_path.emplace_back(wx, wy);
  }

  // A same-cell turn must not take a detour through the cell center.
  if (sx == gx && sy == gy) {world_path.clear();}
  // Exact endpoints connect to their cell centers; these connectors are checked too.
  world_path.insert(world_path.begin(), {gs.pose.position.x, gs.pose.position.y});
  world_path.emplace_back(gg.pose.position.x, gg.pose.position.y);
  const auto validate_path = [&](const auto & points) {
      for (std::size_t i = 1; i < points.size(); ++i) {
        check_cancel();
        if (!segment_free(points[i - 1].first, points[i - 1].second,
          points[i].first, points[i].second))
        {
          throw nav2_core::NoValidPathCouldBeFound("ArenaPlanner: invalid path segment");
        }
      }
      if (!footprint_enabled) {return;}
      // A path vertex can be a turn; check the body under both adjacent
      // headings, not only along the straight segment.
      for (std::size_t i = 0; i < points.size(); ++i) {
        check_cancel();
        double in_yaw = 0.0, out_yaw = 0.0;
        if (i + 1 < points.size()) {
          out_yaw = std::atan2(
            points[i + 1].second - points[i].second,
            points[i + 1].first - points[i].first);
        }
        if (i > 0) {
          in_yaw = std::atan2(
            points[i].second - points[i - 1].second,
            points[i].first - points[i - 1].first);
        }
        if (i == 0) {in_yaw = out_yaw;}
        if (i + 1 == points.size()) {out_yaw = in_yaw;}
        if (!pose_free(points[i].first, points[i].second, in_yaw) ||
          !pose_free(points[i].first, points[i].second, out_yaw))
        {
          throw nav2_core::NoValidPathCouldBeFound(
                  "ArenaPlanner: invalid footprint at path vertex");
        }
      }
    };
  validate_path(world_path);
  if (use_smoother_ && world_path.size() > 2) {
    try {
      auto smoothed = smooth_path(
        world_path, centerline_segment_free, {}, check_cancel, pose_free);
      validate_path(smoothed);
      world_path = std::move(smoothed);
    } catch (const std::invalid_argument & ex) {
      RCLCPP_WARN(
        logger_, "ArenaPlanner: smoother rejected input (%s); using validated raw path",
        ex.what());
    } catch (const nav2_core::NoValidPathCouldBeFound & ex) {
      RCLCPP_WARN(
        logger_, "ArenaPlanner: smoothed path rejected (%s); using validated raw path",
        ex.what());
    }
  }
  validate_path(world_path);

  path.poses.reserve(world_path.size());
  for (const auto & [wx, wy] : world_path) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }

  for (std::size_t i = 0; i + 1 < path.poses.size(); ++i) {
    const auto & a = path.poses[i].pose.position;
    const auto & b = path.poses[i + 1].pose.position;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, std::atan2(b.y - a.y, b.x - a.x));
    path.poses[i].pose.orientation = tf2::toMsg(q);
  }
  path.poses.back().pose = gg.pose;
  check_cancel();
  const auto end = std::chrono::steady_clock::now();
  RCLCPP_DEBUG(
    rclcpp::get_logger("ArenaPlanner"),
    "plan: %zu poses, expanded=%zu, %.1f ms",
    path.poses.size(), result.expanded,
    std::chrono::duration<double, std::milli>(end - begin).count());
  return path;
}

}  // namespace ldc_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ldc_navigation::ArenaPlanner, nav2_core::GlobalPlanner)
