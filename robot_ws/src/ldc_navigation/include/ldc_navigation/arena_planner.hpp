#pragma once

#include <memory>
#include <mutex>
#include <string>

#include <tf2/exceptions.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ldc_navigation/grid_a_star.hpp"
#include "ldc_navigation/path_smoother.hpp"

namespace ldc_navigation
{

/// A* 全局规划器插件：算法在 grid_a_star.hpp（纯 C++），本类只做数据搬运。
class ArenaPlanner : public nav2_core::GlobalPlanner
{
public:
  ArenaPlanner() = default;
  ~ArenaPlanner() override = default;

  void configure(
    const nav2::LifecycleNode::WeakPtr & parent, std::string name,
    nav2::TransformBuffer::SharedPtr tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
  void cleanup() override;
  void activate() override;
  void deactivate() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    const std::vector<geometry_msgs::msg::PoseStamped> & viapoints,
    std::function<bool()> cancel_checker) override;

private:
  std::string plugin_name_;
  nav2::TransformBuffer::SharedPtr tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  rclcpp::Logger logger_{rclcpp::get_logger("ArenaPlanner")};

  double weight_cost_{0.5};
  bool allow_unknown_{false};
  bool use_smoother_{true};
  std::size_t max_expand_{200000};

};

}  // namespace ldc_navigation
