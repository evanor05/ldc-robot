// Exercise the actual ROS plugin wrapper with an inactive, sensor-free costmap.
#include "ldc_navigation/arena_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
void require(bool value, const std::string & message)
{
  if (!value) {throw std::runtime_error(message);}
}

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw = 0.0)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

struct Fixture
{
  std::shared_ptr<nav2::LifecycleNode> node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> ros_map;
  ldc_navigation::ArenaPlanner planner;

  Fixture()
  {
    node = std::make_shared<nav2::LifecycleNode>("arena_planner_test");
    rclcpp::NodeOptions options;
    options.parameter_overrides({
        rclcpp::Parameter("plugins", std::vector<std::string>{}),
        rclcpp::Parameter("filters", std::vector<std::string>{}),
        rclcpp::Parameter("global_frame", "map")});
    ros_map = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "arena_test_costmap", "/", false, options);
    ros_map->configure();
    auto * map = ros_map->getCostmap();
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
    map->resizeMap(30, 30, 0.05, 0.0, 0.0);
    map->resetMapToValue(0, 0, 30, 30, nav2_costmap_2d::FREE_SPACE);
    planner.configure(node, "GridBased", ros_map->getTfBuffer(), ros_map);
  }

  ~Fixture()
  {
    planner.cleanup();
    ros_map->cleanup();
  }
};

void check_endpoint(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::PoseStamped & goal)
{
  require(!path.poses.empty(), "successful plan must not be empty");
  const auto & last = path.poses.back();
  require(std::hypot(last.pose.position.x - goal.pose.position.x,
    last.pose.position.y - goal.pose.position.y) < 1e-12, "goal position snapped to cell center");
  require(std::abs(last.pose.orientation.z - goal.pose.orientation.z) < 1e-12 &&
    std::abs(last.pose.orientation.w - goal.pose.orientation.w) < 1e-12,
    "goal orientation lost");
  for (const auto & p : path.poses) {
    require(p.header.frame_id == "map", "path pose frame differs from path frame");
    require(std::abs(std::hypot(p.pose.orientation.z, p.pose.orientation.w) - 1.0) < 1e-9,
      "path orientation is not normalized");
  }
}

void run(const std::string & name)
{
  Fixture f;
  const auto start = pose(0.212, 0.218);
  const auto goal = pose(1.213, 1.217, 1.5707963267948966);
  if (name == "endpoint") {
    check_endpoint(f.planner.createPlan(start, goal, {}, {}), goal);
  } else if (name == "same_cell") {
    const auto nearby = pose(0.239, 0.231, -1.2);
    check_endpoint(f.planner.createPlan(start, nearby, {}, {}), nearby);
    const auto rotation = pose(start.pose.position.x, start.pose.position.y, 2.0);
    const auto turn = f.planner.createPlan(start, rotation, {}, {});
    check_endpoint(turn, rotation);
    for (const auto & p : turn.poses) {
      require(p.pose.position.x == start.pose.position.x &&
        p.pose.position.y == start.pose.position.y, "in-place turn contains a translation");
    }
  } else if (name == "shared_lock") {
    auto * map = f.ros_map->getCostmap();
    std::promise<void> started;
    std::future<nav_msgs::msg::Path> result;
    bool blocked;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
      result = std::async(std::launch::async, [&]() {
            started.set_value();
            return f.planner.createPlan(start, goal, {}, {});
      });
      started.get_future().wait();
      blocked = result.wait_for(std::chrono::milliseconds(150)) == std::future_status::timeout;
      map->setCost(24, 24, nav2_costmap_2d::LETHAL_OBSTACLE);
    }
    bool occupied = false;
    try {
      result.get();
    } catch (const nav2_core::GoalOccupied &) {
      occupied = true;
    }
    require(blocked && occupied, "planner did not share the costmap writer lock");
  } else if (name == "snapshot") {
    int checks = 0;
    const auto path = f.planner.createPlan(start, goal, {}, [&]() {
          if (++checks == 3) {
            auto * map = f.ros_map->getCostmap();
            std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*map->getMutex());
            map->resizeMap(2, 2, 0.1, 100.0, 100.0);
          }
          return false;
    });
    require(checks >= 3, "snapshot mutation was not exercised");
    check_endpoint(path, goal);
    for (const auto & p : path.poses) {
      require(p.pose.position.x < 1.5 && p.pose.position.y < 1.5,
        "path mixed old map data with new origin");
    }
  } else if (name == "cancel") {
    bool caught = false;
    try {
      f.planner.createPlan(start, start, {}, []() {
          return true;
                                                                  });
    } catch (const nav2_core::PlannerCancelled &) {
      caught = true;
    }
    require(caught, "same-cell goal ignored cancellation");
  } else if (name == "invalid_pose") {
    auto invalid = goal;
    invalid.pose.position.x = std::numeric_limits<double>::quiet_NaN();
    bool caught = false;
    try {
      f.planner.createPlan(start, invalid, {}, {});
    } catch (const nav2_core::PlannerException &) {
      caught = true;
    }
    require(caught, "non-finite target accepted");
    invalid = goal;
    invalid.pose.orientation.z = invalid.pose.orientation.w = 0.0;
    caught = false;
    try {
      f.planner.createPlan(start, invalid, {}, {});
    } catch (const nav2_core::PlannerException &) {
      caught = true;
    }
    require(caught, "zero quaternion accepted");
  } else if (name == "reconfigure") {
    f.planner.cleanup();
    f.planner.configure(f.node, "GridBased", f.ros_map->getTfBuffer(), f.ros_map);
    check_endpoint(f.planner.createPlan(start, goal, {}, {}), goal);
  } else {
    throw std::runtime_error("unknown test case: " + name);
  }
}
}  // namespace

int main(int argc, char ** argv)
{
  const std::string name = argc == 2 ? argv[1] : "endpoint";
  rclcpp::init(0, nullptr);
  int code = 0;
  try {
    run(name); std::cout << "PASS " << name << '\n';
  } catch (const std::exception & e) {
    std::cerr << "FAIL " << name << ": " << e.what() << '\n'; code = 1;
  }
  rclcpp::shutdown();
  return code;
}
