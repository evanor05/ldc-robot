#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

// The URDF owns base_footprint -> base_link. Publish each measurement once,
// at its acquisition time, without inventing fresh timestamps for stale poses.
class OdomTransform : public rclcpp::Node
{
public:
  OdomTransform() : Node("ldc_bringup")
  {
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
    base_link_height_ = declare_parameter<double>("base_link_height", 0.06205);
    broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { on_odom(*msg); });
  }

private:
  void on_odom(const nav_msgs::msg::Odometry & msg)
  {
    const auto & p = msg.pose.pose.position;
    const auto & q = msg.pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (msg.header.frame_id != odom_frame_ ||
      (msg.child_frame_id != base_frame_ && msg.child_frame_id != "base_link") ||
      !std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || norm < 1e-12 ||
      (msg.header.stamp.sec == 0 && msg.header.stamp.nanosec == 0))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Ignoring odometry with invalid pose, timestamp or frame IDs");
      return;
    }
    tf2::Quaternion rotation(q.x, q.y, q.z, q.w);
    rotation.normalize();
    tf2::Vector3 position(p.x, p.y, p.z);
    if (msg.child_frame_id == "base_link" && base_frame_ != "base_link") {
      // T_odom_footprint = T_odom_link * inverse(T_footprint_link).
      position += tf2::quatRotate(rotation, tf2::Vector3(0, 0, -base_link_height_));
    }
    geometry_msgs::msg::TransformStamped transform;
    transform.header = msg.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    transform.transform.rotation.w = rotation.w();
    broadcaster_->sendTransform(transform);
  }

  std::string odom_frame_, base_frame_;
  double base_link_height_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomTransform>());
  rclcpp::shutdown();
  return 0;
}
