/**
 * @file ball.cpp
 * @author Tommy ROYER
 * @brief Simulate the behavior of a handball ball
 * @version 0.1
 * @date 2026-03-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <rclcpp/rclcpp.hpp>

#include "handball_msgs/msg/ball_state.hpp"
#include "handball_msgs/msg/ball_velocity.hpp"
#include "std_msgs/msg/empty.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;

class BallNode : public rclcpp::Node
{
public:
  BallNode() : Node("ball")
  {
    RCLCPP_INFO(this->get_logger(), "Ball node started");

    /* ===== Publisher ===== */
    state_pub_ = this->create_publisher<handball_msgs::msg::BallState>(
      "/ball_state", 10);

    /* ===== Subscriptions ===== */
    kick_sub_ = this->create_subscription<handball_msgs::msg::BallVelocity>(
      "/ball_kick", 10,
      std::bind(&BallNode::kickCallback, this, _1));

    reset_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/ball_reset", 10,
      std::bind(&BallNode::resetCallback, this, _1));

    attach_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "/ball_attach", 10,
      std::bind(&BallNode::attachCallback, this, _1));

    /* ===== Timer ===== */
    timer_ = this->create_wall_timer(
      100ms, std::bind(&BallNode::update, this));

    /* ===== TF broadcaster ===== */
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  /* ===== Callbacks ===== */

  void kickCallback(const handball_msgs::msg::BallVelocity::SharedPtr msg)
  {
    vx_ = msg->vx;
    vy_ = msg->vy;
  }

  void resetCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    x_ = 0.0;
    y_ = 0.0;
    vx_ = 0.0;
    vy_ = 0.0;
  }

  void attachCallback(const std_msgs::msg::Empty::SharedPtr)
  {
    vx_ = 0.0;
    vy_ = 0.0;
  }

  /* ===== Mise à jour périodique ===== */
  void update()
  {
    constexpr double dt = 0.1;  // 100 ms

    /* Intégration simple */
    x_ += vx_ * dt;
    y_ += vy_ * dt;

    /* ===== Rebonds sur les limites du terrain ===== */
    if (x_ < X_MIN) {
      x_ = X_MIN;
      vx_ = -vx_;
    } else if (x_ > X_MAX) {
      x_ = X_MAX;
      vx_ = -vx_;
    }

    if (y_ < Y_MIN) {
      y_ = Y_MIN;
      vy_ = -vy_;
    } else if (y_ > Y_MAX) {
      y_ = Y_MAX;
      vy_ = -vy_;
    }

    /* ===== Publication de l'état ===== */
    handball_msgs::msg::BallState state;
    state.pos.x = x_;
    state.pos.y = y_;
    state.vel.vx = vx_;
    state.vel.vy = vy_;

    state_pub_->publish(state);

    /* ===== Publication TF ===== */
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "ball";

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0.0;

    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
  }

  /* ===== État interne ===== */
  double x_ = 0.0;
  double y_ = 0.0;
  double vx_ = 0.0;
  double vy_ = 0.0;

  /* ===== Dimensions du terrain ===== */
  static constexpr double X_MIN = -20.0;
  static constexpr double X_MAX =  20.0;
  static constexpr double Y_MIN = -10.0;
  static constexpr double Y_MAX =  10.0;

  /* ===== ROS interfaces ===== */
  rclcpp::Publisher<handball_msgs::msg::BallState>::SharedPtr state_pub_;
  rclcpp::Subscription<handball_msgs::msg::BallVelocity>::SharedPtr kick_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr attach_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr reset_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BallNode>());
  rclcpp::shutdown();
  return 0;
}
