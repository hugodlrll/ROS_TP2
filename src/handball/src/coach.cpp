/**
 * @class coach node
 * @brief Simulates the behavior of a handball coach
 *
 * ### Description
 * The coach node simulates a handball coach that monitors the positions of players and give them some advices.
 */

#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "handball_msgs/msg/ball_state.hpp"
#include "handball_msgs/msg/player_state.hpp"
#include "handball_msgs/msg/coach_instruction.hpp"

using std::placeholders::_1;

class Coach : public rclcpp::Node
{
public:
  Coach()
  : Node("coach")
  {
    auto qos = rclcpp::QoS(10);

    // Subscriptions
    ball_sub_ = this->create_subscription<handball_msgs::msg::BallState>(
      "ball_state",
      qos,
      std::bind(&Coach::ball_callback, this, _1)
    );

    player_sub_ = this->create_subscription<handball_msgs::msg::PlayerState>(
      "player_state",
      qos,
      std::bind(&Coach::player_callback, this, _1)
    );

    // Publisher
    instr_pub_ = this->create_publisher<handball_msgs::msg::CoachInstruction>(
      "coach_instruction",
      qos
    );

    RCLCPP_INFO(this->get_logger(), "Coach node started: subscribing to 'ball_state' and 'player_state', publishing on 'coach_instruction'");
  }

private:
  void ball_callback(const handball_msgs::msg::BallState::SharedPtr msg)
  {
    // Very simple logic: if ball speed is high -> ask players to defend, else ask to keep formation
    double speed = std::hypot(msg->vel.vx, msg->vel.vy);
    handball_msgs::msg::CoachInstruction inst;
    if (speed > 2.0) {
      inst.instr = "DEFEND: ball moving fast";
    } else {
      inst.instr = "MAINTAIN: keep formation";
    }
    instr_pub_->publish(inst);

    RCLCPP_DEBUG(this->get_logger(), "Published coach instruction from ball_callback: %s", inst.instr.c_str());
  }

  void player_callback(const handball_msgs::msg::PlayerState::SharedPtr msg)
  {
    // If player's energy is low, tell them to rest; otherwise small encouragement message
    handball_msgs::msg::CoachInstruction inst;
    if (msg->energie < 20.0) {
      inst.instr = "REST: reduce intensity";
    } else {
      inst.instr = "KEEP_GOING: good energy";
    }
    instr_pub_->publish(inst);

    RCLCPP_DEBUG(this->get_logger(), "Published coach instruction from player_callback (id=%u): %s", msg->id, inst.instr.c_str());
  }

  rclcpp::Subscription<handball_msgs::msg::BallState>::SharedPtr ball_sub_;
  rclcpp::Subscription<handball_msgs::msg::PlayerState>::SharedPtr player_sub_;
  rclcpp::Publisher<handball_msgs::msg::CoachInstruction>::SharedPtr instr_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Coach>());
  rclcpp::shutdown();
  return 0;
}

