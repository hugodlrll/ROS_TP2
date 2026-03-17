/**
 * @file controller.cpp
 * @author Bastien LAVAUX
 * @brief Control the player 
 * @version 0.1
 * @date 2026-03-17
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "handball_msgs/msg/player_control.hpp"
#include "handball_msgs/msg/planar_twist.hpp"
#include "handball_msgs/msg/ball_velocity.hpp"

using std::placeholders::_1;

class ControllerLavaux : public rclcpp::Node
{
public:
    ControllerLavaux()
    : Node("controller_lavaux")
    {
        // Publisher: sends PlayerControl messages to player node
        player_control_pub_ = this->create_publisher<handball_msgs::msg::PlayerControl>(
            "player_control", 10);

        // Subscriber to /joy
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&ControllerLavaux::joyCallback, this, _1)
        );

        // Timer (debug)
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ControllerLavaux::timerCallback, this)
        );

        RCLCPP_INFO(this->get_logger(), "Controller node started.");
    }

private:

    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        handball_msgs::msg::PlayerControl control_msg;

        bool moving = false;
        bool kicking = false;

        // Joystick axes (left stick)
        float x = msg->axes[0];   // left/right
        float y = msg->axes[1];   // forward/backward
        float rot = msg->axes[3]; // rotation (right stick)

        // If joystick moved significantly → MOVE action
        if (std::fabs(x) > 0.1 || std::fabs(y) > 0.1 || std::fabs(rot) > 0.1)
        {
            moving = true;
            control_msg.action |= handball_msgs::msg::PlayerControl::MOVE;

            control_msg.twist.vx = y * 2.0;      // Forward/backwards velocity
            control_msg.twist.vy = x * 2.0;      // Left/right velocity
            control_msg.twist.omega = rot * 1.5; // Angular velocity
        }

        // Button A (button[0]) → KICK action
        if (msg->buttons[0] == 1)
        {
            kicking = true;
            control_msg.action |= handball_msgs::msg::PlayerControl::KICK;

            control_msg.vel.vx = 5.0;      // shoot force
            control_msg.vel.vy = 0.0;      // forward
        }

        // If no action → NONE
        if (!moving && !kicking)
        {
            control_msg.action = handball_msgs::msg::PlayerControl::NONE;
        }

        player_control_pub_->publish(control_msg);

        RCLCPP_INFO(this->get_logger(),
            "Published PlayerControl: action=%u (MOVE=%d, KICK=%d)",
            control_msg.action, moving, kicking);

        /*RCLCPP_INFO(this->get_logger(),
            "AXES: [%f, %f, %f, %f] BUTTONS: [%d, %d, %d, %d]",
            msg->axes[0], msg->axes[1], msg->axes[2], msg->axes[3],
            msg->buttons[0], msg->buttons[1], msg->buttons[2], msg->buttons[3]
        );*/
    }

    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "ControllerLavaux running...");
    }

    rclcpp::Publisher<handball_msgs::msg::PlayerControl>::SharedPtr player_control_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerLavaux>());
    rclcpp::shutdown();
    return 0;
}
