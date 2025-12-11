/**
 * @class coach node
 * @brief Simulates the behavior of a handball coach
 *
 * ### Description
 * The coach node simulates a handball coach that monitors the positions of players and give them some advices.
 *
 * ### Parameters
 *
 * ### Subscribed Topics
 * 
 * ### Published Topics
 *
 * ### Example Usage
 * @code{.bash}
 * ros2 run handball coach
 * @endcode
 *
 * @note
 *
 * @author
 *   John Dow
 *
 * @date
 *   2025-12-11
 *
 * @version
 *   0.1
 *
 * @license
 *   Apache 2.0
 */

#include <random>
#include <functional>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "handball"

using std::placeholders::_1;

class Coach : public rclcpp::Node
{
public:
  Coach() : Node("coach"),
    
  {
    // Handle parameters.
    // This code is for parameters that are defined at startup and remain constant during
    // execution. If changes during execution is necessary, see the ros2 Galactic tutorial.

    {

    }

    // Create subscriptions, publishers, tf broadcaster and listeners... High overhead objects.

    {
		
	
    // Create subscription to the position of the virtual leader
    leader_position_received_ = false ;
    auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
    leader_pos_sub_ = this->create_subscription<swarm_msgs::msg::BirdState>(
        leader_topic,
        qos_settings,
        std::bind(&SwarmNode::leader_pos_callback, this, _1)
    );

    // Create publisher for the message id+color+position used by swarm_viz to
    // generate visualization messages.
    state_pub_ = this->create_publisher<swarm_msgs::msg::BirdState>("/all_states", qos_settings);
    }

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SwarmNode::timer_callback, this));

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Started with : "
               << "color = " << color_ << ", "
               << "id = " << id_);
  }

private:

  double random_double(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
  }

  void leader_pos_callback(const swarm_msgs::msg::BirdState::SharedPtr msg)
  {
    leader_position_ = *msg;
    leader_position_received_ = true;
    // Define random goal position for this bird at polar coordinates (r,theta)
    // from virtual leader
    float r     = random_double(1,1.5) ;
    float theta = random_double(-M_PI,M_PI) ;
    goal_x_ = leader_position_.pos.x + r*cos(theta) ;
    goal_y_ = leader_position_.pos.y + r*sin(theta) ;
    goal_z_ = 0.0 ;
  }
  

  swarm_msgs::msg::BirdState leader_position_ ;
  rclcpp::Subscription<swarm_msgs::msg::BirdState>::SharedPtr leader_pos_sub_;
  rclcpp::Publisher<swarm_msgs::msg::BirdState>::SharedPtr state_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Coach>());
  rclcpp::shutdown();
  return 0;
}

