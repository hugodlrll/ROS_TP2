/**
 * @class PlayerNode
 * @brief Simulates a member of handball team.
 *
 * ### Description
 * To complete
 * 
 * ### Parameters
 * - @c id (uint8, default: 1)
 *   Each player has its id.
 * - @c team (string, "red"/"blue", default: "red")
 *   Each team has its color.
 * - @c energie (float64, default: 100.00)
 *   Energy left for the player.
 *
 * To complete
 * 
 * ### Subscribed Topics
 * - @c /ball_pos_vit (@c handball_msgs::msg::BallState)
 * 
 * - @c coach_instr (@c handball_msgs::msg::CoachInstruction)
 * 
 * - @c player_control (@c handball_msgs::msg::PlayerControl)
 *
 * - @c /whistle (@c handball_msgs::msg::Whistle)
 *
 * - @c team_states (@c handball_msgs::msg::PlayerState)
 *
 * ### Published Topics
 * - @c /player_state (@c handball_msgs::msg::PlayerState)
 * 
 * ### Example Usage
 * @code{.bash}
 * ros2 run handball player_node
 * @endcode
 *
 * @note
 *
 * @author
 *   Julien Depeyris (julien.depeyris@gmail.com)
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
#include <memory>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "handball_msgs/msg/ball_state.hpp"
#include "handball_msgs/msg/coach_instruction.hpp"
#include "handball_msgs/msg/player_control.hpp"
#include "handball_msgs/msg/whistle.hpp"
#include "handball_msgs/msg/player_state.hpp"
#include "handball_msgs/srv/substitution_execution.hpp"


//using std::placeholders::_1;
using namespace std::placeholders;

class PlayerNode : public rclcpp::Node
{
public:
  PlayerNode() : Node("player_node"),
                  ball_pos_vit_received_(false),
                  coach_instr_received_(false),
                  player_control_received_(false),
                  whistle_received_(false),
                  team_state_received_(false),
                  first_timer_call_(true)
  {

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Handle parameters.
    // This code is for parameters that are defined at startup and remain constant during
    // execution. If changes during execution is necessary, see the ros2 Galactic tutorial.

    {
	// Parameter "id"
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Strictly positive integer (default is 1)";
    this->declare_parameter("id", 1, param_desc);
    id_ = this->get_parameter("id").get_parameter_value().get<uint8_t>();

    // Parameter "color"
    param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Allowed values are red (default), blue (strings, case sensitive)";
    this->declare_parameter("team", "red", param_desc);
    team_ = this->get_parameter("team").get_parameter_value().get<std::string>();

	// Parameter "energie"
    param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Float (default is 100.00)";
    this->declare_parameter("energie", 100.0, param_desc);
    energie_ = this->get_parameter("energie").get_parameter_value().get<float>();
  
    /*// Parameter "kp". TBD: make it modifiable during runtime. Makes sense.
    param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = "Real > 0 and < 1 (default is 0.1)";
    this->declare_parameter("kp", 0.1, param_desc);
    kp_ = this->get_parameter("kp").get_parameter_value().get<float>();*/
    }

    // Create subscriptions, publishers, tf broadcaster and listeners... High overhead objects.

    {
    // Create subscriptions
    ball_pos_vit_received_ = false ;
    auto qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    ball_pos_vit_sub_ = this->create_subscription<handball_msgs::msg::BallState>(
        "/ball_state",
        qos_settings,
        std::bind(&PlayerNode::ball_pos_vit_callback, this, _1)
    );
    coach_instr_received_ = false ;
    qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    coach_instr_sub_ = this->create_subscription<handball_msgs::msg::CoachInstruction>(
        "coach_instruction",
        qos_settings,
        std::bind(&PlayerNode::coach_instr_callback, this, _1)
    );
    player_control_received_ = false ;
    qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    player_control_sub_ = this->create_subscription<handball_msgs::msg::PlayerControl>(
        "player_control",
        qos_settings,
        std::bind(&PlayerNode::player_control_callback, this, _1)
    );
    whistle_received_ = false ;//referee
    qos_settings = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    whistle_sub_ = this->create_subscription<handball_msgs::msg::Whistle>(
      "/whistle",
        qos_settings,
        std::bind(&PlayerNode::whistle_callback, this, _1)
    );
    team_state_received_ = false ;
    qos_settings = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();
    team_state_sub_ = this->create_subscription<handball_msgs::msg::PlayerState>(
        "team_state",
        qos_settings,
        std::bind(&PlayerNode::team_state_callback, this, _1)
    );

    // Create publisher for the message id+team+energy to
    // generate visualization messages.
    state_pub_ = this->create_publisher<handball_msgs::msg::PlayerState>("/player_state", qos_settings);
    }
    

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period),
      std::bind(&PlayerNode::timer_callback, this));

    RCLCPP_INFO_STREAM(
      this->get_logger(), "Started with : "
               << "team = " << team_ << ", "
               << "id = " << id_ << ", "
               << "energy = " << energie_
               );
  }

  void substitution(const std::shared_ptr<handball_msgs::srv::SubstitutionExecution::Request> request,
                   std::shared_ptr<handball_msgs::srv::SubstitutionExecution::Response> response)
  {
      if(request->outgoing == id_){
        energie_ = 100;
        response->set__status(response->DONE);
        RCLCPP_INFO_STREAM(this->get_logger(), "Substitution of player " << id_);
      }
  }

private:

  double random_double(double min, double max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dis(min, max);
    return dis(gen);
  }

	void ball_pos_vit_callback(const handball_msgs::msg::BallState::SharedPtr msg)
  {
        RCLCPP_INFO_STREAM(this->get_logger(), "Ball pos vit callback");
        ball_x_ = msg->pos.x;
        ball_y_ = msg->pos.y;
        ball_vx_ = msg->vel.vx;
        ball_vy_ = msg->vel.vy;
  }
  
 	void coach_instr_callback(const handball_msgs::msg::CoachInstruction::SharedPtr msg)
  {
      RCLCPP_INFO_STREAM(this->get_logger(), "coach instr callback" << msg->instr);
  }

    void player_control_callback(const handball_msgs::msg::PlayerControl::SharedPtr msg)
  {
      // RCLCPP_INFO_STREAM(this->get_logger(), "player control callback");
      if(!player_stop and energie_>0){
        switch(msg->action){
            case 1: //movement
                vx_ = msg->twist.vx;
                vy_ = msg->twist.vy;
                omega_ = msg->twist.omega;
                break;
            case 2: //kick
                if(sqrt((x_ - ball_x_)*(x_ - ball_x_)+(y_ - ball_y_)*(y_ - ball_y_))< 2){
                    RCLCPP_INFO_STREAM(this->get_logger(), "Player can kick ball");
                    energie_ -= 0.01;
                    ball_vx_ = msg->vel.vx;
                    ball_vy_ = msg->vel.vy;
                    // Besoin d'une action sur la balle
                }
                break;
            default:
                break;
        }
      }
  }
	void whistle_callback(const handball_msgs::msg::Whistle::SharedPtr msg)
  {
      RCLCPP_INFO_STREAM(this->get_logger(),"whistle callback");
      switch(msg->why){
        case 1:
          player_stop = false;
          break;
        case 2:
          player_stop = true;
          break;
        default:
          player_stop = true;
          break;

      }
  }
  
  void team_state_callback(const handball_msgs::msg::PlayerState::SharedPtr msg)
  {
      RCLCPP_INFO_STREAM(this->get_logger(), "team state callback" << msg->team);
  }

  void timer_callback()
  {
    static handball_msgs::msg::PlayerState player_state ;
    RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "Player running");
    energie_ -= 0.001;
    x_ +=std::max(std::min(period/1000.0f * vx_,dim_max),dim_min);
    y_ +=std::max(std::min(period/1000.0f * vy_,dim_max),dim_min);
    theta_ += period/1000.0f * omega_;

	player_state.id = id_    ;
	player_state.team  = team_ ;
	player_state.energie = energie_;
    player_state.posture.x = x_;
    player_state.posture.y = y_;
    player_state.posture.theta = theta_;
    player_state.twist.vx = vx_;
    player_state.twist.vy = vy_;
    player_state.twist.omega = omega_;

    state_pub_->publish(player_state);

    //Display
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "player_" + std::to_string(id_);

    t.transform.translation.x = x_;
    t.transform.translation.y = y_;
    t.transform.translation.z = 0;

    tf2::Quaternion q;
    q.setRPY(0,0, theta_);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
  }

  std::string team_;
  int id_;
  int period=100;
  float energie_, x_, y_, theta_, vx_, vy_, omega_;
  float ball_x_, ball_y_, ball_vx_, ball_vy_;
  float dim_max=100, dim_min=0;
  bool ball_pos_vit_received_,coach_instr_received_,
	player_control_received_,whistle_received_,
    team_state_received_,first_timer_call_;
  bool player_stop = false;
  rclcpp::Subscription<handball_msgs::msg::BallState>::SharedPtr ball_pos_vit_sub_;
  rclcpp::Subscription<handball_msgs::msg::CoachInstruction>::SharedPtr coach_instr_sub_;
  rclcpp::Subscription<handball_msgs::msg::PlayerControl>::SharedPtr player_control_sub_;
  rclcpp::Subscription<handball_msgs::msg::Whistle>::SharedPtr whistle_sub_;
  rclcpp::Publisher<handball_msgs::msg::PlayerState>::SharedPtr state_pub_;
  rclcpp::Subscription<handball_msgs::msg::PlayerState>::SharedPtr team_state_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlayerNode>();
  auto service = node -> create_service<handball_msgs::srv::SubstitutionExecution>(
              "substitution_execution",
              std::bind(
                  &PlayerNode::substitution,
                  node.get(),
                  _1,
                  _2
                  )
              );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

