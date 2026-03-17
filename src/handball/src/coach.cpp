/**
 * @file coach.cpp
 * @author Hugo DELARUELLE
 * @brief Simulates the behavior of a handball coach
 * @version 0.1
 * @date 2026-03-17
 * @details The coach node simulates a handball coach that monitors the positions of players and give them some advices.
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <memory>
#include <cmath>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "handball_msgs/msg/ball_state.hpp"
#include "handball_msgs/msg/player_state.hpp"
#include "handball_msgs/msg/coach_instruction.hpp"
#include "handball_msgs/srv/substitution_approval.hpp"
#include "handball_msgs/srv/substitution_execution.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
using SubstitutionApprovalSrv = handball_msgs::srv::SubstitutionApproval;
using SubstitutionExecutionSrv = handball_msgs::srv::SubstitutionExecution;


class Coach : public rclcpp::Node
{
public:
  Coach()
  : Node("coach")
  {
    auto qos = rclcpp::QoS(10);

    // Subscriptions
    ball_sub_ = this->create_subscription<handball_msgs::msg::BallState>(
      "/ball_state",
      qos,
      std::bind(&Coach::ball_callback, this, _1)
    );

    player_sub_ = this->create_subscription<handball_msgs::msg::PlayerState>(
      "/player_state",
      qos,
      std::bind(&Coach::player_callback, this, _1)
    );

    // Publisher
    instr_pub_ = this->create_publisher<handball_msgs::msg::CoachInstruction>(
      "coach_instruction",
      qos
    );

    subApp_cli_ = create_client<SubstitutionApprovalSrv>("/substitution_approval");
    subEx_cli_ = create_client<SubstitutionExecutionSrv>("/substitution_execution");

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
      inst.instr = "REQUEST PLAYER CHANGER : rest";
      request_substitution(msg->team, static_cast<uint8_t>(msg->id), static_cast<uint8_t>(msg->id));
    } else {
      inst.instr = "KEEP_GOING: good energy";
    }
    instr_pub_->publish(inst);

    RCLCPP_DEBUG(this->get_logger(), "Published coach instruction from player_callback (id=%u): %s", msg->id, inst.instr.c_str());
  }

  void request_substitution(const std::string & team, uint8_t outgoing, uint8_t incoming)
  {
    if (!subApp_cli_) {
      RCLCPP_WARN(this->get_logger(), "SubstitutionApproval client not initialized");
      return;
    }

    // Wait briefly for the service to be available, avoid hard failure if server is late
    if (!subApp_cli_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "SubstitutionApproval service unavailable, will retry later");
      return;
    }

    auto req = std::make_shared<SubstitutionApprovalSrv::Request>();
    req->team = team;
    req->outgoing = outgoing;
    req->incoming = incoming;

    auto future = subApp_cli_->async_send_request(
      req,
      [this, outgoing, incoming](rclcpp::Client<SubstitutionApprovalSrv>::SharedFuture resp) {
        if (!resp.valid()) {
          RCLCPP_WARN(this->get_logger(), "Substitution response invalid");
          return;
        }

        const auto status = resp.get()->status;
        switch (status) {
          case SubstitutionApprovalSrv::Response::GRANTED:
            RCLCPP_INFO(this->get_logger(), "Substitution granted: out=%u in=%u", outgoing, incoming);
            execute_substitution(outgoing);
            break;
          case SubstitutionApprovalSrv::Response::NO_SUCH_PLAYER:
            RCLCPP_WARN(this->get_logger(), "Substitution refused (no such player): out=%u", outgoing);
            break;
          case SubstitutionApprovalSrv::Response::NO_MORE_SUBS:
            RCLCPP_WARN(this->get_logger(), "Substitution refused (no more subs): out=%u in=%u", outgoing, incoming);
            break;
          default:
            RCLCPP_WARN(this->get_logger(), "Substitution response unknown status=%u", status);
            break;
        }
      }
    );

    (void)future;  // keep future alive until callback completes
  }

  void execute_substitution(uint8_t outgoing) {
    if (!subEx_cli_) {
      RCLCPP_WARN(this->get_logger(), "SubstitutionExecution client not initialized");
      return;
    }

    // Wait briefly for the service to be available, avoid hard failure if server is late
    if (!subEx_cli_->wait_for_service(1s)) {
      RCLCPP_WARN(this->get_logger(), "SubstitutionExecution service unavailable, will retry later");
      return;
    }

    auto req = std::make_shared<SubstitutionExecutionSrv::Request>();
    req->outgoing = outgoing;

    auto future = subEx_cli_->async_send_request(
      req,
      [this, outgoing](rclcpp::Client<SubstitutionExecutionSrv>::SharedFuture resp) {
          if (!resp.valid()) {
            RCLCPP_WARN(this->get_logger(), "Substitution response invalid");
            return;
          }

          const auto status = resp.get()->status;
          switch (status) {
            case SubstitutionExecutionSrv::Response::DONE:
              RCLCPP_INFO(this->get_logger(), "SubstitutionExecution succeeded  : %u is out", outgoing);
              break;
            default:
              RCLCPP_WARN(this->get_logger(), "SubstitutionExecution service unavailable, will retry later");
              break;
          }
      }
    );

    (void)future;
  }

  rclcpp::Subscription<handball_msgs::msg::BallState>::SharedPtr ball_sub_;
  rclcpp::Subscription<handball_msgs::msg::PlayerState>::SharedPtr player_sub_;
  rclcpp::Publisher<handball_msgs::msg::CoachInstruction>::SharedPtr instr_pub_;
  rclcpp::Client<SubstitutionApprovalSrv>::SharedPtr subApp_cli_;
  rclcpp::Client<SubstitutionExecutionSrv>::SharedPtr subEx_cli_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Coach>());
  rclcpp::shutdown();
  return 0;
}

