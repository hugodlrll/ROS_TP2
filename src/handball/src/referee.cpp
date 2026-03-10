#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>

#include <handball_msgs/msg/whistle.hpp>
#include <handball_msgs/msg/ball_state.hpp>
#include <handball_msgs/msg/player_state.hpp>
#include <handball_msgs/msg/posture.hpp>

#include "handball_msgs/srv/substitution_approval.hpp"
#include "handball_msgs/srv/substitution_execution.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/* ============================================================
 * TEAM HANDLER
 * ============================================================ */
class TeamHandler
{
public:
    TeamHandler(
        rclcpp::Node * node,
        const std::string & team_name,
        const rclcpp::Publisher<handball_msgs::msg::Whistle>::SharedPtr & whistle_pub,
        double forbidden_x_min,
        double forbidden_x_max)
    : node_(node),
      team_name_(team_name),
      whistle_pub_(whistle_pub),
      forbidden_x_min_(forbidden_x_min),
      forbidden_x_max_(forbidden_x_max)
    {
        auto qos = rclcpp::QoS(10).reliable();

        player_sub_ = node_->create_subscription<handball_msgs::msg::PlayerState>(
            "/player_state",
            qos,
            std::bind(&TeamHandler::player_callback, this, _1));

        RCLCPP_INFO(node_->get_logger(),
                    "TeamHandler initialized for team '%s' | forbidden zone x=[%.2f, %.2f]",
                    team_name_.c_str(), forbidden_x_min_, forbidden_x_max_);
    }

    bool has_player(uint8_t id) const
    {
        bool found = on_field_ids_.count(id) > 0;
        if (found) {
            RCLCPP_INFO(node_->get_logger(),
                        "Player %d is available for substitution (team=%s)", id, team_name_.c_str());
        } else {
            RCLCPP_WARN(node_->get_logger(),
                        "Player %d is NOT on field (team=%s)", id, team_name_.c_str());
        }
        return found;
    }

private:
    void player_callback(const handball_msgs::msg::PlayerState::SharedPtr msg)
    {
        if (msg->team != team_name_) {
            return;
        }

        on_field_ids_.insert(static_cast<uint8_t>(msg->id));

        if (msg->id == 0) return; // ignorer le gardien

        if (msg->posture.x >= forbidden_x_min_ && msg->posture.x <= forbidden_x_max_)
        {
            publish_foul(team_name_, msg->id, msg->posture.x, msg->posture.y);
        }
    }

    void publish_foul(const std::string & team, uint8_t player_id, double x, double y)
    {
        handball_msgs::msg::Whistle msg;
        msg.why        = handball_msgs::msg::Whistle::FOUL_PLAY;
        msg.team       = team;
        msg.position.x = x;
        msg.position.y = y;
        whistle_pub_->publish(msg);

        RCLCPP_WARN(node_->get_logger(),
                    "FOUL_PLAY: player %u from team %s in forbidden zone (%.2f, %.2f)",
                    player_id, team.c_str(), x, y);
    }

    rclcpp::Node * node_;
    std::string    team_name_;
    rclcpp::Subscription<handball_msgs::msg::PlayerState>::SharedPtr player_sub_;
    rclcpp::Publisher<handball_msgs::msg::Whistle>::SharedPtr        whistle_pub_;
    std::unordered_set<uint8_t> on_field_ids_;
    double forbidden_x_min_;
    double forbidden_x_max_;
};

/* ============================================================
 * REFEREE NODE
 * ============================================================ */
class Referee : public rclcpp::Node
{
public:
    Referee()
    : Node("referee"),
      match_started_(false),
      match_paused_(false),
      subs_used_blue_(0),
      subs_used_red_(0),
    elapsed_time_before_pause_(0.0),
    reset_in_progress_(false)
    {
        declare_parameters();
        read_parameters();

        init_publishers();
        init_subscribers();

        blue_team_ = std::make_unique<TeamHandler>(
            this, "blue", whistle_pub_, forbidden_x_min_, forbidden_x_max_);
        red_team_  = std::make_unique<TeamHandler>(
            this, "red",  whistle_pub_, forbidden_x_min_, forbidden_x_max_);

        init_services();
        init_timer();

        start_time_ = now();

        RCLCPP_INFO(get_logger(),
                    "Referee ready | duration=%ds | max_subs=%d | forbidden_x=[%.2f,%.2f]",
                    match_duration_, max_subs_per_team_, forbidden_x_min_, forbidden_x_max_);
    }

private:
    /* ================= PARAMETRES ================= */
    void declare_parameters()
    {
        declare_parameter<int>   ("match_duration",    600);
        declare_parameter<int>   ("max_subs_per_team", 2);
        declare_parameter<double>("forbidden_x_min",  -10.0);
        declare_parameter<double>("forbidden_x_max",   0.0);
        declare_parameter<double>("goal_line_x",      20.0);
        declare_parameter<double>("goal_half_width",   3.0);
    }

    void read_parameters()
    {
        match_duration_    = get_parameter("match_duration").as_int();
        max_subs_per_team_ = get_parameter("max_subs_per_team").as_int();
        forbidden_x_min_   = get_parameter("forbidden_x_min").as_double();
        forbidden_x_max_   = get_parameter("forbidden_x_max").as_double();
        goal_line_x_       = get_parameter("goal_line_x").as_double();
        goal_half_width_   = get_parameter("goal_half_width").as_double();
    }

    /* ================= PUBLISHERS ================= */
    void init_publishers()
    {
        whistle_pub_ = create_publisher<handball_msgs::msg::Whistle>(
            "/whistle", rclcpp::QoS(10).reliable());
        ball_reset_pub_ = create_publisher<std_msgs::msg::Empty>(
            "/ball_reset", rclcpp::QoS(10).reliable());
    }

    /* ================= SUBSCRIBERS ================= */
    void init_subscribers()
    {
        ball_sub_ = create_subscription<handball_msgs::msg::BallState>(
            "/ball_state", rclcpp::QoS(10).reliable(),
            std::bind(&Referee::ball_callback, this, _1));

        end_match_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/end_match", rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) end_match_now();
            });

        pause_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/pause_match", rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && !match_paused_) {
                    match_paused_ = true;
                    pause_start_  = now();
                    publish_whistle(handball_msgs::msg::Whistle::SUSPEND_PLAY, "", 0.0, 0.0);
                    RCLCPP_INFO(get_logger(), "Match PAUSED");
                } else if (!msg->data && match_paused_) {
                    elapsed_time_before_pause_ += (now() - pause_start_).seconds();
                    match_paused_ = false;
                    publish_whistle(handball_msgs::msg::Whistle::START_PLAY, "", 0.0, 0.0);
                    RCLCPP_INFO(get_logger(), "Match RESUMED");
                }
            });
    }

    /* ================= TIMER ================= */
    void init_timer()
    {
        timer_ = create_wall_timer(
            100ms, std::bind(&Referee::timer_callback, this));
    }

    /* ================= SERVICES ================= */
    void init_services()
    {
        // --- SubstitutionApproval : vérifie si le changement est autorisé ---
        sub_approval_srv_ = create_service<handball_msgs::srv::SubstitutionApproval>(
            "/substitution_approval",
            [this](const handball_msgs::srv::SubstitutionApproval::Request::SharedPtr  request,
                         handball_msgs::srv::SubstitutionApproval::Response::SharedPtr response)
            {
                TeamHandler * team      = nullptr;
                int *         subs_used = nullptr;

                if      (request->team == "blue") { team = blue_team_.get(); subs_used = &subs_used_blue_; }
                else if (request->team == "red")  { team = red_team_.get();  subs_used = &subs_used_red_;  }
                else {
                    response->status = handball_msgs::srv::SubstitutionApproval::Response::NO_SUCH_PLAYER;
                    RCLCPP_WARN(get_logger(), "Unknown team: %s", request->team.c_str());
                    return;
                }

                if (*subs_used >= max_subs_per_team_) {
                    response->status = handball_msgs::srv::SubstitutionApproval::Response::NO_MORE_SUBS;
                    RCLCPP_WARN(get_logger(),
                                "Substitution refused (no more subs): team=%s out=%u in=%u used=%d/%d",
                                request->team.c_str(), request->outgoing, request->incoming,
                                *subs_used, max_subs_per_team_);
                    return;
                }

                if (!team->has_player(request->outgoing)) {
                    response->status = handball_msgs::srv::SubstitutionApproval::Response::NO_SUCH_PLAYER;
                    RCLCPP_WARN(get_logger(),
                                "Substitution refused (player not on field): team=%s out=%u",
                                request->team.c_str(), request->outgoing);
                    return;
                }

                ++(*subs_used);
                response->status = handball_msgs::srv::SubstitutionApproval::Response::GRANTED;
                RCLCPP_INFO(get_logger(),
                            "Substitution granted: team=%s out=%u in=%u (used=%d/%d)",
                            request->team.c_str(), request->outgoing, request->incoming,
                            *subs_used, max_subs_per_team_);
            });

        // --- SubstitutionExecution : effectue le changement (suspend → attend → reprend) ---
        sub_exec_srv_ = create_service<handball_msgs::srv::SubstitutionExecution>(
            "substitution_execution",
            [this](const handball_msgs::srv::SubstitutionExecution::Request::SharedPtr  request,
                         handball_msgs::srv::SubstitutionExecution::Response::SharedPtr response)
            {
                RCLCPP_INFO(get_logger(),
                            "SubstitutionExecution: outgoing=%u", request->outgoing);

                publish_whistle(handball_msgs::msg::Whistle::SUSPEND_PLAY, "", 0.0, 0.0);
                std::this_thread::sleep_for(std::chrono::seconds(3));
                publish_whistle(handball_msgs::msg::Whistle::START_PLAY, "", 0.0, 0.0);

                response->status = handball_msgs::srv::SubstitutionExecution::Response::DONE;
                RCLCPP_INFO(get_logger(),
                            "SubstitutionExecution done: outgoing=%u", request->outgoing);
            });

    } // ← fin de init_services()

    /* ================= TIMER CALLBACK ================= */
    void timer_callback()
    {
        if (match_paused_) return;

        double elapsed = (now() - start_time_).seconds() + elapsed_time_before_pause_;

        if (!match_started_)
        {
            publish_whistle(handball_msgs::msg::Whistle::START_PLAY, "", 0.0, 0.0);
            match_started_ = true;
            RCLCPP_INFO(get_logger(), "Match started");
        }

        if (elapsed >= static_cast<double>(match_duration_))
        {
            end_match_now();
        }
    }

    /* ================= CALLBACKS ================= */
    void ball_callback(const handball_msgs::msg::BallState::SharedPtr msg)
    {
        RCLCPP_DEBUG(get_logger(),
                     "Ball pos=(%.2f, %.2f) vel=(%.2f, %.2f)",
                     msg->pos.x, msg->pos.y,
                     msg->vel.vx, msg->vel.vy);

        if (match_paused_ || reset_in_progress_) {
            return;
        }

        const bool in_goal_window = std::abs(msg->pos.y) <= goal_half_width_;
        const bool crossed_goal_line = std::abs(msg->pos.x) >= goal_line_x_;

        if (in_goal_window && crossed_goal_line) {
            handle_goal(msg->pos.x > 0.0 ? "blue" : "red");
        }
    }

    void handle_goal(const std::string & conceding_team)
    {
        reset_in_progress_ = true;

        RCLCPP_INFO(get_logger(),
                    "Goal detected (against %s) -> reset ball to center",
                    conceding_team.c_str());

        publish_whistle(handball_msgs::msg::Whistle::SUSPEND_PLAY, "", 0.0, 0.0);

        std_msgs::msg::Empty reset_msg;
        ball_reset_pub_->publish(reset_msg);

        publish_whistle(handball_msgs::msg::Whistle::START_PLAY, "", 0.0, 0.0);
        reset_in_progress_ = false;
    }

    /* ================= HELPERS ================= */
    void publish_whistle(uint8_t code, const std::string & team, double x, double y)
    {
        handball_msgs::msg::Whistle msg;
        msg.why        = code;
        msg.team       = team;
        msg.position.x = x;
        msg.position.y = y;
        whistle_pub_->publish(msg);
    }

    void end_match_now()
    {
        publish_whistle(handball_msgs::msg::Whistle::END_OF_GAME, "", 0.0, 0.0);
        RCLCPP_INFO(get_logger(), "Match ended");
        timer_->cancel();
        rclcpp::shutdown();
    }

    /* ================= MEMBRES ================= */
    int    match_duration_;
    int    max_subs_per_team_;
    double forbidden_x_min_;
    double forbidden_x_max_;
    double goal_line_x_;
    double goal_half_width_;

    bool   match_started_;
    bool   match_paused_;
    double elapsed_time_before_pause_;
    bool   reset_in_progress_;

    rclcpp::Time start_time_;
    rclcpp::Time pause_start_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<handball_msgs::msg::Whistle>::SharedPtr      whistle_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr             ball_reset_pub_;
    rclcpp::Subscription<handball_msgs::msg::BallState>::SharedPtr ball_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           end_match_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr           pause_sub_;

    rclcpp::Service<handball_msgs::srv::SubstitutionApproval>::SharedPtr  sub_approval_srv_;
    rclcpp::Service<handball_msgs::srv::SubstitutionExecution>::SharedPtr sub_exec_srv_;

    std::unique_ptr<TeamHandler> blue_team_;
    std::unique_ptr<TeamHandler> red_team_;

    int subs_used_blue_;
    int subs_used_red_;
};

/* ============================================================
 * MAIN
 * ============================================================ */
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Referee>());
    rclcpp::shutdown();
    return 0;
}
