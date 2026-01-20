#include <chrono>
#include <memory>
#include <string>
#include <unordered_set>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>

#include <handball_msgs/msg/whistle.hpp>
#include <handball_msgs/msg/ball_state.hpp>
#include <handball_msgs/msg/player_state.hpp>
#include <handball_msgs/msg/posture.hpp>

#include "handball_msgs/srv/substitution_approval.hpp"
#include "handball_msgs/action/substitution_execution.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

/* ============================================================
 * TEAM HANDLER
 * ============================================================
 * Classe pour gérer chaque équipe :
 * - écoute les états des joueurs via /blue_team/player_states ou /red_team/player_states
 * - détecte les fautes de placement (FOUL_PLAY)
 * - publie un coup de sifflet si un joueur entre dans une zone interdite
 */
class TeamHandler
{
public:
    TeamHandler(
        rclcpp::Node * node,
        const std::string & team_name,
        const rclcpp::Publisher<handball_msgs::msg::Whistle>::SharedPtr & whistle_pub)
    : node_(node),
      team_name_(team_name),
      whistle_pub_(whistle_pub)
    {
        auto qos = rclcpp::QoS(10).reliable();

        player_sub_ = node_->create_subscription<handball_msgs::msg::PlayerState>(
            "/" + team_name_ + "/player_state",
            qos,
            std::bind(&TeamHandler::player_callback, this, _1));

        RCLCPP_INFO(node_->get_logger(), "TeamHandler initialized for team '%s'", team_name_.c_str());
    }

    bool has_player(uint8_t id) const
    {
        if(id == 0 ||id == 1 || id == 2){
            RCLCPP_WARN(node_->get_logger(), "Player %d is available for substiution", id);
            return true;
        } else {
            RCLCPP_WARN(node_->get_logger(), "Player %d is not available for substiution", id);    
            return false;
        }
    }

private:
    void player_callback(const handball_msgs::msg::PlayerState::SharedPtr msg)
    {
        // Mémoriser les joueurs vus comme étant sur le terrain
        on_field_ids_.insert(static_cast<uint8_t>(msg->id));

        // Ignorer le gardien (id=0)
        if(msg->id == 0) return;

        // Détection zone interdite (exemple simple : x < 0)
        if(msg->posture.x < 0.0)
        {
            publish_foul(msg->team, msg->id, msg->posture.x, msg->posture.y);
        }
    }

    void publish_foul(const std::string & team, uint8_t player_id, double x, double y)
    {
        handball_msgs::msg::Whistle msg;
        msg.why = handball_msgs::msg::Whistle::FOUL_PLAY; // code 10
        msg.team = team;
        msg.position.x = x;
        msg.position.y = y;
        whistle_pub_->publish(msg);

        RCLCPP_WARN(node_->get_logger(),
                    "FOUL_PLAY: player %u from team %s in forbidden zone (%.2f, %.2f)",
                    player_id, team.c_str(), x, y);
    }

    rclcpp::Node * node_;
    std::string team_name_;
    rclcpp::Subscription<handball_msgs::msg::PlayerState>::SharedPtr player_sub_;
    rclcpp::Publisher<handball_msgs::msg::Whistle>::SharedPtr whistle_pub_;
    std::unordered_set<uint8_t> on_field_ids_;
};

/* ============================================================
 * REFEREE NODE
 * ============================================================
 * Node principal qui gère :
 * - le déroulement du match
 * - les coups de sifflet (START, SUSPEND, END, FOUL)
 * - les substitutions via Service + Action
 * - le timer du match et la pause/reprise
 */
class Referee : public rclcpp::Node
{
public:
        Referee()
        : Node("referee"),
            match_started_(false),
            match_paused_(false),
            subs_used_blue_(0),
            subs_used_red_(0)
    {
        // Paramètres
        declare_parameters();
        read_parameters();

        // Publishers / Subscribers
        init_publishers();
        init_subscribers();

                // Équipes (pour suivre les joueurs présents)
                blue_team_ = std::make_unique<TeamHandler>(this, "blue", whistle_pub_);
                red_team_  = std::make_unique<TeamHandler>(this, "red", whistle_pub_);

        // Services
        init_services();

        // Timer pour déroulement du match
        init_timer();

        start_time_ = now();
        elapsed_time_before_pause_ = 0.0;

        RCLCPP_INFO(get_logger(),
                    "Referee ready | duration=%d",
                    match_duration_);
    }

private:
    /* ================= PARAMETRES ================= */
    void declare_parameters()
    {
        declare_parameter<int>("match_duration", 600); // durée du match en secondes
    }

    void read_parameters()
    {
        match_duration_ = get_parameter("match_duration").as_int();
    }

    /* ================= INIT ================= */
    void init_publishers()
    {
        whistle_pub_ = create_publisher<handball_msgs::msg::Whistle>("/whistle", rclcpp::QoS(10).reliable());
    }

    void init_subscribers()
    {
        // Suivi de la balle
        ball_sub_ = create_subscription<handball_msgs::msg::BallState>(
            "/ball_state", rclcpp::QoS(10).reliable(),
            std::bind(&Referee::ball_callback, this, _1));

        // Fin de match manuelle
        end_match_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/end_match", rclcpp::QoS(10).reliable(),
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                if(msg->data) end_match_now();
            });
    }

    void init_timer()
    {
        // Timer appelé toutes les 100ms pour avancer le match
        timer_ = create_wall_timer(
            100ms, std::bind(&Referee::timer_callback, this));
    }

    void init_services()
    {
        // Politique: max 2 changements par équipe, et le joueur sortant doit être présent
        sub_approval_srv_ = create_service<handball_msgs::srv::SubstitutionApproval>(
            "/substitution_approval",
            [this](const handball_msgs::srv::SubstitutionApproval::Request::SharedPtr request,
                   handball_msgs::srv::SubstitutionApproval::Response::SharedPtr response)
            {
                TeamHandler* team = nullptr;
                int* subs_used = nullptr;

                if (request->team == "blue") {
                    team = blue_team_.get();
                    subs_used = &subs_used_blue_;
                } else if (request->team == "red") {
                    team = red_team_.get();
                    subs_used = &subs_used_red_;
                }

                if (*subs_used >= 2) {
                    response->status = handball_msgs::srv::SubstitutionApproval::Response::NO_MORE_SUBS;
                    RCLCPP_WARN(get_logger(), "Substitution refused (no more subs): team=%s out=%u in=%u",
                                request->team.c_str(), request->outgoing, request->incoming);
                    return;
                }

                if (!team->has_player(request->outgoing)) {
                    response->status = handball_msgs::srv::SubstitutionApproval::Response::NO_SUCH_PLAYER;
                    RCLCPP_WARN(get_logger(), "Substitution refused (no such player on field): team=%s out=%u",
                                request->team.c_str(), request->outgoing);
                    return;
                }

                // Accordé
                ++(*subs_used);
                response->status = handball_msgs::srv::SubstitutionApproval::Response::GRANTED;
                RCLCPP_INFO(get_logger(), "Substitution granted: team=%s out=%u in=%u (used=%d)",
                            request->team.c_str(), request->outgoing, request->incoming, *subs_used);
            });
    }

    /* ================= TIMER ================= */
    void timer_callback()
    {
        if(match_paused_) return; // ne rien faire si match en pause

        double elapsed = (now() - start_time_).seconds() + elapsed_time_before_pause_;

        // Début du match
        if(!match_started_)
        {
            publish_whistle(handball_msgs::msg::Whistle::START_PLAY, "", 0.0, 0.0);
            match_started_ = true;
            RCLCPP_INFO(get_logger(), "Match started");
        }

        // Fin du match automatique
        if(elapsed >= match_duration_)
        {
            publish_whistle(handball_msgs::msg::Whistle::END_OF_GAME, "", 0.0, 0.0);
            RCLCPP_INFO(get_logger(), "Match ended");
            timer_->cancel();
            rclcpp::shutdown();
        }
    }

    void ball_callback(const handball_msgs::msg::BallState::SharedPtr msg)
    {
        RCLCPP_DEBUG(get_logger(),
                     "Ball pos=(%.2f, %.2f) vel=(%.2f, %.2f)",
                     msg->pos.x, msg->pos.y,
                     msg->vel.vx, msg->vel.vy);
    }

    void publish_whistle(uint8_t code, const std::string & team, double x, double y)
    {
        handball_msgs::msg::Whistle msg;
        msg.why = code;
        msg.team = team;
        msg.position.x = x;
        msg.position.y = y;
        whistle_pub_->publish(msg);
    }

    /* ================= FIN DE MATCH MANUEL ================= */
    void end_match_now()
    {
        publish_whistle(handball_msgs::msg::Whistle::END_OF_GAME, "", 0.0, 0.0);
        RCLCPP_INFO(get_logger(), "Match ended manually");
        timer_->cancel();
        rclcpp::shutdown();
    }

    /* ================= MEMBRES ================= */
    int match_duration_;
    bool match_started_;
    bool match_paused_;
    double elapsed_time_before_pause_;
    rclcpp::Time start_time_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<handball_msgs::msg::Whistle>::SharedPtr whistle_pub_;
    rclcpp::Subscription<handball_msgs::msg::BallState>::SharedPtr ball_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr end_match_sub_;
    
    rclcpp::Service<handball_msgs::srv::SubstitutionApproval>::SharedPtr sub_approval_srv_;

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
