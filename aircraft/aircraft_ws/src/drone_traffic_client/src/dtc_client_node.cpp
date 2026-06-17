#include <chrono>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"
#include "autopilot_interface_msgs/action/orbit.hpp"
#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/srv/set_speed.hpp"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

class DTCClient : public rclcpp::Node
{
public:
    DTCClient() : Node("dtc_client"), action_accepted_(true), request_being_sent_(false), target_action_("")
    {
        const char* env_drone_id = std::getenv("DRONE_ID");
        drone_id_ = env_drone_id ? std::string(env_drone_id) : "1";
        RCLCPP_INFO(this->get_logger(), "Starting DTC Client for Drone %s", drone_id_.c_str());

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/dtc_commands", 10, std::bind(&DTCClient::cmd_cb, this, std::placeholders::_1));

        tkf_cli_ = rclcpp_action::create_client<autopilot_interface_msgs::action::Takeoff>(this, "/Drone" + drone_id_ + "/takeoff_action");
        land_cli_ = rclcpp_action::create_client<autopilot_interface_msgs::action::Land>(this, "/Drone" + drone_id_ + "/land_action");
        orbit_cli_ = rclcpp_action::create_client<autopilot_interface_msgs::action::Orbit>(this, "/Drone" + drone_id_ + "/orbit_action");
        offboard_cli_ = rclcpp_action::create_client<autopilot_interface_msgs::action::Offboard>(this, "/Drone" + drone_id_ + "/offboard_action");
        repo_cli_ = this->create_client<autopilot_interface_msgs::srv::SetReposition>("/Drone" + drone_id_ + "/set_reposition");
        speed_cli_ = this->create_client<autopilot_interface_msgs::srv::SetSpeed>("/Drone" + drone_id_ + "/set_speed");

        enforcer_timer_ = this->create_wall_timer(1s, std::bind(&DTCClient::enforcement_loop, this)); // 1Hz timer
    }

private:
    void cmd_cb(const std_msgs::msg::String::SharedPtr msg)
    {
        try {
            auto cmd = json::parse(msg->data);

            // Handle drone_id being sent as an int or string
            std::string rx_id;
            if (cmd.contains("drone_id")) {
                if (cmd["drone_id"].is_number()) {
                    rx_id = std::to_string(cmd["drone_id"].get<int>());
                } else {
                    rx_id = cmd["drone_id"].get<std::string>();
                }
            }
            if (rx_id != drone_id_) return;

            // Extract action/service and payload (some fields are re-used)
            target_action_ = cmd.value("action", "");
            target_alt_    = cmd.value("alt", 40.0f);
            target_east_   = cmd.value("east", 0.0f);
            target_north_  = cmd.value("north", 0.0f);
            target_radius_ = cmd.value("radius", 50.0f);
            target_vtol_heading_ = cmd.value("vtol_heading", 90.0f);
            target_vtol_loiter_n_ = cmd.value("vtol_loiter_n", 150.0f);
            target_vtol_loiter_e_ = cmd.value("vtol_loiter_e", 0.0f);
            target_vtol_loiter_alt_ = cmd.value("vtol_loiter_alt", 100.0f);
            target_offboard_type_ = cmd.value("offboard_type", 1);
            target_duration_ = cmd.value("duration", 3.0f);
            target_speed_  = cmd.value("speed", 5.0f);

            action_accepted_ = false;
            request_being_sent_ = false;
            RCLCPP_INFO(this->get_logger(), "New Command Queued: %s", target_action_.c_str());

        } catch (const json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process command: %s", e.what());
        }
    }

    void enforcement_loop()
    {
        if (action_accepted_ || target_action_.empty() || request_being_sent_) return;

        RCLCPP_INFO(this->get_logger(), "Attempting to enforce %s...", target_action_.c_str());
        request_being_sent_ = true;

        auto action_cb = [this](auto gh) {
            request_being_sent_ = false;
            if (!gh) RCLCPP_WARN(this->get_logger(), "Autopilot REJECTED %s. Retrying...", target_action_.c_str());
            else { RCLCPP_INFO(this->get_logger(), "Autopilot ACCEPTED %s!", target_action_.c_str()); action_accepted_ = true; }
        };

        if (target_action_ == "takeoff") {
            if (!tkf_cli_->action_server_is_ready()) return;
            auto goal = autopilot_interface_msgs::action::Takeoff::Goal();
            goal.takeoff_altitude = target_alt_;
            goal.vtol_transition_heading = target_vtol_heading_;
            goal.vtol_loiter_nord = target_vtol_loiter_n_;
            goal.vtol_loiter_east = target_vtol_loiter_e_;
            goal.vtol_loiter_alt = target_vtol_loiter_alt_;
            auto opts = rclcpp_action::Client<autopilot_interface_msgs::action::Takeoff>::SendGoalOptions();
            opts.goal_response_callback = action_cb;
            tkf_cli_->async_send_goal(goal, opts);

        } else if (target_action_ == "orbit") {
            if (!orbit_cli_->action_server_is_ready()) return;
            auto goal = autopilot_interface_msgs::action::Orbit::Goal();
            goal.east = target_east_; goal.north = target_north_;
            goal.altitude = target_alt_; goal.radius = target_radius_;
            auto opts = rclcpp_action::Client<autopilot_interface_msgs::action::Orbit>::SendGoalOptions();
            opts.goal_response_callback = action_cb;
            orbit_cli_->async_send_goal(goal, opts);

        } else if (target_action_ == "land") {
            if (!land_cli_->action_server_is_ready()) return;
            auto goal = autopilot_interface_msgs::action::Land::Goal();
            goal.landing_altitude = target_alt_;
            goal.vtol_transition_heading = target_vtol_heading_;
            auto opts = rclcpp_action::Client<autopilot_interface_msgs::action::Land>::SendGoalOptions();
            opts.goal_response_callback = action_cb;
            land_cli_->async_send_goal(goal, opts);

        } else if (target_action_ == "offboard") {
            if (!offboard_cli_->action_server_is_ready()) return;
            auto goal = autopilot_interface_msgs::action::Offboard::Goal();
            goal.offboard_setpoint_type = target_offboard_type_;
            goal.max_duration_sec = target_duration_;
            auto opts = rclcpp_action::Client<autopilot_interface_msgs::action::Offboard>::SendGoalOptions();
            opts.goal_response_callback = action_cb;
            offboard_cli_->async_send_goal(goal, opts);

        } else if (target_action_ == "reposition") {
            if (!repo_cli_->service_is_ready()) return;
            auto req = std::make_shared<autopilot_interface_msgs::srv::SetReposition::Request>();
            req->east = target_east_; req->north = target_north_; req->altitude = target_alt_;
            repo_cli_->async_send_request(req, [this](rclcpp::Client<autopilot_interface_msgs::srv::SetReposition>::SharedFuture f) {
                request_being_sent_ = false;
                auto res = f.get();
                if (res->success) { RCLCPP_INFO(this->get_logger(), "Autopilot ACCEPTED Reposition!"); action_accepted_ = true; }
                else RCLCPP_WARN(this->get_logger(), "Autopilot REJECTED Reposition: %s. Retrying...", res->message.c_str());
            });

        } else if (target_action_ == "set_speed") {
            if (!speed_cli_->service_is_ready()) return;
            auto req = std::make_shared<autopilot_interface_msgs::srv::SetSpeed::Request>();
            req->speed = target_speed_;
            speed_cli_->async_send_request(req, [this](rclcpp::Client<autopilot_interface_msgs::srv::SetSpeed>::SharedFuture f) {
                request_being_sent_ = false;
                auto res = f.get();
                if (res->success) { RCLCPP_INFO(this->get_logger(), "Autopilot ACCEPTED SetSpeed!"); action_accepted_ = true; }
                else RCLCPP_WARN(this->get_logger(), "Autopilot REJECTED SetSpeed: %s. Retrying...", res->message.c_str());
            });
        }
    }

    std::string drone_id_;
    std::string target_action_;
    float target_alt_, target_east_, target_north_;
    float target_radius_, target_speed_, target_duration_;
    float target_vtol_heading_, target_vtol_loiter_n_, target_vtol_loiter_e_, target_vtol_loiter_alt_;
    int target_offboard_type_;
    bool action_accepted_;
    bool request_being_sent_; // Prevent spamming the same command

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp_action::Client<autopilot_interface_msgs::action::Takeoff>::SharedPtr tkf_cli_;
    rclcpp_action::Client<autopilot_interface_msgs::action::Land>::SharedPtr land_cli_;
    rclcpp_action::Client<autopilot_interface_msgs::action::Orbit>::SharedPtr orbit_cli_;
    rclcpp_action::Client<autopilot_interface_msgs::action::Offboard>::SharedPtr offboard_cli_;
    rclcpp::Client<autopilot_interface_msgs::srv::SetReposition>::SharedPtr repo_cli_;
    rclcpp::Client<autopilot_interface_msgs::srv::SetSpeed>::SharedPtr speed_cli_;
    rclcpp::TimerBase::SharedPtr enforcer_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DTCClient>());
    rclcpp::shutdown();
    return 0;
}
