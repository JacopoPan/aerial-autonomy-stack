#include <chrono>
#include <memory>
#include <string>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"

#include <nlohmann/json.hpp>

using json = nlohmann::json;
using namespace std::chrono_literals;

class DTCClient : public rclcpp::Node
{
public:
    DTCClient() : Node("dtc_client"), action_accepted_(true), target_action_("")
    {
        const char* env_drone_id = std::getenv("DRONE_ID");
        drone_id_ = env_drone_id ? std::string(env_drone_id) : "1";

        RCLCPP_INFO(this->get_logger(), "Starting DTC Client for Drone %s", drone_id_.c_str());

        cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/dtc_commands", 10,
            std::bind(&DTCClient::cmd_cb, this, std::placeholders::_1));

        std::string repo_service = "/Drone" + drone_id_ + "/set_reposition";
        repo_cli_ = this->create_client<autopilot_interface_msgs::srv::SetReposition>(repo_service);

        std::string tkf_action = "/Drone" + drone_id_ + "/takeoff_action";
        tkf_cli_ = rclcpp_action::create_client<autopilot_interface_msgs::action::Takeoff>(this, tkf_action);

        enforcer_timer_ = this->create_wall_timer(1s, // 1Hz timer
            std::bind(&DTCClient::enforcement_loop, this));
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

            // Extract Target Action and Payload
            target_action_ = cmd.value("action", "");
            target_alt_    = cmd.value("alt", 40.0f);
            target_east_   = cmd.value("east", 0.0f);
            target_north_  = cmd.value("north", 0.0f);

            action_accepted_ = false;
            RCLCPP_INFO(this->get_logger(), "New Command Queued: %s", target_action_.c_str());

        } catch (const json::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to process command: %s", e.what());
        }
    }

    void enforcement_loop()
    {
        if (action_accepted_ || target_action_.empty()) return;

        RCLCPP_INFO(this->get_logger(), "Attempting to enforce %s...", target_action_.c_str());

        if (target_action_ == "takeoff") {
            if (!tkf_cli_->action_server_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Takeoff server not ready. Waiting...");
                return;
            }

            auto goal_msg = autopilot_interface_msgs::action::Takeoff::Goal();
            goal_msg.takeoff_altitude = target_alt_;

            auto send_goal_options = rclcpp_action::Client<autopilot_interface_msgs::action::Takeoff>::SendGoalOptions();
            send_goal_options.goal_response_callback = 
                std::bind(&DTCClient::takeoff_response_cb, this, std::placeholders::_1);

            tkf_cli_->async_send_goal(goal_msg, send_goal_options);

        } else if (target_action_ == "reposition") {
            if (!repo_cli_->service_is_ready()) {
                RCLCPP_WARN(this->get_logger(), "Reposition service not ready. Waiting...");
                return;
            }

            auto request = std::make_shared<autopilot_interface_msgs::srv::SetReposition::Request>();
            request->east = target_east_;
            request->north = target_north_;
            request->altitude = target_alt_;

            repo_cli_->async_send_request(request, 
                std::bind(&DTCClient::reposition_response_cb, this, std::placeholders::_1));
        }
    }

    void takeoff_response_cb(const rclcpp_action::ClientGoalHandle<autopilot_interface_msgs::action::Takeoff>::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_WARN(this->get_logger(), "Autopilot REJECTED Takeoff. Will retry...");
        } else {
            RCLCPP_INFO(this->get_logger(), "Autopilot ACCEPTED Takeoff!");
            action_accepted_ = true;
        }
    }

    void reposition_response_cb(rclcpp::Client<autopilot_interface_msgs::srv::SetReposition>::SharedFuture future)
    {
        auto res = future.get();
        if (res->success) {
            RCLCPP_INFO(this->get_logger(), "Autopilot ACCEPTED Reposition!");
            action_accepted_ = true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Autopilot REJECTED Reposition: %s. Will retry...", res->message.c_str());
        }
    }

    std::string drone_id_;
    std::string target_action_;
    float target_alt_, target_east_, target_north_;
    bool action_accepted_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
    rclcpp::Client<autopilot_interface_msgs::srv::SetReposition>::SharedPtr repo_cli_;
    rclcpp_action::Client<autopilot_interface_msgs::action::Takeoff>::SharedPtr tkf_cli_;
    rclcpp::TimerBase::SharedPtr enforcer_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DTCClient>());
    rclcpp::shutdown();
    return 0;
}
