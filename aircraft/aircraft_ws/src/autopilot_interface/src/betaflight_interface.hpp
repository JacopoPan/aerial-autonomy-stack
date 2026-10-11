#ifndef AUTOPILOT_INTERFACE__BETAFLIGHT_INTERFACE_HPP_
#define AUTOPILOT_INTERFACE__BETAFLIGHT_INTERFACE_HPP_

#include <fcntl.h>
#include <netdb.h>
#include <signal.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include <GeographicLib/LocalCartesian.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"
#include "autopilot_interface_msgs/msg/offboard_flag.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "betaflight_stick_constants.hpp"

using namespace std::chrono_literals;
using Takeoff = autopilot_interface_msgs::action::Takeoff;
using Land = autopilot_interface_msgs::action::Land;
using Offboard = autopilot_interface_msgs::action::Offboard;
using SetReposition = autopilot_interface_msgs::srv::SetReposition;
template <typename ActionT> using GoalHandle = std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>>;

enum : uint8_t { // MSP v1 commands
    MSP_RAW_IMU = 102,
    MSP_RAW_GPS = 106,
    MSP_ATTITUDE = 108,
    MSP_ALTITUDE = 109,
    MSP_GPS_RESCUE = 135,
    MSP_STATUS_EX = 150,
    MSP_SET_RAW_RC = 200,
    MSP_MULTIPLE_MSP = 230,
};

enum class State { INIT, STARTED, ARMING, REPOSITIONING, HOVER, LANDING, OFFBOARD };
constexpr const char *STATE_NAMES[] = {"INIT", "STARTED", "ARMING", "REPOSITIONING", "HOVER", "LANDING", "OFFBOARD"};

class BetaflightInterface : public rclcpp::Node
{
public:
    BetaflightInterface();
    ~BetaflightInterface();

private:
    // Parameters - Guidance, through the POS HOLD and ALTHOLD sticks (see betaflight-2026.6.cli)
    double POSITION_GAIN, ALTITUDE_GAIN, MAX_CLIMB, LAND_SPEED;
    // Parameters - Action completion and timeouts
    double TOUCHDOWN_TIME, ARRIVAL_DISTANCE, ARRIVAL_SPEED, ARMING_TIMEOUT, RC_OVERRIDE_TIMEOUT;

    // MSP link
    const std::vector<uint8_t> state_cmds_; // Handled in this order, so velocity (published on MSP_RAW_GPS) uses this request's vertical speed (from MSP_ALTITUDE)
    int fd_;
    std::vector<uint8_t> rx_;
    int acc_1g_;

    // Betaflight state, from MSP replies
    bool status_received_, armed_, gps_fix_, hover_read_;
    uint32_t arming_disable_flags_;
    uint16_t hover_pwm_; // ap_hover_throttle
    double latitude_, longitude_; // deg
    double altitude_; // m, from the arming point once armed
    double vertical_speed_; // m/s, up
    double ground_speed_; // m/s
    double heading_; // rad, clockwise from north

    // Interface state
    State state_;
    rclcpp::Time state_since_;
    bool descended_; // While landing, the descent began
    rclcpp::Time quiet_since_; // While landing, neither descending nor moving since
    GeographicLib::LocalCartesian home_; // ENU from the arming point
    bool home_set_;
    std::array<double, 3> target_; // m, east, north, up from home
    double takeoff_altitude_; // m
    std::string offboard_controller_;
    double offboard_duration_; // s
    sensor_msgs::msg::Joy rc_override_;
    rclcpp::Time rc_override_time_;
    GoalHandle<Takeoff> takeoff_goal_;
    GoalHandle<Land> land_goal_;
    GoalHandle<Offboard> offboard_goal_;

    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr local_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::Publisher<autopilot_interface_msgs::msg::OffboardFlag>::SharedPtr offboard_flag_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr rc_override_sub_;
    rclcpp_action::Server<Takeoff>::SharedPtr takeoff_server_;
    rclcpp_action::Server<Land>::SharedPtr land_server_;
    rclcpp_action::Server<Offboard>::SharedPtr offboard_server_;
    rclcpp::Service<SetReposition>::SharedPtr reposition_service_;
    rclcpp::TimerBase::SharedPtr timer_, offboard_flag_timer_;

    // MSP link
    static int open_device(const std::string &device);
    void send_msp(uint8_t cmd, const std::vector<uint8_t> &payload = {});
    void step();
    void handle(uint8_t cmd, const uint8_t *d, size_t size);

    // State machine and RC channels
    void update_state();
    std::vector<uint8_t> rc_channels() const;
    double hold_throttle(double climb) const;
    std::array<double, 3> position() const;
    std::array<double, 3> offset() const;
    void hover_here();
    bool arrived() const;
    bool rc_override_valid() const;

    // Actions and services
    rclcpp_action::GoalResponse accept_takeoff();
    void start_takeoff(const GoalHandle<Takeoff> &goal);
    rclcpp_action::GoalResponse accept_land();
    void start_land(const GoalHandle<Land> &goal);
    rclcpp_action::GoalResponse accept_offboard();
    void start_offboard(const GoalHandle<Offboard> &goal);
    void reposition(const SetReposition::Request &request, SetReposition::Response &response);

    // Utility
    void set_state(State state);
    template <typename ActionT>
    void feedback(const GoalHandle<ActionT> &goal, const std::string &message);
    template <typename ActionT>
    void finish(GoalHandle<ActionT> &goal, bool success, const std::string &message);
    static std::string hex(uint32_t value);
};

#endif // AUTOPILOT_INTERFACE__BETAFLIGHT_INTERFACE_HPP_
