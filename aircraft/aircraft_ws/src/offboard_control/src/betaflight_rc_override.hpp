#ifndef OFFBOARD_CONTROL__BETAFLIGHT_RC_OVERRIDE_HPP_
#define OFFBOARD_CONTROL__BETAFLIGHT_RC_OVERRIDE_HPP_

#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <atomic>
#include <array>
#include <algorithm>
#include <string>
#include <sstream>
#include <iomanip>
#include <unordered_map>
#include <functional>

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <GeographicLib/Geodesic.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autopilot_interface_msgs/msg/offboard_flag.hpp"

#include "ground_system_msgs/msg/swarm_obs.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace geometry_msgs::msg;
using namespace sensor_msgs::msg;
using namespace std::chrono_literals; // for time literals (e.g. 1s)

// Stick scales of rc_override, from betaflight-2026.6.cli (linear stick curves) and Betaflight's defaults
constexpr double MAX_RATE_RAD_S = 670.0 * M_PI / 180.0; // Rate at full stick (rc_rate = srate = 67)
constexpr double ANGLE_LIMIT_RAD = 60.0 * M_PI / 180.0; // Tilt at full roll or pitch stick in ANGLE (angle_limit)
constexpr double FULL_STICK_SPEED_MS = 5.0; // Horizontal velocity at full roll or pitch stick in POS HOLD (ap_max_velocity)
constexpr double FULL_STICK_CLIMB_MS = 5.0; // Climb rate at full throttle in ALTHOLD (alt_hold_climb_rate)
constexpr double HOVER_THROTTLE = 0.595; // Throttle stick in [0, 1] that hovers the simulated Iris in ACRO and ANGLE (ap_hover_throttle 1595)
enum class RcMode : int { ACRO = 0, ANGLE = 1, VELOCITY = 2 }; // rc_override buttons[0], the flight mode switch (CH6) position

class BetaflightRcOverride : public rclcpp::Node
{
public:
    BetaflightRcOverride();

private:
    std::shared_mutex node_data_mutex_;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // Node variables
    bool offboard_active_;
    std::string active_controller_name_;
    int offboard_loop_frequency;
    std::atomic<int> offboard_loop_count_;
    std::atomic<int> last_offboard_loop_count_;
    rclcpp::Time last_offboard_rate_check_time_;
    int own_id_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_printout_;
    rclcpp::CallbackGroup::SharedPtr callback_group_offboard_control_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    // Node timers
    rclcpp::TimerBase::SharedPtr betaflight_interface_printout_timer_;
    rclcpp::TimerBase::SharedPtr offboard_control_loop_timer_;

    // betaflight_interface subscribers
    rclcpp::Subscription<NavSatFix>::SharedPtr global_position_sub_;
    rclcpp::Subscription<PointStamped>::SharedPtr local_position_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr velocity_sub_;
    rclcpp::Subscription<Vector3Stamped>::SharedPtr attitude_sub_;
    rclcpp::Subscription<Imu>::SharedPtr imu_sub_;

    // Offboard flag subscriber
    rclcpp::Subscription<autopilot_interface_msgs::msg::OffboardFlag>::SharedPtr offboard_flag_sub_;

    // Perception subscribers
    rclcpp::Subscription<ground_system_msgs::msg::SwarmObs>::SharedPtr ground_tracks_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_detections_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kiss_odometry_sub_;

    // Subscribers variables
    double lat_, lon_, alt_;
    double ve_, vn_, vu_;
    std::array<double, 3> position_; // ENU from the arming point
    std::array<double, 4> q_; // FLU body to ENU world (w, x, y, z)
    std::array<double, 3> angular_velocity_; // FLU
    double heading_; // rad, clockwise from North
    std::array<double, 3> kiss_position_;
    std::array<double, 4> kiss_q_;
    ground_system_msgs::msg::SwarmObs::SharedPtr ground_tracks_;
    vision_msgs::msg::Detection2DArray::SharedPtr yolo_detections_;

    // Guidance variables
    double desired_bearing_rad_, desired_elevation_rad_, closing_distance_;
    double target_vn_, target_ve_, target_vd_;
    rclcpp::Time last_track_time_;

    // Vision guidance variables
    std::vector<double> camera_extrinsics_;
    std::vector<std::string> search_classes_;
    double detect_az_rad_, detect_el_rad_;
    std::array<double, 3> detect_fix_enu_;
    int detect_fix_count_;
    rclcpp::Time last_detect_time_;

    // betaflight_interface publisher
    rclcpp::Publisher<Joy>::SharedPtr rc_override_pub_;

    // Callbacks for timers
    void betaflight_interface_printout_callback();
    void offboard_loop_callback();

    // Callbacks for betaflight_interface subscribers
    void global_position_callback(const NavSatFix::SharedPtr msg);
    void local_position_callback(const PointStamped::SharedPtr msg);
    void velocity_callback(const TwistStamped::SharedPtr msg);
    void attitude_callback(const Vector3Stamped::SharedPtr msg);
    void imu_callback(const Imu::SharedPtr msg);

    // Offboard flag call back
    void offboard_flag_callback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg);

    // Callbacks for perception subscribers
    void ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg);
    void yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void kiss_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Utility
    double normalize_heading(double angle_rad);
    std::array<double, 3> camera_bearings_to_enu(double az_rad, double el_rad);
    void publish_rc_override(double roll, double pitch, double throttle, double yaw, RcMode mode);
    void publish_rates_ref(double roll_rate, double pitch_rate, double yaw_rate, double throttle);
    void publish_attitude_ref(double roll, double pitch, double yaw_rate, double throttle);
    void publish_velocity_ref(double v_east, double v_north, double v_up, double yaw_rate);

    // Controller map and controllers
    using ControllerFunction = std::function<void()>;
    std::unordered_map<std::string, ControllerFunction> controller_map_;
    ControllerFunction active_controller_func_;
    void ctbr_ref_test();
    void att_ref_test();
    void vel_ref_test();
};

#endif // OFFBOARD_CONTROL__BETAFLIGHT_RC_OVERRIDE_HPP_
