#ifndef OFFBOARD_CONTROL__ARDUPILOT_GUIDED_HPP_
#define OFFBOARD_CONTROL__ARDUPILOT_GUIDED_HPP_

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

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <GeographicLib/Geodesic.hpp>

#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include "geometry_msgs/msg/vector3.hpp"
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <mavros_msgs/msg/vfr_hud.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autopilot_interface_msgs/msg/offboard_flag.hpp"

#include "ground_system_msgs/msg/swarm_obs.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace geometry_msgs::msg;
using namespace mavros_msgs::msg;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace GeographicLib;
using namespace geographic_msgs::msg;
using namespace std::chrono_literals;  // for time literals (e.g. 1s)

class ArdupilotGuided : public rclcpp::Node
{
public:
    ArdupilotGuided();

private:
    std::shared_mutex node_data_mutex_;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // Node variables
    std::atomic<int> offboard_flag_;
    int offboard_loop_frequency;
    std::atomic<int> offboard_loop_count_;
    std::atomic<int> last_offboard_loop_count_;
    rclcpp::Time last_offboard_rate_check_time_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;

    // Node timers
    rclcpp::TimerBase::SharedPtr ardupilot_interface_printout_timer_;
    rclcpp::TimerBase::SharedPtr offboard_control_loop_timer_;

    // MAVROS subscribers
    rclcpp::Subscription<NavSatFix>::SharedPtr mavros_global_position_global_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr mavros_local_position_odom_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr mavros_global_position_local_sub_;
    rclcpp::Subscription<VfrHud>::SharedPtr mavros_vfr_hud_sub_;

    // Offboard flag subscriber
    rclcpp::Subscription<autopilot_interface_msgs::msg::OffboardFlag>::SharedPtr offboard_flag_sub_;

    // Perception subscribers
    rclcpp::Subscription<ground_system_msgs::msg::SwarmObs>::SharedPtr ground_tracks_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_detections_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kiss_odometry_sub_;

    // Subscribers variables
    double lat_, lon_, alt_, alt_ellipsoid_;
    double x_, y_, z_, vx_, vy_, vz_;
    double ref_lat_, ref_lon_, ref_alt_;
    std::array<float, 3> position_;
    std::array<float, 4> q_;
    std::array<float, 3> velocity_;
    std::array<float, 3> angular_velocity_;
    double true_airspeed_m_s_, heading_;
    std::array<float, 3> kiss_position_;
    std::array<float, 4> kiss_q_;

    // MAVROS publishers
    rclcpp::Publisher<Vector3Stamped>::SharedPtr setpoint_accel_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr setpoint_vel_pub_; // Or drone frame message rclcpp::Publisher<Twist>::SharedPtr setpoint_vel_local_pub_;

    // Callbacks for timers
    void ardupilot_interface_printout_callback();
    void offboard_loop_callback();

    // Callbacks for MAVROS subscribers
    void global_position_global_sub_callback(const NavSatFix::SharedPtr msg);
    void local_position_odom_callback(const Odometry::SharedPtr msg);
    void global_position_local_callback(const Odometry::SharedPtr msg);
    void vfr_hud_callback(const VfrHud::SharedPtr msg);

    // Offboard flag call back
    void offboard_flag_callaback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg);

    // Callbacks for perception subscribers
    void ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg);
    void yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void kiss_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // OFFBOARD_CONTROL__ARDUPILOT_GUIDED_HPP_
