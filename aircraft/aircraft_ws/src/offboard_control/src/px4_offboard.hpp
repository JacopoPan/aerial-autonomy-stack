#ifndef OFFBOARD_CONTROL__PX4_OFFBOARD_HPP_
#define OFFBOARD_CONTROL__PX4_OFFBOARD_HPP_

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

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <GeographicLib/Geodesic.hpp>
#include <GeographicLib/LocalCartesian.hpp>

#include "geometry_msgs/msg/vector3.hpp"

#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "autopilot_interface_msgs/msg/offboard_flag.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include "ground_system_msgs/msg/swarm_obs.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

using namespace px4_msgs::msg;
using namespace GeographicLib;
using namespace std::chrono_literals; // For time literals (e.g. 1s)

class PX4Offboard : public rclcpp::Node
{
public:
    PX4Offboard();

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
    rclcpp::TimerBase::SharedPtr px4_interface_printout_timer_;
    rclcpp::TimerBase::SharedPtr offboard_control_loop_timer_;

    // PX4 subscribers
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<AirspeedValidated>::SharedPtr airspeed_validated_sub_;

    // Offboard flag subscriber
    rclcpp::Subscription<autopilot_interface_msgs::msg::OffboardFlag>::SharedPtr offboard_flag_sub_;

    // Perception subscribers
    rclcpp::Subscription<ground_system_msgs::msg::SwarmObs>::SharedPtr ground_tracks_sub_;
    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr yolo_detections_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr kiss_odometry_sub_;

    // Subscribers variables
    double lat_, lon_, alt_, alt_ellipsoid_;
    bool xy_valid_, z_valid_, v_xy_valid_, v_z_valid_, xy_global_, z_global_;
    double x_, y_, z_, heading_, vx_, vy_, vz_;
    double ref_lat_, ref_lon_, ref_alt_;
    int pose_frame_, velocity_frame_;
    std::array<float, 3> position_;
    std::array<float, 4> q_;
    std::array<float, 3> velocity_;
    std::array<float, 3> angular_velocity_;
    double true_airspeed_m_s_;
    std::array<float, 3> kiss_position_;
    std::array<float, 4> kiss_q_;
    ground_system_msgs::msg::SwarmObs::SharedPtr ground_tracks_;
    vision_msgs::msg::Detection2DArray::SharedPtr yolo_detections_;

    // Guidance variables
    double traj_ref_east_, traj_ref_north_, traj_ref_up_;
    double target_vn_, target_ve_, target_vd_;
    rclcpp::Time last_track_time_;

    // PX4 publishers
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
    rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_ref_pub_;
    rclcpp::Publisher<VehicleRatesSetpoint>::SharedPtr rates_ref_pub_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_ref_pub_;

    // Callbacks for timers
    void px4_interface_printout_callback();
    void offboard_loop_callback();

    // Callbacks for PX4 subscribers
    void global_position_callback(const VehicleGlobalPosition::SharedPtr msg);
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void odometry_callback(const VehicleOdometry::SharedPtr msg);
    void airspeed_callback(const AirspeedValidated::SharedPtr msg);

    // Offboard flag call back
    void offboard_flag_callaback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg);

    // Callbacks for perception subscribers
    void ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg);
    void yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
    void kiss_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};

#endif // OFFBOARD_CONTROL__PX4_OFFBOARD_HPP_
