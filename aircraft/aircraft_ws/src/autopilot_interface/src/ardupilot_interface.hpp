/*

TODO

# TAKEOFF AND LANDING ACTIONS (quad parameters examples)

python3 /aircraft_resources/patches/cancellable_action.py "ros2 action send_goal /Drone1/takeoff_action autopilot_interface_msgs/action/Takeoff '{takeoff_altitude: 40.0}'"
python3 /aircraft_resources/patches/cancellable_action.py "ros2 action send_goal /Drone1/land_action autopilot_interface_msgs/action/Land '{landing_altitude: 60.0}'"

# TAKEOFF AND LANDING ACTIONS (vtol parameters example)

python3 /aircraft_resources/patches/cancellable_action.py "ros2 action send_goal /Drone1/takeoff_action autopilot_interface_msgs/action/Takeoff '{takeoff_altitude: 40.0, vtol_transition_heading: 330.0, vtol_loiter_nord: 200.0, vtol_loiter_east: 100.0, vtol_loiter_alt: 120.0}'"
python3 /aircraft_resources/patches/cancellable_action.py "ros2 action send_goal /Drone1/land_action autopilot_interface_msgs/action/Land '{landing_altitude: 60.0, vtol_transition_heading: 60.0}'"

# ORBIT AND OFFBOARD (refs: attitude = 0, rates = 1, trajectory = 2) ACTIONS 

python3 /aircraft_resources/patches/cancellable_action.py "ros2 action send_goal /Drone1/orbit_action autopilot_interface_msgs/action/Orbit '{east: 500.0, north: 0.0, altitude: 150.0, radius: 200.0}'"
python3 /aircraft_resources/patches/cancellable_action.py "ros2 action send_goal /Drone1/offboard_action autopilot_interface_msgs/action/Offboard '{offboard_setpoint_type: 1, max_duration_sec: 2.0}'"

# SET SPEED (always limited by the autopilot params, for quads applies from the next command) and REPOSITION (quad only) SERVICES

ros2 service call /Drone1/set_speed autopilot_interface_msgs/srv/SetSpeed '{speed: 15.0}'
ros2 service call /Drone1/set_reposition autopilot_interface_msgs/srv/SetReposition '{east: 100.0, north: 200.0, altitude: 60.0}' # relative to Home

*/
#ifndef AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_
#define AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_

#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <shared_mutex>
#include <memory>
#include <atomic>
#include <array>
#include <algorithm>

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

#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/vehicle_info.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>

#include <mavros_msgs/srv/vehicle_info_get.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/waypoint_push.hpp>
#include <mavros_msgs/srv/waypoint_set_current.hpp>

#include <nav_msgs/msg/odometry.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>

#include "autopilot_interface_msgs/srv/set_speed.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"

#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/action/orbit.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"

using namespace geometry_msgs::msg;
using namespace mavros_msgs::msg;
using namespace mavros_msgs::srv;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace GeographicLib;
using namespace geographic_msgs::msg;
using namespace std::chrono_literals;  // for time literals (e.g. 1s)

enum class ArdupilotInterfaceState {
    STARTED,
    GUIDED_PRETAKEOFF,
    ARMED,
    // MC_TAKEOFF,
    MC_HOVER,
    MC_ORBIT,
    VTOL_TAKEOFF_MC,
    VTOL_TAKEOFF_TRANSITION,
    VTOL_TAKEOFF_MISSION_UPLOADED,
    VTOL_TAKEOFF_MISSION_MODE,
    VTOL_TAKEOFF_MISSION_STARTED,
    FW_CRUISE,
    // FW_LANDING_LOITER,
    // FW_LANDING_DESCENT,
    // FW_LANDING_APPROACH,
    // VTOL_LANDING_TRANSITION,
    // RTL,
    // MC_LANDING,
    // OFFBOARD_ATTITUDE,
    // OFFBOARD_RATES,
    // OFFBOARD_TRAJECTORY
};

class ArdupilotInterface : public rclcpp::Node
{
public:
    ArdupilotInterface();

private:
    std::shared_mutex node_data_mutex_;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // Node variables
    ArdupilotInterfaceState aircraft_fsm_state_;
    std::atomic<bool> active_srv_or_act_flag_;
    double home_lat_, home_lon_, home_alt_; // Saved on takeoff
    int offboard_loop_frequency;
    std::atomic<int> offboard_loop_count_;
    std::atomic<int> last_offboard_loop_count_;
    rclcpp::Time last_offboard_rate_check_time_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_action_;

    // Node timers
    rclcpp::TimerBase::SharedPtr ardupilot_interface_printout_timer_;
    rclcpp::TimerBase::SharedPtr offboard_control_loop_timer_;

    // MAVROS subscribers
    rclcpp::Subscription<NavSatFix>::SharedPtr mavros_global_position_global_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr mavros_local_position_odom_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr mavros_global_position_local_sub_;
    rclcpp::Subscription<VfrHud>::SharedPtr mavros_vfr_hud_sub_;
    rclcpp::Subscription<HomePosition>::SharedPtr mavros_home_position_home_sub_;
    rclcpp::Subscription<State>::SharedPtr mavros_state_sub_;

    // MAVROS service clients
    rclcpp::Client<VehicleInfoGet>::SharedPtr vehicle_info_client_;
    rclcpp::Client<CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<CommandLong>::SharedPtr command_long_client_;
    rclcpp::Client<CommandTOL>::SharedPtr takeoff_client_;
    rclcpp::Client<CommandTOL>::SharedPtr landing_client_;
    rclcpp::Client<ParamSetV2>::SharedPtr set_param_client_;
    rclcpp::Client<SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<WaypointPush>::SharedPtr wp_push_client_;
    rclcpp::Client<WaypointSetCurrent>::SharedPtr set_wp_client_;

    // Subscribers variables
    int target_system_id_, mav_state_, mav_type_;
    bool armed_flag_;
    std::string ardupilot_mode_;
    double lat_, lon_, alt_, alt_ellipsoid_;
    double x_, y_, z_, vx_, vy_, vz_;
    double ref_lat_, ref_lon_, ref_alt_;
    std::array<float, 3> position_;
    std::array<float, 4> q_;
    std::array<float, 3> velocity_;
    std::array<float, 3> angular_velocity_;
    double true_airspeed_m_s_, heading_;

    // MAVROS publishers
    rclcpp::Publisher<Vector3Stamped>::SharedPtr setpoint_accel_pub_;
    rclcpp::Publisher<TwistStamped>::SharedPtr setpoint_vel_pub_; // Drone frame message rclcpp::Publisher<Twist>::SharedPtr setpoint_vel_pub_;
    rclcpp::Publisher<GeoPoseStamped>::SharedPtr setpoint_pos_pub_; // Cartesian/local message rclcpp::Publisher<PoseStamped>::SharedPtr setpoint_pos_pub_;

    // Node Services
    rclcpp::Service<autopilot_interface_msgs::srv::SetSpeed>::SharedPtr set_speed_service_;
    rclcpp::Service<autopilot_interface_msgs::srv::SetReposition>::SharedPtr set_reposition_service_;

    // Node Actions
    rclcpp_action::Server<autopilot_interface_msgs::action::Land>::SharedPtr land_action_server_;
    rclcpp_action::Server<autopilot_interface_msgs::action::Offboard>::SharedPtr offboard_action_server_;
    rclcpp_action::Server<autopilot_interface_msgs::action::Orbit>::SharedPtr orbit_action_server_;
    rclcpp_action::Server<autopilot_interface_msgs::action::Takeoff>::SharedPtr takeoff_action_server_;

    // Callbacks for timers
    void ardupilot_interface_printout_callback();
    void offboard_control_loop_callback();

    // Callbacks for MAVROS subscribers
    void global_position_global_sub_callback(const NavSatFix::SharedPtr msg);
    void local_position_odom_callback(const Odometry::SharedPtr msg);
    void global_position_local_callback(const Odometry::SharedPtr msg);
    void vfr_hud_callback(const VfrHud::SharedPtr msg);
    void home_position_home_callback(const HomePosition::SharedPtr msg);
    void state_callback(const State::SharedPtr msg);

    // Callbacks for non-blocking services
    void set_speed_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Request> request,
                            std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Response> response);
    void set_reposition_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Request> request,
                            std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Response> response);

    // Callbacks for actions
    rclcpp_action::GoalResponse land_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Land::Goal> goal);
    rclcpp_action::CancelResponse land_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle);
    void land_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle);
    //
    rclcpp_action::GoalResponse offboard_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Offboard::Goal> goal);
    rclcpp_action::CancelResponse offboard_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle);
    void offboard_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle);
    //
    rclcpp_action::GoalResponse orbit_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Orbit::Goal> goal);
    rclcpp_action::CancelResponse orbit_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle);
    void orbit_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle);
    //
    rclcpp_action::GoalResponse takeoff_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Takeoff::Goal> goal);
    rclcpp_action::CancelResponse takeoff_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle);
    void takeoff_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle);
    
    // Transformations
    std::pair<double, double> lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset);
    std::pair<double, double> lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear);

};

#endif // AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_
