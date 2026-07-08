#ifndef AUTOPILOT_INTERFACE__PX4_INTERFACE_HPP_
#define AUTOPILOT_INTERFACE__PX4_INTERFACE_HPP_

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

#include "geometry_msgs/msg/vector3.hpp"

#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/vehicle_command_ack.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/airspeed_validated.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <px4_msgs/msg/vehicle_command.hpp>

#include "autopilot_interface_msgs/msg/offboard_flag.hpp"

#include "autopilot_interface_msgs/srv/set_speed.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"

#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/action/orbit.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"

using namespace px4_msgs::msg;
using namespace GeographicLib;
using namespace std::chrono_literals; // For time literals (e.g. 1s)

enum class PX4InterfaceState {
    STARTED,
    MC_TAKEOFF,
    MC_HOVER,
    MC_ORBIT,
    VTOL_TAKEOFF_TRANSITION,
    FW_CRUISE,
    FW_LANDING_LOITER,
    FW_LANDING_DESCENT,
    FW_LANDING_APPROACH,
    VTOL_LANDING_TRANSITION,
    RTL,
    MC_LANDING,
    OFFBOARD
};

class PX4Interface : public rclcpp::Node
{
public:
    PX4Interface();

private:
    // Parameters - Action Handle Accepted (Landing, Offboard, Orbit, Takeoff)
    int ACTION_LOOP_RATE_HZ;
    // Parameters - Landing
    double LAND_INIT_DIST_THRESH, VTOL_LAND_LOITER_DIST, VTOL_LAND_LOITER_RADIUS, VTOL_LAND_LOITER_ALT_HIGH, VTOL_LAND_LOITER_ALT_LOW;
    double VTOL_LAND_LOITER_STARTED_DIST_THRESH, VTOL_LAND_LOITER_EXIT_DIST_THRESH, VTOL_LAND_LOITER_EXIT_ALT_THRESH, VTOL_LAND_FAKE_REPOSITION_DISTANCE;
    double VTOL_LAND_TRANSITION_START_DISTANCE, TAIL_LAND_TRANSITION_START_DISTANCE;
    // Parameters - Orbit
    double MC_ORBIT_SPEED_MS;
    // Parameters - Takeoff
    double MC_TAKEOFF_COMPLETED_RATIO, VTOL_TAKEOFF_TRANSITION_WAIT_SEC, VTOL_TAKEOFF_LOITER_RADIUS;
    // Parameters - Abort Action Handle (Landing, Offboard, Orbit, Takeoff)
    double ABORT_REPOSITION_ALT;

    std::shared_mutex node_data_mutex_;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // Node variables
    PX4InterfaceState aircraft_fsm_state_;
    std::atomic<bool> active_srv_or_act_flag_;
    double home_lat_, home_lon_, home_alt_; // Saved on takeoff
    int offboard_flag_frequency;
    std::atomic<int> offboard_flag_count_;
    std::atomic<int> last_offboard_flag_count_;
    rclcpp::Time last_offboard_flag_rate_check_time_;
    std::string active_offboard_controller_name_;
    static constexpr uint64_t UNSET_TIME_US = std::numeric_limits<uint64_t>::max();

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_printout_;
    rclcpp::CallbackGroup::SharedPtr callback_group_offboard_control_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_action_;

    // Node timers
    rclcpp::TimerBase::SharedPtr px4_interface_printout_timer_;
    rclcpp::TimerBase::SharedPtr offboard_flag_timer_;

    // PX4 subscribers
    rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr vehicle_global_position_sub_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr vehicle_local_position_sub_;
    rclcpp::Subscription<VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    rclcpp::Subscription<VehicleStatus>::SharedPtr vehicle_status_sub_;
    rclcpp::Subscription<AirspeedValidated>::SharedPtr airspeed_validated_sub_;
    rclcpp::Subscription<VehicleCommandAck>::SharedPtr vehicle_command_ack_sub_;

    // Subscribers variables
    std::atomic<int> target_system_id_;
    int arming_state_, vehicle_type_;
    bool is_vtol_, is_vtol_tailsitter_, in_transition_mode_, in_transition_to_fw_, pre_flight_checks_pass_;
    double lat_, lon_, alt_, alt_ellipsoid_;
    bool xy_valid_, z_valid_, v_xy_valid_, v_z_valid_, xy_global_, z_global_;
    double x_, y_, z_, heading_, vx_, vy_, vz_;
    double ref_lat_, ref_lon_, ref_alt_;
    int pose_frame_, velocity_frame_;
    // float to match PX4 float32 VehicleOdometry
    std::array<float, 3> position_;
    std::array<float, 4> q_;
    std::array<float, 3> velocity_;
    std::array<float, 3> angular_velocity_;
    double true_airspeed_m_s_;
    int command_ack_;
    int command_ack_result_;
    bool command_ack_from_external_;

    // PX4 publishers
    rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;

    // Offboard flag publisher
    rclcpp::Publisher<autopilot_interface_msgs::msg::OffboardFlag>::SharedPtr offboard_flag_pub_;

    // Node Services
    rclcpp::Service<autopilot_interface_msgs::srv::SetSpeed>::SharedPtr set_speed_service_;
    rclcpp::Service<autopilot_interface_msgs::srv::SetReposition>::SharedPtr set_reposition_service_;

    // Node Actions
    rclcpp_action::Server<autopilot_interface_msgs::action::Land>::SharedPtr land_action_server_;
    rclcpp_action::Server<autopilot_interface_msgs::action::Offboard>::SharedPtr offboard_action_server_;
    rclcpp_action::Server<autopilot_interface_msgs::action::Orbit>::SharedPtr orbit_action_server_;
    rclcpp_action::Server<autopilot_interface_msgs::action::Takeoff>::SharedPtr takeoff_action_server_;

    // Callbacks for timers
    void px4_interface_printout_callback();
    void offboard_flag_callback();

    // Callbacks for PX4 subscribers
    void global_position_callback(const VehicleGlobalPosition::SharedPtr msg);
    void local_position_callback(const VehicleLocalPosition::SharedPtr msg);
    void odometry_callback(const VehicleOdometry::SharedPtr msg);
    void status_callback(const VehicleStatus::SharedPtr msg);
    void airspeed_callback(const AirspeedValidated::SharedPtr msg);
    void vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg);

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

    // vehicle_command methods
    void do_takeoff(double alt, double yaw, bool is_vtol, double home_lat, double home_lon, double home_alt);
    void do_orbit(double lat, double lon, double alt, double r, double speed, double home_alt);
    void do_change_altitude(double alt, double home_alt);
    void do_change_speed(double speed);
    void do_reposition(double lat, double lon, double alt, double heading, double home_alt);
    void do_vtol_transition(int trans_type);
    void do_rtl();
    void do_land(double home_lat, double home_lon, double home_alt);
    void do_set_mode(int mode, int submode);
    void send_vehicle_command(int command, double param1 = 0.0, double param2 = 0.0, double param3 = 0.0, 
                                double param4 = 0.0, double param5 = 0.0, double param6 = 0.0, double param7 = 0.0, 
                                int conf = 0);
    void abort_action();
    
    // Transformations
    std::pair<double, double> lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset);
    std::pair<double, double> lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear);

    // Utility
    std::string fsm_state_to_string(PX4InterfaceState state);
};

#endif // AUTOPILOT_INTERFACE__PX4_INTERFACE_HPP_
