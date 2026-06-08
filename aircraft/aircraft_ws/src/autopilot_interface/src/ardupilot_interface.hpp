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
#include <string>

#include <rclcpp/clock.hpp>
#include <rclcpp/parameter.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <GeographicLib/Geodesic.hpp>

#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <mavros_msgs/msg/home_position.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/vehicle_info.hpp>
#include <mavros_msgs/msg/vfr_hud.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>

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

#include "autopilot_interface_msgs/msg/offboard_flag.hpp"

#include "autopilot_interface_msgs/srv/set_speed.hpp"
#include "autopilot_interface_msgs/srv/set_reposition.hpp"

#include "autopilot_interface_msgs/action/land.hpp"
#include "autopilot_interface_msgs/action/offboard.hpp"
#include "autopilot_interface_msgs/action/orbit.hpp"
#include "autopilot_interface_msgs/action/takeoff.hpp"

using namespace mavros_msgs::msg;
using namespace mavros_msgs::srv;
using namespace nav_msgs::msg;
using namespace sensor_msgs::msg;
using namespace GeographicLib;
using namespace geographic_msgs::msg;
using namespace geometry_msgs::msg;
using namespace std::chrono_literals; // for time literals (e.g. 1s)

enum class ArdupilotInterfaceState {
    STARTED,
    GUIDED_PRETAKEOFF,
    ARMED,
    MC_HOVER,
    VTOL_QLOITER_PRETAKEOFF,
    VTOL_TAKEOFF_MC,
    VTOL_TAKEOFF_HEADING,
    VTOL_TAKEOFF_TRANSITION,
    VTOL_TAKEOFF_MISSION_UPLOADED,
    VTOL_TAKEOFF_MISSION_WP_SET,
    VTOL_TAKEOFF_AUTO_MODE,
    FW_CRUISE,
    MC_ORBIT,
    MC_ORBIT_MISSION_UPLOADED,
    MC_ORBIT_MISSION_WP_SET,
    MC_ORBIT_AUTO_MODE,
    MC_ORBIT_TRANSFER,
    VTOL_ORBIT_MISSION_UPLOADED,
    VTOL_ORBIT_MISSION_WP_SET,
    VTOL_ORBIT_AUTO_MODE,
    VTOL_ORBIT_MISSION_COMPLETED,
    MC_RTL_PARAM_SET,
    MC_RTL,
    MC_RETURNED_READY_TO_LAND,
    MC_LANDING,
    VTOL_LANDING_MISSION_UPLOADED,
    VTOL_LANDING_MISSION_WP_SET,
    VTOL_LANDING_AUTO_MODE,
    VTOL_LANDING_READY_FOR_QRTL,
    VTOL_QRTL_PARAM_SET,
    VTOL_QRTL,
    LANDED,
    OFFBOARD_VELOCITY,
    OFFBOARD_ACCELERATION
};

class ArdupilotInterface : public rclcpp::Node
{
public:
    ArdupilotInterface();

private:
    // Constants - Reposition service
    static constexpr int REPOSITION_REQ_DELAY_MS = 100; // Delay in ms between repeated change mode requests and repeated target pose publications
    static constexpr int REPOSITION_PUB_RETRIES = 3; // Number of time the target pose is published
    // Constants - Action Handle Accepted (Landing, Offboard, Orbit, Takeoff)
    static constexpr int ACTION_LOOP_RATE_HZ = 100; // Frequency of the while loops in long duration action handles for takeoff, landing, orbit, and offboard
    static constexpr double ACTION_REQ_DELAY_SEC = 1.0; // Delay between successive service requests in long duration action handles
    // Constants - Landing
    static constexpr double MC_LAND_INIT_DIST_THRESH = 5.0; // Distance (m) from home, for a multicopter, to start the final landing descent
    static constexpr double VTOL_LAND_LOITER_DIST = 300.0; // Distance (m) from home, for a VTOL, of the pre-landing loiter descent
    static constexpr double VTOL_LAND_LOITER_RADIUS = 150.0; // Radius (m), for a VTOL, of the pre-landing loiter descent
    static constexpr double VTOL_LAND_LOITER_ALT = 150.0; // Initial altitude (m), for a VTOL, of the pre-landing loiter descent
    static constexpr double VTOL_LAND_LOITER_EXIT_DIST_THRESH = 30.0; // Threshold (m) in X-Y to exit the pre-landing loiter descent
    static constexpr double VTOL_LAND_LOITER_EXIT_ALT_THRESH = 10.0; // Threshold (m) in Z to exit the pre-landing loiter descent
    static constexpr double VTOL_LAND_LOITER_EXIT_HEADING_THRESH = 10.0; // Threshold (deg) in desired approach heading to exit the pre-landing loiter descent
    static constexpr double LAND_COMPLETED_ALT_THRESH = 2.0; // Altitude (m) from home, for a multicopter or VTOL, to consider the landing action complete
    // Constants - Orbit
    static constexpr int ORBIT_MIN_POINTS = 8; // Minimum number of points in a orbit
    static constexpr double ORBIT_POINT_SPACING = 15.0; // Target spacing (m) between points in a orbit
    // Quad tangential speed is determinted by WPNAV_SPEED 500 (in cm/s) in iris_with_ardupilot/ardupilot-4.6.params
    // Constants - Takeoff
    static constexpr double MC_TAKEOFF_COMPLETED_RATIO = 0.9; // Percentage of the target altitude, for a multicopter, to consider the takeoff action complete
    static constexpr double VTOL_TAKEOFF_ALT_THRESH = 2.0; // Altitude (m) to switch to state VTOL_TAKEOFF_HEADING (Unused)
    static constexpr double VTOL_TAKEOFF_TRANSITION_WAIT_SEC = 10.0; // Time in seconds to wait in CRUISE mode before sending the VTOL takeoff loiter mission
    static constexpr double VTOL_TAKEOFF_LOITER_RADIUS = 200.0; // Radius (m), for a VTOL, of the post-takeoff loiter

    std::shared_mutex node_data_mutex_;
    const GeographicLib::Geodesic& geod = GeographicLib::Geodesic::WGS84();

    // Node variables
    ArdupilotInterfaceState aircraft_fsm_state_;
    std::atomic<bool> active_srv_or_act_flag_;
    double home_lat_, home_lon_, home_alt_; // Saved on takeoff
    int offboard_flag_frequency;
    std::atomic<int> offboard_flag_count_;
    std::atomic<int> last_offboard_flag_count_;
    rclcpp::Time last_offboard_flag_rate_check_time_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr callback_group_timer_;
    rclcpp::CallbackGroup::SharedPtr callback_group_subscriber_;
    rclcpp::CallbackGroup::SharedPtr callback_group_service_;
    rclcpp::CallbackGroup::SharedPtr callback_group_action_;

    // Node timers
    rclcpp::TimerBase::SharedPtr ardupilot_interface_printout_timer_;
    rclcpp::TimerBase::SharedPtr offboard_flag_timer_;

    // MAVROS subscribers
    rclcpp::Subscription<NavSatFix>::SharedPtr mavros_global_position_global_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr mavros_local_position_odom_sub_;
    rclcpp::Subscription<Odometry>::SharedPtr mavros_global_position_local_sub_;
    rclcpp::Subscription<TwistStamped>::SharedPtr mavros_local_position_vel_local_sub_;
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
    double x_, y_, z_, vx_, vy_, vz_, ve_, vn_, vu_;
    double ref_lat_, ref_lon_, ref_alt_;
    std::array<float, 3> position_;
    std::array<float, 4> q_;
    std::array<float, 3> velocity_;
    std::array<float, 3> angular_velocity_;
    double true_airspeed_m_s_, heading_;

    // MAVROS publishers
    rclcpp::Publisher<GeoPoseStamped>::SharedPtr setpoint_pos_pub_;

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
    void ardupilot_interface_printout_callback();
    void offboard_flag_callback();

    // Callbacks for MAVROS subscribers
    void global_position_global_sub_callback(const NavSatFix::SharedPtr msg);
    void local_position_odom_callback(const Odometry::SharedPtr msg);
    void global_position_local_callback(const Odometry::SharedPtr msg);
    void local_position_vel_local_callback(const TwistStamped::SharedPtr msg);
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
    
    void abort_action();

    // Transformations
    std::pair<double, double> lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset);
    std::pair<double, double> lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear);

    // Utility
    std::string fsm_state_to_string(ArdupilotInterfaceState state);

    // Template for service calls and FSM updates
    template<typename ServiceT, typename ActionT>
    void call_service_and_update_fsm(
        typename rclcpp::Client<ServiceT>::SharedPtr client, typename ServiceT::Request::SharedPtr request,
        std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle, const std::string& feedback_str,
        ArdupilotInterfaceState next_state)
    {
        auto feedback = std::make_shared<typename ActionT::Feedback>();
        feedback->message = feedback_str;
        goal_handle->publish_feedback(feedback);
        client->async_send_request(request,
            [this, feedback, goal_handle, next_state, feedback_str](typename rclcpp::Client<ServiceT>::SharedFuture future) {
                bool success = false;
                if constexpr (std::is_same_v<ServiceT, mavros_msgs::srv::SetMode>) {
                    success = future.get()->mode_sent; // The success field is named differently in the SetMode service
                } else {
                    success = future.get()->success;
                }

                if (success) {
                    feedback->message = feedback_str + " success";
                    goal_handle->publish_feedback(feedback);
                    std::unique_lock<std::shared_mutex> lock(node_data_mutex_);
                    aircraft_fsm_state_ = next_state;
                } else {
                    feedback->message = feedback_str + " failed";
                    goal_handle->publish_feedback(feedback);
                }
            });
    }
};

#endif // AUTOPILOT_INTERFACE__ARDUPILOT_INTERFACE_HPP_
