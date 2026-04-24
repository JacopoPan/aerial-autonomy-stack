#include "px4_interface.hpp"

PX4Interface::PX4Interface() : Node("px4_interface"),
    active_srv_or_act_flag_(false), aircraft_fsm_state_(PX4InterfaceState::STARTED),
    offboard_flag_frequency(10), offboard_flag_count_(0), last_offboard_flag_count_(0),
    target_system_id_(-1), arming_state_(-1), vehicle_type_(-1),
    is_vtol_(false), is_vtol_tailsitter_(false), in_transition_mode_(false), in_transition_to_fw_(false), pre_flight_checks_pass_(false),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    xy_valid_(false), z_valid_(false), v_xy_valid_(false), v_z_valid_(false), xy_global_(false), z_global_(false),
    x_(NAN), y_(NAN), z_(NAN), heading_(NAN), vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    pose_frame_(-1), velocity_frame_(-1), true_airspeed_m_s_(NAN),
    command_ack_(-1), command_ack_result_(-1), command_ack_from_external_(false),
    home_lat_(NAN), home_lon_(NAN), home_alt_(NAN)
{
    RCLCPP_INFO(this->get_logger(), "PX4 interfacing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    // Check and log whether simulation time is enabled or not
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
    } else {
        RCLCPP_WARN(this->get_logger(), "Simulation time is disabled.");
    }
    last_offboard_flag_rate_check_time_ = this->get_clock()->now(); // Monitor the rate of offboard flag
    // Initialize the arrays
    position_.fill(NAN);
    q_.fill(NAN);
    velocity_.fill(NAN);
    angular_velocity_.fill(NAN);

    // PX4 publishers
    rclcpp::QoS qos_profile_pub(10);  // Depth of 10
    qos_profile_pub.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Or rclcpp::DurabilityPolicy::Volatile
    command_pub_ = this->create_publisher<VehicleCommand>("fmu/in/vehicle_command", qos_profile_pub);

    // Offboard flag publisher
    offboard_flag_pub_ = this->create_publisher<autopilot_interface_msgs::msg::OffboardFlag>("/offboard_flag", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Timed callbacks in parallel
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel
    callback_group_service_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Services are parallel but refused if active_srv_or_act_flag_ is true
    callback_group_action_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Actions are parallel but refused if active_srv_or_act_flag_ is true

    // Timers
    px4_interface_printout_timer_ = this->create_wall_timer( // Follow wall clock for printouts
        3s, // Timer period of 3 seconds
        std::bind(&PX4Interface::px4_interface_printout_callback, this),
        callback_group_timer_
    );
    offboard_flag_timer_ = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(1000000000 / offboard_flag_frequency),
        std::bind(&PX4Interface::offboard_flag_callback, this),
        callback_group_timer_
    );

    // Subscribers configuration
    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.callback_group = callback_group_subscriber_;
    rclcpp::QoS qos_profile_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile_sub.keep_last(10);  // History: KEEP_LAST with depth 10
    qos_profile_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // PX4 subscribers
    vehicle_global_position_sub_= this->create_subscription<VehicleGlobalPosition>(
        "fmu/out/vehicle_global_position", qos_profile_sub, // 100Hz
        std::bind(&PX4Interface::global_position_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_local_position_sub_= this->create_subscription<VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", qos_profile_sub, // 100Hz
        std::bind(&PX4Interface::local_position_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_odometry_sub_= this->create_subscription<VehicleOdometry>(
        "fmu/out/vehicle_odometry", qos_profile_sub, // 100Hz
        std::bind(&PX4Interface::odometry_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
        "fmu/out/vehicle_status_v1", qos_profile_sub, // 2Hz
        std::bind(&PX4Interface::status_callback, this, std::placeholders::_1), subscriber_options);
    airspeed_validated_sub_ = this->create_subscription<AirspeedValidated>(
        "fmu/out/airspeed_validated", qos_profile_sub, // 10Hz
        std::bind(&PX4Interface::airspeed_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_command_ack_sub_ = this->create_subscription<VehicleCommandAck>(
        "fmu/out/vehicle_command_ack", qos_profile_sub, // n/a
        std::bind(&PX4Interface::vehicle_command_ack_callback, this, std::placeholders::_1), subscriber_options);

    // Services
    set_speed_service_ = this->create_service<autopilot_interface_msgs::srv::SetSpeed>(
        "set_speed", std::bind(&PX4Interface::set_speed_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_);
    set_reposition_service_ = this->create_service<autopilot_interface_msgs::srv::SetReposition>(
        "set_reposition", std::bind(&PX4Interface::set_reposition_callback, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, callback_group_service_);

    // Actions
    land_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Land>(this, "land_action",
            std::bind(&PX4Interface::land_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PX4Interface::land_handle_cancel, this, std::placeholders::_1),
            std::bind(&PX4Interface::land_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    takeoff_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Takeoff>(this, "takeoff_action",
            std::bind(&PX4Interface::takeoff_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PX4Interface::takeoff_handle_cancel, this, std::placeholders::_1),
            std::bind(&PX4Interface::takeoff_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    offboard_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Offboard>(this, "offboard_action",
            std::bind(&PX4Interface::offboard_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PX4Interface::offboard_handle_cancel, this, std::placeholders::_1),
            std::bind(&PX4Interface::offboard_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
    orbit_action_server_ = rclcpp_action::create_server<autopilot_interface_msgs::action::Orbit>(this, "orbit_action",
            std::bind(&PX4Interface::orbit_handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&PX4Interface::orbit_handle_cancel, this, std::placeholders::_1),
            std::bind(&PX4Interface::orbit_handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), callback_group_action_);
}

// Callbacks for subscribers (reentrant group)
void PX4Interface::global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->lat;
    lon_ = msg->lon;
    alt_ = msg->alt; // AMSL
    alt_ellipsoid_ = msg->alt_ellipsoid; // TODO: double-check
    // New to v1.16: bool lat_lon_valid, bool alt_valid
}
void PX4Interface::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    xy_valid_ = msg->xy_valid;
    z_valid_ = msg->z_valid;
    v_xy_valid_ = msg->v_xy_valid;
    v_z_valid_ = msg->v_z_valid;
    // Position in local NED frame
    x_ = msg->x; // N
    y_= msg->y; // E
    z_ = msg->z; // D
    heading_ = msg->heading; // Euler yaw angle transforming the tangent plane relative to NED earth-fixed frame, -PI..+PI,  (radians)
    // Velocity in NED frame
    vx_ = msg->vx;
    vy_ = msg->vy;
    vz_ = msg->vz;
    // Position of reference point (local NED frame origin) in global (GPS / WGS84) frame
    xy_global_ = msg->xy_global; // Validity of reference
    z_global_ = msg->z_global; // Validity of reference
    ref_lat_ = msg->ref_lat;
    ref_lon_ = msg->ref_lon;
    ref_alt_ = msg->ref_alt; // AMSL
}
void PX4Interface::odometry_callback(const VehicleOdometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    pose_frame_ = msg->pose_frame; // 1:  NED earth-fixed frame, 2: FRD world-fixed frame, arbitrary heading
    velocity_frame_ = msg->velocity_frame; // 1:  NED earth-fixed frame, 2: FRD world-fixed frame, arbitrary heading, 3: FRD body-fixed frame
    position_ = msg->position;
    q_ = msg->q;
    velocity_ = msg->velocity;
    angular_velocity_ = msg->angular_velocity;
}
void PX4Interface::status_callback(const VehicleStatus::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    if (target_system_id_ == -1) {
        target_system_id_ = msg->system_id; // get target_system_id from PX4's MAV_SYS_ID once
        RCLCPP_WARN(get_logger(), "target_system_id (MAV_SYS_ID) saved as: %d", target_system_id_);
    }
    arming_state_ = msg->arming_state; // DISARMED = 1, ARMED = 2
    vehicle_type_ = msg->vehicle_type; // ROTARY_WING = 1, FIXED_WING = 2 (ROVER = 3)
    is_vtol_ = msg->is_vtol; // bool
    is_vtol_tailsitter_ = msg->is_vtol_tailsitter; // bool
    in_transition_mode_ = msg->in_transition_mode; // bool
    in_transition_to_fw_ = msg->in_transition_to_fw; // bool
    pre_flight_checks_pass_ = msg->pre_flight_checks_pass; // bool
    if ((aircraft_fsm_state_ != PX4InterfaceState::STARTED) && (arming_state_ == 1)) {
        aircraft_fsm_state_ = PX4InterfaceState::STARTED; // Reset PX4 interface state after a disarm (hoping the vehicle is ok)
    }
}
void PX4Interface::airspeed_callback(const AirspeedValidated::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    true_airspeed_m_s_ = msg->true_airspeed_m_s;
}
void PX4Interface::vehicle_command_ack_callback(const VehicleCommandAck::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    command_ack_ = msg->command;
    command_ack_result_ = msg->result;
    command_ack_from_external_ = msg->from_external;
}

// Callbacks for timers (reentrant group)
void PX4Interface::px4_interface_printout_callback()
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    auto now = this->get_clock()->now();
    double elapsed_sec = (now - last_offboard_flag_rate_check_time_).seconds();
    double actual_rate = NAN;
    if (elapsed_sec > 0) {
        actual_rate = (offboard_flag_count_ - last_offboard_flag_count_) / elapsed_sec;
    }
    last_offboard_flag_count_.store(offboard_flag_count_.load());
    last_offboard_flag_rate_check_time_ = now;
    RCLCPP_INFO(get_logger(),
                "Vehicle status:\n"
                "  target_system_id: %d\n"
                "  arming_state: %d\n"
                "  vehicle_type (MC:1, FW:2): %d\n"
                "  is_vtol: %s\n"
                "  is_vtol_tailsitter: %s\n"
                "  in_transition_mode: %s\n"
                "  in_transition_to_fw: %s\n"
                "  pre_flight_checks_pass: %s\n"
                "Global position:\n"
                "  lat: %.5f, lon: %.5f,\n"
                "  alt AMSL: %.2f, alt ell: %.2f\n"
                "Local position:\n"
                "  NED pos: %.2f %.2f %.2f\n"
                "  (XY valid %s, Z valid %s)\n"
                "  NED vel: %.2f %.2f %.2f\n"
                "  (V_XY valid %s, V_Z valid %s)\n"
                "  heading: %.2f\n"
                "  ref lat: %.5f, lon: %.5f, alt AMSL: %.2f\n"
                "  (ref XY valid %s, ref Z valid %s)\n"
                "Odometry:\n"
                "  pose_frame: %.d, velocity_frame %d\n"
                "  pos: %.2f %.2f %.2f\n"
                "  quaternion: %.2f %.2f %.2f %.2f\n"
                "  vel: %.2f %.2f %.2f\n"
                "  angular_vel: %.2f %.2f %.2f\n"
                "Airspeed validated:\n"
                "  true_airspeed_m_s: %.2f\n"
                "Command Ack:\n"
                "  command: %d (result %d from_external %s)\n"
                "Current node time:\n"
                "  %.2f seconds\n"
                "Current FSM State:\n"
                "  %s\n"
                "Offboard flag rate:\n"
                "  %.2f Hz\n\n",
                //
                target_system_id_, arming_state_, vehicle_type_,
                (is_vtol_ ? "true" : "false"),
                (is_vtol_tailsitter_ ? "true" : "false"),
                (in_transition_mode_ ? "true" : "false"),
                (in_transition_to_fw_ ? "true" : "false"),
                (pre_flight_checks_pass_ ? "true" : "false"),
                lat_, lon_, alt_, alt_ellipsoid_,
                x_, y_, z_, (xy_valid_ ? "true" : "false"), (z_valid_ ? "true" : "false"),
                vx_, vy_, vz_, (v_xy_valid_ ? "true" : "false"), (v_z_valid_ ? "true" : "false"),
                heading_ * 180.0 / M_PI,
                ref_lat_, ref_lon_, ref_alt_, (xy_global_ ? "true" : "false"), (z_global_ ? "true" : "false"),
                pose_frame_, velocity_frame_,
                position_[0], position_[1], position_[2],
                q_[0], q_[1], q_[2], q_[3],
                velocity_[0], velocity_[1], velocity_[2],
                angular_velocity_[0] * 180.0 / M_PI, angular_velocity_[1] * 180.0 / M_PI, angular_velocity_[2] * 180.0 / M_PI,
                true_airspeed_m_s_,
                command_ack_, command_ack_result_, (command_ack_from_external_ ? "true" : "false"),
                this->get_clock()->now().seconds(),
                fsm_state_to_string(aircraft_fsm_state_).c_str(),
                actual_rate
            );
}
void PX4Interface::offboard_flag_callback()
{
    offboard_flag_count_++; // Counter to monitor the rate of the offboard flag (no lock, atomic variable)

    auto msg = autopilot_interface_msgs::msg::OffboardFlag();
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if (aircraft_fsm_state_ == PX4InterfaceState::OFFBOARD_ATTITUDE) {
        msg.offboard_flag = is_vtol_ ? 2 : 1;
    } else if (aircraft_fsm_state_ == PX4InterfaceState::OFFBOARD_RATES) {
        msg.offboard_flag = is_vtol_ ? 4 : 3;
    } else if (aircraft_fsm_state_ == PX4InterfaceState::OFFBOARD_TRAJECTORY) {
        msg.offboard_flag = is_vtol_ ? 6 : 5;
    } else {
        msg.offboard_flag = 0; // Inactive
    }
    offboard_flag_pub_->publish(msg);
}

// Callbacks for non-blocking services (reentrant callback group, active_srv_or_act_flag_ acting as semaphore)
void PX4Interface::set_speed_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Request> request,
                        std::shared_ptr<autopilot_interface_msgs::srv::SetSpeed::Response> response)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if ((!is_vtol_ && aircraft_fsm_state_ != PX4InterfaceState::MC_HOVER) || (is_vtol_ && aircraft_fsm_state_ != PX4InterfaceState::FW_CRUISE)) {
        response->message = "Set speed rejected, PX4Interface is not in hover/cruise state";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        response->success = false;
        return;
    }
    if (active_srv_or_act_flag_.exchange(true)) {
        response->message = "Another service/action is active";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        response->success = false;
        return;
    }
    if (!is_vtol_) {
        RCLCPP_WARN(this->get_logger(), "For quads, the change of speed will affect the next (e.g. /set_reposition) service/action");
    }
    RCLCPP_INFO(this->get_logger(), "New requested speed is: %.2f", request->speed);
    do_change_speed(request->speed);
    response->success = true;
    response->message = "set_speed request sent";
    active_srv_or_act_flag_.store(false);
}
void PX4Interface::set_reposition_callback(const std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Request> request,
                        std::shared_ptr<autopilot_interface_msgs::srv::SetReposition::Response> response)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if ((is_vtol_) || (!is_vtol_ && !(aircraft_fsm_state_ == PX4InterfaceState::MC_HOVER || aircraft_fsm_state_ == PX4InterfaceState::MC_ORBIT))) {
        response->message = "Set reposition rejected, PX4Interface is not in a quad hover/orbit state (for VTOLs, use /orbit_action)";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        response->success = false;
        return;
    }
    if (active_srv_or_act_flag_.exchange(true)) {
        response->message = "Another service/action is active";
        RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
        response->success = false;
        return;
    }
    if (aircraft_fsm_state_ == PX4InterfaceState::MC_ORBIT) {
        do_set_mode(4, 3); // If in an Orbit mode, switch to Hold mode
        aircraft_fsm_state_ = PX4InterfaceState::MC_HOVER;
    }
    double desired_east = request->east;
    double desired_north = request->north;
    double desired_alt = request->altitude;
    RCLCPP_INFO(this->get_logger(), "New requested reposition East-North %.2f %.2f Alt. %.2f", desired_east, desired_north, desired_alt);
    auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
    double distance, heading;
    geod.Inverse(lat_, lon_, des_lat, des_lon, distance, heading);
    do_reposition(des_lat, des_lon, desired_alt, fmod(heading + 360.0, 360.0) / 180.0 * M_PI);
    response->success = true;
    response->message = "set_reposition request sent";
    active_srv_or_act_flag_.store(false);
}

// Callbacks for actions (reentrant callback group, to be able to handle_goal and handle_cancel at the same time)
rclcpp_action::GoalResponse PX4Interface::land_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Land::Goal> goal)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    RCLCPP_INFO(this->get_logger(), "land_handle_goal");
    if ((!is_vtol_ && !(aircraft_fsm_state_ == PX4InterfaceState::MC_HOVER || aircraft_fsm_state_ == PX4InterfaceState::MC_ORBIT)) || (is_vtol_ && aircraft_fsm_state_ != PX4InterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, PX4Interface is not in hover/orbit/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) {
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse PX4Interface::land_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void PX4Interface::land_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Land>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "land_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Land::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Land::Feedback>();

    double landing_altitude = goal->landing_altitude;
    double vtol_transition_heading = goal->vtol_transition_heading;

    bool landing = true;
    rclcpp::Rate landing_loop_rate(ACTION_LOOP_RATE_HZ);
    while (landing) {
        landing_loop_rate.sleep();
        std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling landing";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Landing canceled");
            return;
        }

        if (is_vtol_ == false) {
            if (aircraft_fsm_state_ == PX4InterfaceState::MC_ORBIT) {
                do_set_mode(4, 3); // If in an Orbit mode, switch to Hold mode
                aircraft_fsm_state_ = PX4InterfaceState::MC_HOVER;
            }
            if (aircraft_fsm_state_ == PX4InterfaceState::MC_HOVER) {
                double distance, heading;
                geod.Inverse(lat_, lon_, home_lat_, home_lon_, distance, heading);
                do_reposition(home_lat_, home_lon_, landing_altitude, fmod(heading + 360.0, 360.0) / 180.0 * M_PI);
                aircraft_fsm_state_ = PX4InterfaceState::RTL;
                feedback->message = "Returning home in MC mode";
                goal_handle->publish_feedback(feedback);
            } else if (aircraft_fsm_state_ == PX4InterfaceState::RTL) {
                double distance_from_home_in_meters;
                geod.Inverse(lat_, lon_, home_lat_, home_lon_, distance_from_home_in_meters);
                if (distance_from_home_in_meters < LAND_INIT_DIST_THRESH) {
                    do_land();
                    aircraft_fsm_state_ = PX4InterfaceState::MC_LANDING;
                    landing = false;
                    feedback->message = "Final MC mode descent";
                    goal_handle->publish_feedback(feedback);
                }
            }
        } else if (is_vtol_ == true) {
            double loiter_alt_low = VTOL_LAND_LOITER_ALT_LOW;
            double pre_landing_loiter_radius = VTOL_LAND_LOITER_RADIUS;
            double pre_landing_loiter_distance = VTOL_LAND_LOITER_DIST;
            double angle_correction_deg = atan(pre_landing_loiter_radius/pre_landing_loiter_distance) * 180.0 / M_PI;
            if (aircraft_fsm_state_ == PX4InterfaceState::FW_CRUISE) {
                double loiter_alt_hi = VTOL_LAND_LOITER_ALT_HIGH;
                auto [des_lat, des_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0 - angle_correction_deg);
                do_orbit(des_lat, des_lon, loiter_alt_hi, pre_landing_loiter_radius, NAN);
                aircraft_fsm_state_ = PX4InterfaceState::FW_LANDING_LOITER;
                feedback->message = "Going to landing loiter";
                goal_handle->publish_feedback(feedback);
            } else if (aircraft_fsm_state_ == PX4InterfaceState::FW_LANDING_LOITER) {
                auto [des_lat, des_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0 - angle_correction_deg);
                double distance_from_loiter_in_meters;
                geod.Inverse(lat_, lon_, des_lat, des_lon, distance_from_loiter_in_meters);
                if (distance_from_loiter_in_meters < (pre_landing_loiter_radius + VTOL_LAND_LOITER_STARTED_DIST_THRESH) && distance_from_loiter_in_meters > (pre_landing_loiter_radius - VTOL_LAND_LOITER_STARTED_DIST_THRESH)) {
                    do_orbit(des_lat, des_lon, loiter_alt_low, pre_landing_loiter_radius, NAN);
                    aircraft_fsm_state_ = PX4InterfaceState::FW_LANDING_DESCENT;
                    feedback->message = "Starting the descent loiter";
                    goal_handle->publish_feedback(feedback);
                }
            } else if (aircraft_fsm_state_ == PX4InterfaceState::FW_LANDING_DESCENT) {
                auto [exit_lat, exit_lon] = lat_lon_from_polar(home_lat_, home_lon_, pre_landing_loiter_distance, vtol_transition_heading + 180.0);
                double distance_from_exit_in_meters;
                geod.Inverse(lat_, lon_, exit_lat, exit_lon, distance_from_exit_in_meters);
                if (distance_from_exit_in_meters < VTOL_LAND_LOITER_EXIT_DIST_THRESH && std::abs(alt_ - (home_alt_ + loiter_alt_low)) < VTOL_LAND_LOITER_EXIT_ALT_THRESH) { // Exit from the loiter tangentially if distance and altitude requirements are met
                    auto [des_lat, des_lon] = lat_lon_from_polar(home_lat_, home_lon_, VTOL_LAND_FAKE_REPOSITION_DISTANCE, vtol_transition_heading); // Fake reposition behind the home point to fly over the home point
                    do_reposition(des_lat, des_lon, landing_altitude, NAN); // NOTE: this is only to give the VTOL a waypoint on the other side of the landing area
                    aircraft_fsm_state_ = PX4InterfaceState::FW_LANDING_APPROACH;
                    feedback->message = "Exiting the landing loiter";
                    goal_handle->publish_feedback(feedback);
                }
            } else if (aircraft_fsm_state_ == PX4InterfaceState::FW_LANDING_APPROACH) {
                double distance_from_home_in_meters;
                geod.Inverse(lat_, lon_, home_lat_, home_lon_, distance_from_home_in_meters);
                double landing_transition_distance = VTOL_LAND_TRANSITION_START_DISTANCE;
                if (distance_from_home_in_meters < landing_transition_distance) {
                    do_vtol_transition(3.0); // 3 is MAV_VTOL_STATE_MC
                    aircraft_fsm_state_ = PX4InterfaceState::VTOL_LANDING_TRANSITION;
                    feedback->message = "Transitioning to MC mode";
                    goal_handle->publish_feedback(feedback);
                }
            } else if (aircraft_fsm_state_ == PX4InterfaceState::VTOL_LANDING_TRANSITION && !in_transition_mode_ && vehicle_type_ == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING) {
                do_reposition(home_lat_, home_lon_, landing_altitude, NAN); // NOTE: the VTOL is in quad mode
                aircraft_fsm_state_ = PX4InterfaceState::RTL;
                feedback->message = "Repositioning in MC mode";
                goal_handle->publish_feedback(feedback);
            } else if (aircraft_fsm_state_ == PX4InterfaceState::RTL) {
                double distance_from_home_in_meters;
                geod.Inverse(lat_, lon_, home_lat_, home_lon_, distance_from_home_in_meters);
                if (distance_from_home_in_meters < LAND_INIT_DIST_THRESH) {
                    do_land();
                    aircraft_fsm_state_ = PX4InterfaceState::MC_LANDING;
                    landing = false;
                    feedback->message = "Final MC mode descent";
                    goal_handle->publish_feedback(feedback);
                }
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse PX4Interface::offboard_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Offboard::Goal> goal)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    RCLCPP_INFO(this->get_logger(), "offboard_handle_goal");
    if ((!is_vtol_ && aircraft_fsm_state_ != PX4InterfaceState::MC_HOVER) || (is_vtol_ && aircraft_fsm_state_ != PX4InterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Offboard rejected, PX4Interface is not in hover/cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) {
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse PX4Interface::offboard_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void PX4Interface::offboard_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Offboard>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "offboard_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Offboard::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Offboard::Feedback>();

    int offboard_setpoint_type = goal->offboard_setpoint_type;
    double max_duration_sec = goal->max_duration_sec;

    offboard_flag_count_ = 0;
    bool offboarding = true;
    uint64_t time_of_offboard_start_us_ = -1;
    rclcpp::Rate offboard_loop_rate(ACTION_LOOP_RATE_HZ);
    while (offboarding) {
        offboard_loop_rate.sleep();
        std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling offboard";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Offboard canceled");
            return;
        }

        uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
        if (time_of_offboard_start_us_ == -1) {
            if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::ATTITUDE) {
                aircraft_fsm_state_ = PX4InterfaceState::OFFBOARD_ATTITUDE;
                feedback->message = "Offboarding with ATTITUDE setpoints";
            }
            else if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::RATES) {
                aircraft_fsm_state_ = PX4InterfaceState::OFFBOARD_RATES;
                feedback->message = "Offboarding with RATES setpoints";
            }
            else if (offboard_setpoint_type == autopilot_interface_msgs::action::Offboard::Goal::TRAJECTORY) {
                aircraft_fsm_state_ = PX4InterfaceState::OFFBOARD_TRAJECTORY;
                feedback->message = "Offboarding with TRAJECTORY setpoints";
            }
            else {
                result->success = false;
                goal_handle->canceled(result);
                RCLCPP_ERROR(this->get_logger(), "Offboard type is not supported by PX4Interface (only ATTITUDE, RATES and TRAJECTORY)");
                return;
            }
            goal_handle->publish_feedback(feedback);
            time_of_offboard_start_us_ = current_time_us;
            feedback->message = "Starting offboard control at t=" + std::to_string(time_of_offboard_start_us_) + " us";
            goal_handle->publish_feedback(feedback);
        }
        if (current_time_us >= (time_of_offboard_start_us_ + max_duration_sec * 1000000)) {
            time_of_offboard_start_us_ = -1;
            offboarding = false;
            do_set_mode(4, 3); // Auto/Loiter (PX4_CUSTOM_MAIN_MODE 4/PX4_CUSTOM_SUB_MODE_AUTO 3)
            aircraft_fsm_state_ = is_vtol_ ? PX4InterfaceState::FW_CRUISE : PX4InterfaceState::MC_HOVER;
            feedback->message = "Exiting offboard control at t=" + std::to_string(current_time_us) + "us, returning to loiter/hover (Hold) state";
            goal_handle->publish_feedback(feedback);
        } else if ((current_time_us >= (time_of_offboard_start_us_ + 1 * 1000000)) && (current_time_us < (time_of_offboard_start_us_ + 2 * 1000000))) {
            // Send change mode for 1 sec, 1sec after the beginning of the reference stream
            do_set_mode(6, 0); // Offboard (PX4_CUSTOM_MAIN_MODE 6 no sub mode)
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse PX4Interface::orbit_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Orbit::Goal> goal)
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    RCLCPP_INFO(this->get_logger(), "orbit_handle_goal");
    if ((!is_vtol_ && (aircraft_fsm_state_ != PX4InterfaceState::MC_HOVER && aircraft_fsm_state_ != PX4InterfaceState::MC_ORBIT)) || (is_vtol_ && aircraft_fsm_state_ != PX4InterfaceState::FW_CRUISE)) {
        RCLCPP_ERROR(this->get_logger(), "Orbit rejected, PX4Interface is not in hover/orbit or cruise state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) {
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse PX4Interface::orbit_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void PX4Interface::orbit_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Orbit>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "orbit_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Orbit::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Orbit::Feedback>();

    double desired_east = goal->east;
    double desired_north = goal->north;
    double desired_alt = goal->altitude;
    double desired_r = goal->radius;

    bool orbiting = true;
    rclcpp::Rate orbit_loop_rate(ACTION_LOOP_RATE_HZ);
    while (orbiting) {
        orbit_loop_rate.sleep();
        std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling orbit";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Orbit canceled");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "New requested orbit East-North %.2f %.2f Alt. %.2f Radius %.2f", desired_east, desired_north, desired_alt, desired_r);
        auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, desired_east, desired_north);
        if (!is_vtol_) {
            do_orbit(des_lat, des_lon, desired_alt, desired_r, MC_ORBIT_SPEED_MS);
            RCLCPP_WARN(this->get_logger(), "For quads, the orbit speed is determined by constant MC_ORBIT_SPEED_MS");
            aircraft_fsm_state_ = PX4InterfaceState::MC_ORBIT; // For quads, this is a flight mode change, keep track of it
        } else if (is_vtol_) {
            do_orbit(des_lat, des_lon, desired_alt, desired_r, NAN);
        }
        goal_handle->publish_feedback(feedback);
        feedback->message = "Orbit sent";
        goal_handle->publish_feedback(feedback);

        orbiting = false;
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}
//
rclcpp_action::GoalResponse PX4Interface::takeoff_handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const autopilot_interface_msgs::action::Takeoff::Goal> goal)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_goal");
    if (aircraft_fsm_state_ != PX4InterfaceState::STARTED) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, PX4Interface is not in STARTED state");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (pre_flight_checks_pass_ != true) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, pre_flight_checks_pass_ is false");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (active_srv_or_act_flag_.exchange(true)) {
        RCLCPP_ERROR(this->get_logger(), "Another service/action is active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    home_lat_ = lat_;
    home_lon_ = lon_;
    home_alt_ = alt_;
    RCLCPP_WARN(this->get_logger(), "Saved home_lat_: %.5f, home_lon_ %.5f, home_alt_ %.2f", home_lat_, home_lon_, home_alt_);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
rclcpp_action::CancelResponse PX4Interface::takeoff_handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_cancel");
    return rclcpp_action::CancelResponse::ACCEPT;
}
void PX4Interface::takeoff_handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<autopilot_interface_msgs::action::Takeoff>> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "takeoff_handle_accepted");
    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<autopilot_interface_msgs::action::Takeoff::Result>();
    auto feedback = std::make_shared<autopilot_interface_msgs::action::Takeoff::Feedback>();

    double takeoff_altitude = goal->takeoff_altitude;
    double vtol_transition_heading = goal->vtol_transition_heading;
    double vtol_loiter_nord = goal->vtol_loiter_nord;
    double vtol_loiter_east = goal->vtol_loiter_east;
    double vtol_loiter_alt = goal->vtol_loiter_alt;

    bool taking_off = true;
    uint64_t time_of_vtol_transition_us_ = -1;
    rclcpp::Rate takeoff_loop_rate(ACTION_LOOP_RATE_HZ);
    while (taking_off) {
        takeoff_loop_rate.sleep();
        std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Reading data written by subs but also writing the FSM state

        if (goal_handle->is_canceling()) { // Check if there is a cancel request
            abort_action(); // Sets active_srv_or_act_flag_ to false, aircraft_fsm_state_ to MC_HOVER or FW_CRUISE
            feedback->message = "Canceling takeoff";
            goal_handle->publish_feedback(feedback);
            result->success = false;
            goal_handle->canceled(result);
            RCLCPP_WARN(this->get_logger(), "Takeoff canceled");
            return;
        }

        if (is_vtol_ == false) {
            if (aircraft_fsm_state_ == PX4InterfaceState::STARTED) {
                do_takeoff(takeoff_altitude, NAN); // TODO: currently, there's no heading takeoff for multirotors
                aircraft_fsm_state_ = PX4InterfaceState::MC_TAKEOFF;
                feedback->message = "Taking off in MC mode";
                goal_handle->publish_feedback(feedback);
            } else if (aircraft_fsm_state_ == PX4InterfaceState::MC_TAKEOFF) {
                if ((alt_ - home_alt_) > MC_TAKEOFF_COMPLETED_RATIO * takeoff_altitude) {
                    aircraft_fsm_state_ = PX4InterfaceState::MC_HOVER;
                    taking_off = false;
                    feedback->message = "Takeoff completed, hovering";
                    goal_handle->publish_feedback(feedback);
                }
            }
        } else if (is_vtol_ == true) {
            uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
            if (aircraft_fsm_state_ == PX4InterfaceState::STARTED) {
                do_takeoff(takeoff_altitude, vtol_transition_heading);
                aircraft_fsm_state_ = PX4InterfaceState::MC_TAKEOFF;
                feedback->message = "Taking off in MC mode";
                goal_handle->publish_feedback(feedback);
            } else if (aircraft_fsm_state_ == PX4InterfaceState::MC_TAKEOFF) {
                if (vehicle_type_ == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING) {
                    aircraft_fsm_state_ = PX4InterfaceState::VTOL_TAKEOFF_TRANSITION;
                    time_of_vtol_transition_us_ = current_time_us;
                    feedback->message = "Transitioned to FW";
                    goal_handle->publish_feedback(feedback);
                }
            } else if (aircraft_fsm_state_ == PX4InterfaceState::VTOL_TAKEOFF_TRANSITION &&
                (current_time_us > (time_of_vtol_transition_us_ + VTOL_TAKEOFF_TRANSITION_WAIT_SEC * 1000000))) {
                auto [des_lat, des_lon] = lat_lon_from_cartesian(home_lat_, home_lon_, vtol_loiter_east, vtol_loiter_nord);
                do_orbit(des_lat, des_lon, vtol_loiter_alt, VTOL_TAKEOFF_LOITER_RADIUS, NAN);
                aircraft_fsm_state_ = PX4InterfaceState::FW_CRUISE;
                taking_off = false;
                feedback->message = "Takeoff loiter sent";
                goal_handle->publish_feedback(feedback);
            }
        }
    }
    result->success = true;
    goal_handle->succeed(result);
    active_srv_or_act_flag_.store(false);
    return;
}

// vehicle_commands
void PX4Interface::do_takeoff(double alt, double yaw) {
    // Send arm command 3 times
    for (int i = 0; i < 3; ++i) {
        send_vehicle_command(
            400,  // MAV_CMD_COMPONENT_ARM_DISARM
            1.0,  // arm, 0.0, disarm
            0.0, // arm-disarm unless prevented by safety checks with 0.0, 21196.0: forced (e.g. allow arming to override preflight checks and disarming in flight)
                 // note that non-zero or above 50% RC throttle prevents arming in Stabilized and Position mode, respectively
            0.0, 0.0, 0.0, 0.0, 0.0, // Unused parameters
            i  // Confirmation, up to 255
        );
    }
    if (is_vtol_ == false) {
        send_vehicle_command(
            22,  // VEHICLE_CMD_NAV_TAKEOFF
            0.0,  // Unused
            0.0,  // Takeoff mode (specified) works with custom implementation of navigator_main.cpp and vtol_takeoff.cpp in PX4
            0.0,  // Unused
            yaw,  // TODO: implement heading for multirotor takeoff
            home_lat_,  // Latitude
            home_lon_,  // Longitude
            home_alt_ + alt,  // Altitude
            0  // Confirmation
        );
    } else if (is_vtol_ == true) {
        send_vehicle_command(
            84,  // VEHICLE_CMD_NAV_VTOL_TAKEOFF
            0.0,  // Unused
            3.0,  // Takeoff mode (specified) works with custom implementation of navigator_main.cpp and vtol_takeoff.cpp in PX4
            0.0,  // Unused
            yaw,  // Heading works with custom implementation of navigator_main.cpp and vtol_takeoff.cpp in PX4
            home_lat_,  // Latitude
            home_lon_,  // Longitude
            home_alt_ + alt,  // Altitude
            0  // Confirmation
        );
    }
}
void PX4Interface::do_change_altitude(double alt) // UNUSED
{
    send_vehicle_command(
        186,  // VEHICLE_CMD_DO_CHANGE_ALTITUDE
        home_alt_ + alt,  // New altitude
        1.0,  // Global frame
        0.0, 0.0, 0.0, 0.0, 0.0,  // Unused parameters
        0  // Confirmation
    );
}
void PX4Interface::do_change_speed(double speed)
{
    send_vehicle_command(
        178,  // VEHICLE_CMD_DO_CHANGE_SPEED
        0.0,  // Speed type: 0: airspeed, 1: ground speed, 2: climb speed, 3: descend speed
        speed,  // Speed setpoint
        0.0, 0.0, 0.0, 0.0, 0.0,  // Unused parameters
        0  // Confirmation
    );
}
void PX4Interface::do_orbit(double lat, double lon, double alt, double r, double speed)
{
    send_vehicle_command(
        34,  // VEHICLE_CMD_DO_ORBIT
        r,   // Orbit radius (positive: CW, negative: CCW)
        speed,  // Orbit speed (Tangential Velocity. NaN: Use vehicle default velocity, or current velocity if already orbiting. m/s)
        0,  // Yaw behavior: 0: towards the center of the orbit (for quads, not VTOLs), 1: initial, 2: uncontrolled, 3: tangent, 4: rc, 5: unchanged
        NAN,  // Number of loops in MAVLINK but unused in PX4
        lat,  // Target latitude
        lon,  // Target longitude
        home_alt_ + alt,  // Altitude
        0  // Confirmation
    );
}
void PX4Interface::do_reposition(double lat, double lon, double alt, double heading)
{
    send_vehicle_command(
        192,  // MAV_CMD_DO_REPOSITION
        0.0, 0.0, 0.0, // Unused parameters
        heading, // Heading in radians, only used for quads
        lat,  // Latitude
        lon,  // Longitude
        home_alt_ + alt,  // Altitude
        0  // Confirmation
    );
}
void PX4Interface::do_vtol_transition(int trans_type)
{
    send_vehicle_command(
        3000,  // VEHICLE_CMD_DO_VTOL_TRANSITION
        trans_type,  // Transition type (3: MC, 4: FW)
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Unused parameters
        0  // Confirmation
    );
}
void PX4Interface::do_rtl() // UNUSED
{
    send_vehicle_command(
        20,  // VEHICLE_CMD_NAV_RETURN_TO_LAUNCH
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  // Unused parameters
        0  // Confirmation
    );
}
void PX4Interface::do_land()
{
    send_vehicle_command(
        21,  // VEHICLE_CMD_NAV_LAND
        0.0, 0.0, 0.0, 0.0,  // Unused parameters
        home_lat_,  // Latitude
        home_lon_,  // Longitude
        home_alt_,  // Ground altitude
        0  // Confirmation
    );
}
void PX4Interface::do_set_mode(int mode, int submode)
{
    send_vehicle_command(
        176,  // MAV_CMD_DO_SET_MODE
        1.0, // Flags (custom mode)
        mode, // Custom Mode https://github.com/PX4/PX4-Autopilot/blob/v1.16.2/src/modules/commander/px4_custom_mode.h
        submode, // Custom Sub Mode
        0.0, 0.0, 0.0, 0.0,  // Unused parameters
        0  // Confirmation
    );
}
void PX4Interface::send_vehicle_command(int command, double param1, double param2, double param3,
                        double param4, double param5, double param6, double param7, int conf)
{
    VehicleCommand vehicle_command;
    vehicle_command.command = command;

    uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    vehicle_command.timestamp = current_time_us;

    vehicle_command.param1 = param1;
    vehicle_command.param2 = param2;
    vehicle_command.param3 = param3;
    vehicle_command.param4 = param4;
    vehicle_command.param5 = param5;  // Latitude
    vehicle_command.param6 = param6;  // Longitude
    vehicle_command.param7 = param7;  // Altitude

    vehicle_command.target_system = target_system_id_; // In PX4 MAV_SYS_ID param for real systems, based on -i N for SITL, fetched once by status_callback
    vehicle_command.target_component = 1;
    vehicle_command.source_system = 255; // Same as QGC's default, can be different
    vehicle_command.source_component = 0;
    vehicle_command.confirmation = conf;
    vehicle_command.from_external = true;

    command_pub_->publish(vehicle_command);

    RCLCPP_INFO(this->get_logger(), "Sent VehicleCommand: %d", command);
}
void PX4Interface::abort_action()
{
    if (vehicle_type_ == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_ROTARY_WING) {
        do_reposition(lat_, lon_, ABORT_REPOSITION_ALT, NAN); //For quads, hover in place
        aircraft_fsm_state_ = PX4InterfaceState::MC_HOVER;
    } else if (vehicle_type_ == px4_msgs::msg::VehicleStatus::VEHICLE_TYPE_FIXED_WING) {
        do_reposition(home_lat_, home_lon_, ABORT_REPOSITION_ALT, NAN); // For VTOLs, reposition above home, the loiter radius is parameter NAV_LOITER_RAD
        aircraft_fsm_state_ = PX4InterfaceState::FW_CRUISE;
    } // TODO: if a VTOL errenously ended in the MC_HOVER state, it would not be recoverable by PX4Interface as an explicit transition is not exposed (land via QGC)
    active_srv_or_act_flag_.store(false);
}

std::pair<double, double> PX4Interface::lat_lon_from_cartesian(double ref_lat, double ref_lon, double x_offset, double y_offset)
{
    double temp_lat, temp_lon;
    double bearing_ns = (y_offset >= 0) ? 0 : 180; // North-South offset (bearing 0 for north, 180 for south)
    geod.Direct(ref_lat, ref_lon, bearing_ns, std::abs(y_offset), temp_lat, temp_lon);
    double return_lat, return_lon;
    double bearing_ew = (x_offset >= 0) ? 90 : 270; // East-West offset (bearing 90 for east, 270 for west)
    geod.Direct(temp_lat, temp_lon, bearing_ew, std::abs(x_offset), return_lat, return_lon);
    return {return_lat, return_lon};
}

std::pair<double, double> PX4Interface::lat_lon_from_polar(double ref_lat, double ref_lon, double dist, double bear)
{
    double return_lat, return_lon;
    geod.Direct(ref_lat, ref_lon, bear, dist, return_lat, return_lon);
    return {return_lat, return_lon};
}

std::string PX4Interface::fsm_state_to_string(PX4InterfaceState state)
{
    switch (state) {
        case PX4InterfaceState::STARTED: return "STARTED";
        case PX4InterfaceState::MC_TAKEOFF: return "MC_TAKEOFF";
        case PX4InterfaceState::MC_HOVER: return "MC_HOVER";
        case PX4InterfaceState::MC_ORBIT: return "MC_ORBIT";
        case PX4InterfaceState::VTOL_TAKEOFF_TRANSITION: return "VTOL_TAKEOFF_TRANSITION";
        case PX4InterfaceState::FW_CRUISE: return "FW_CRUISE";
        case PX4InterfaceState::FW_LANDING_LOITER: return "FW_LANDING_LOITER";
        case PX4InterfaceState::FW_LANDING_DESCENT: return "FW_LANDING_DESCENT";
        case PX4InterfaceState::FW_LANDING_APPROACH: return "FW_LANDING_APPROACH";
        case PX4InterfaceState::VTOL_LANDING_TRANSITION: return "VTOL_LANDING_TRANSITION";
        case PX4InterfaceState::RTL: return "RTL";
        case PX4InterfaceState::MC_LANDING: return "MC_LANDING";
        case PX4InterfaceState::OFFBOARD_ATTITUDE: return "OFFBOARD_ATTITUDE";
        case PX4InterfaceState::OFFBOARD_RATES: return "OFFBOARD_RATES";
        case PX4InterfaceState::OFFBOARD_TRAJECTORY: return "OFFBOARD_TRAJECTORY";
        default: return "UNKNOWN";
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<PX4Interface>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
