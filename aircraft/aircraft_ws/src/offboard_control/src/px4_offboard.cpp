#include "px4_offboard.hpp"

PX4Offboard::PX4Offboard() : Node("px4_offboard"), 
    own_id_(-1), offboard_active_(false), active_controller_name_(""), active_controller_func_(nullptr),
    offboard_loop_frequency(50), offboard_loop_count_(0), last_offboard_loop_count_(0),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    xy_valid_(false), z_valid_(false), v_xy_valid_(false), v_z_valid_(false), xy_global_(false), z_global_(false),
    x_(NAN), y_(NAN), z_(NAN), heading_(NAN), vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    pose_frame_(-1), velocity_frame_(-1), true_airspeed_m_s_(NAN), vehicle_type_(-1), is_vtol_(false), is_vtol_tailsitter_(false),
    ground_tracks_(nullptr), yolo_detections_(nullptr),
    traj_ref_east_(NAN), traj_ref_north_(NAN), traj_ref_up_(NAN),
    target_vn_(NAN), target_ve_(NAN), target_vd_(NAN)
{
    RCLCPP_INFO(this->get_logger(), "PX4 offboard referencing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    // Grab own ID from the namespace
    std::string ns = this->get_namespace();
    size_t pos = ns.find("Drone");
    if (pos != std::string::npos) {
        try { own_id_ = std::stoi(ns.substr(pos + 5)); }
        catch (const std::exception&) {}
    }
    if (own_id_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: Could not parse drone ID from namespace '%s'.", ns.c_str());
    }
    // Check and log whether simulation time is enabled or not
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
    } else {
        RCLCPP_WARN(this->get_logger(), "Simulation time is disabled.");
    }
    last_offboard_rate_check_time_ = this->get_clock()->now(); // Monitor the rate of offboard control loop
    // Initialize the arrays
    position_.fill(NAN);
    q_.fill(NAN);
    velocity_.fill(NAN);
    angular_velocity_.fill(NAN);
    kiss_position_.fill(NAN);
    kiss_q_.fill(NAN);

    // PX4 publishers
    rclcpp::QoS qos_profile_pub(10);  // Depth of 10
    qos_profile_pub.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Or rclcpp::DurabilityPolicy::Volatile
    offboard_mode_pub_ = this->create_publisher<OffboardControlMode>("fmu/in/offboard_control_mode", qos_profile_pub);
    attitude_ref_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("fmu/in/vehicle_attitude_setpoint_v1", qos_profile_pub); // MESSAGE_VERSION = 1 -> _v1 since 1.17
    rates_ref_pub_ = this->create_publisher<VehicleRatesSetpoint>("fmu/in/vehicle_rates_setpoint", qos_profile_pub);
    trajectory_ref_pub_ = this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_printout_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Strictly sequential callbacks
    callback_group_offboard_control_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Strictly sequential callbacks
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel

    // Timers
    px4_interface_printout_timer_ = this->create_wall_timer( // Follow wall clock for printouts
        3s, // Timer period of 3 seconds
        std::bind(&PX4Offboard::px4_interface_printout_callback, this),
        callback_group_printout_
    );
    offboard_control_loop_timer_ = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(1000000000 / offboard_loop_frequency),
        std::bind(&PX4Offboard::offboard_loop_callback, this),
        callback_group_offboard_control_
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
        std::bind(&PX4Offboard::global_position_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_local_position_sub_= this->create_subscription<VehicleLocalPosition>(
        "fmu/out/vehicle_local_position_v1", qos_profile_sub, // 100Hz, MESSAGE_VERSION = 1 -> _v1 since 1.17
        std::bind(&PX4Offboard::local_position_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_odometry_sub_= this->create_subscription<VehicleOdometry>(
        "fmu/out/vehicle_odometry", qos_profile_sub, // 100Hz
        std::bind(&PX4Offboard::odometry_callback, this, std::placeholders::_1), subscriber_options);
    airspeed_validated_sub_ = this->create_subscription<AirspeedValidated>(
        "fmu/out/airspeed_validated_v1", qos_profile_sub, // 10Hz, MESSAGE_VERSION = 1 -> _v1 since 1.17
        std::bind(&PX4Offboard::airspeed_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_status_sub_ = this->create_subscription<VehicleStatus>(
        "fmu/out/vehicle_status_v1", qos_profile_sub, // 2Hz, MESSAGE_VERSION = 1 -> _v1 since 1.16
        std::bind(&PX4Offboard::status_callback, this, std::placeholders::_1), subscriber_options);

    // Offboard flag subscriber
    offboard_flag_sub_ = this->create_subscription<autopilot_interface_msgs::msg::OffboardFlag>(
        "/offboard_flag", qos_profile_sub, // 10Hz
        std::bind(&PX4Offboard::offboard_flag_callback, this, std::placeholders::_1), subscriber_options);

    // Perception subscribers
    ground_tracks_sub_ = this->create_subscription<ground_system_msgs::msg::SwarmObs>(
        "/tracks", qos_profile_sub, // 1Hz
        std::bind(&PX4Offboard::ground_tracks_callback, this, std::placeholders::_1), subscriber_options);
    yolo_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections", qos_profile_sub, // 15Hz
        std::bind(&PX4Offboard::yolo_detections_callback, this, std::placeholders::_1), subscriber_options);
    kiss_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/kiss/odometry", qos_profile_sub, // 10Hz
        std::bind(&PX4Offboard::kiss_odometry_callback, this, std::placeholders::_1), subscriber_options);

    // Controllers map
    // Examples
    controller_map_["att-test"] = std::bind(&PX4Offboard::att_ref_test, this, std::placeholders::_1);
    controller_map_["ctbr-test"] = std::bind(&PX4Offboard::ctbr_ref_test, this, std::placeholders::_1);
    controller_map_["traj-test"] = std::bind(&PX4Offboard::traj_ref_test, this, std::placeholders::_1);
    // Custom controllers
    controller_map_["traj-prv"] = std::bind(&PX4Offboard::traj_ref_predictive_rendezvous, this, std::placeholders::_1);
}

// Callbacks for subscribers (reentrant group)
void PX4Offboard::global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->lat;
    lon_ = msg->lon;
    alt_ = msg->alt; // AMSL
    alt_ellipsoid_ = msg->alt_ellipsoid; // TODO: double-check
}
void PX4Offboard::local_position_callback(const VehicleLocalPosition::SharedPtr msg)
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
void PX4Offboard::odometry_callback(const VehicleOdometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    pose_frame_ = msg->pose_frame; // 1:  NED earth-fixed frame, 2: FRD world-fixed frame, arbitrary heading
    velocity_frame_ = msg->velocity_frame; // 1:  NED earth-fixed frame, 2: FRD world-fixed frame, arbitrary heading, 3: FRD body-fixed frame
    position_ = msg->position;
    q_ = msg->q;
    velocity_ = msg->velocity;
    angular_velocity_ = msg->angular_velocity;
}
void PX4Offboard::airspeed_callback(const AirspeedValidated::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    true_airspeed_m_s_ = msg->true_airspeed_m_s;
}
void PX4Offboard::status_callback(const VehicleStatus::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    // arming_state_ = msg->arming_state; // DISARMED = 1, ARMED = 2
    vehicle_type_ = msg->vehicle_type; // ROTARY_WING = 1, FIXED_WING = 2 (ROVER = 3)
    is_vtol_ = msg->is_vtol; // bool
    is_vtol_tailsitter_ = msg->is_vtol_tailsitter; // bool
    // in_transition_mode_ = msg->in_transition_mode; // bool
    // in_transition_to_fw_ = msg->in_transition_to_fw; // bool
    // pre_flight_checks_pass_ = msg->pre_flight_checks_pass; // bool
}
void PX4Offboard::offboard_flag_callback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    offboard_active_ = msg->is_active;
    if (offboard_active_) {
        if (active_controller_name_ != msg->controller_name) { // Only perform the map lookup if the requested controller has changed
            active_controller_name_ = msg->controller_name;
            auto it = controller_map_.find(active_controller_name_);
            if (it != controller_map_.end()) {
                active_controller_func_ = it->second; // Cache the controller function
            } else {
                active_controller_func_ = nullptr; // Failsafe
            }
        }
    } else { // Clean up when offboard flag is inactive
        active_controller_name_ = "";
        active_controller_func_ = nullptr;
    }
}
void PX4Offboard::ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ground_tracks_ = msg; // Save the smart pointer to the latest message
    last_track_time_ = this->get_clock()->now();

    // Verify LLA position of own reference point (used in PX4 local position)
    double reference_lat = ref_lat_;
    double reference_lon = ref_lon_;
    double reference_alt = ref_alt_;
    if (std::isnan(reference_lat) || std::isnan(reference_lon) || std::isnan(reference_alt)) {
        RCLCPP_WARN_ONCE(get_logger(), "Waiting for reference position");
        return;
    }

    // Find our own track to see whom the GroundSystem assigned us to
    auto my_it = std::find_if(ground_tracks_->tracks.begin(), ground_tracks_->tracks.end(),
                              [this](const auto& track) { return track.id == this->own_id_; });
    if (my_it == ground_tracks_->tracks.end()) {
        RCLCPP_WARN_ONCE(get_logger(), "Own track (ID %d) not found in tracks", own_id_);
        return;
    }
    // Get assignment and find its track
    int assigned_target_id = my_it->label;
    auto target_it = std::find_if(ground_tracks_->tracks.begin(), ground_tracks_->tracks.end(),
                                  [assigned_target_id](const auto& track) { return track.id == assigned_target_id; });
    if (target_it == ground_tracks_->tracks.end()) {
        RCLCPP_WARN_ONCE(get_logger(), "Assigned target ID %d not found in tracks.", assigned_target_id);
        return;
    }
    const auto& target_track = *target_it; // Bind a reference without copying

    // Save target velocities
    target_vn_ = target_track.velocity_n_m_s;
    target_ve_ = target_track.velocity_e_m_s;
    target_vd_ = target_track.velocity_d_m_s;

    // Predict LLA position of target
    constexpr double PREDICTION_TIME_SEC = 0.0; // TODO: enable prediction
    constexpr double ALT_SAFETY_MARGIN = 0.0; // TODO: add vertical separation to avoid collisions

    double target_ground_speed = std::hypot(target_track.velocity_n_m_s, target_track.velocity_e_m_s);
    double target_course_rad = std::atan2(target_track.velocity_e_m_s, target_track.velocity_n_m_s); // Azimuth from North
    double target_course_deg = target_course_rad * (180.0 / M_PI);
    double distance_traveled = target_ground_speed * PREDICTION_TIME_SEC;

    double future_lat = 0.0, future_lon = 0.0;
    geod.Direct(target_track.latitude_deg, target_track.longitude_deg, target_course_deg, distance_traveled,
                future_lat, future_lon);
    double future_alt = target_track.altitude_m - (target_track.velocity_d_m_s * PREDICTION_TIME_SEC) + ALT_SAFETY_MARGIN;

    // Compute GeographicLib ENU position of label48 w.r.t. PX4 vehicle (using NED)
    const GeographicLib::LocalCartesian proj(reference_lat, reference_lon, reference_alt);
    proj.Forward(future_lat, future_lon, future_alt, traj_ref_east_, traj_ref_north_, traj_ref_up_);
}

void PX4Offboard::yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    if (msg->header.frame_id == "camera_frame_0") { // Only process the primary camera
        yolo_detections_ = msg; // Save the smart pointer to the latest message
    }
}

void PX4Offboard::kiss_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    kiss_position_[0] = msg->pose.pose.position.x; // ENU
    kiss_position_[1] = msg->pose.pose.position.y;
    kiss_position_[2] = msg->pose.pose.position.z;
    kiss_q_[0] = msg->pose.pose.orientation.w;
    kiss_q_[1] = msg->pose.pose.orientation.x;
    kiss_q_[2] = msg->pose.pose.orientation.y;
    kiss_q_[3] = msg->pose.pose.orientation.z;
}

// Callbacks for timers (reentrant group)
void PX4Offboard::px4_interface_printout_callback()
{
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    auto now = this->get_clock()->now();
    double elapsed_sec = (now - last_offboard_rate_check_time_).seconds();
    double actual_rate = NAN;
    if (elapsed_sec > 0) {
        actual_rate = (offboard_loop_count_ - last_offboard_loop_count_) / elapsed_sec;
    }    
    last_offboard_loop_count_.store(offboard_loop_count_.load());
    last_offboard_rate_check_time_ = now;
    RCLCPP_INFO(get_logger(),
                "\n  Current node time: %.2f seconds\n"
                "  KISS pos: %.2f %.2f %.2f\n"
                "  Offboard active:\t%s\n"
                "  Controller:\t%s\n"
                "  Offboard loop rate:\t%.2f Hz",
                this->get_clock()->now().seconds(),
                kiss_position_[0], kiss_position_[1], kiss_position_[2],
                offboard_active_ ? "true" : "false",
                offboard_active_ ? active_controller_name_.c_str() : "None",
                actual_rate
            );
    std::stringstream ss;
    auto local_tracks = ground_tracks_;
    if (local_tracks) {
        if (local_tracks->tracks.empty()) {
            ss << "\nGround Tracks: [No tracks in message]\n";
        } else {
            ss << "\nGround Tracks:\n";
            for (const auto& track : local_tracks->tracks) {
                ss << "  Id " << static_cast<int>(track.id)
                << " lat: " << std::fixed << std::setprecision(5) << track.latitude_deg
                << " lon: " << std::fixed << std::setprecision(5) << track.longitude_deg
                << " alt (msl): " << std::fixed << std::setprecision(2) << track.altitude_m << "\n";
            }
        }
    } else {
        ss << "\nGround Tracks: [No message received yet]\n";
    }
    auto local_detections = yolo_detections_;
    if (local_detections) {
        if (local_detections->detections.empty()) {
            ss << "YOLO Detections: [No detections in message]\n";
        } else {
            ss << "YOLO Detections:\n";
            for (const auto& detection : local_detections->detections) {
                for (const auto& result : detection.results) {
                    double azimuth = result.pose.pose.position.x; // Computed in yolo_node.py
                    double elevation = result.pose.pose.position.y;
                    ss << "  Label: " << result.hypothesis.class_id
                    << " - conf: " << std::fixed << std::setprecision(2) << result.hypothesis.score
                    << " - az: " << std::setprecision(1) << azimuth << "°"
                    << " - el: " << elevation << "°\n";
                }
            }
        }
    } else {
        ss << "YOLO Detections: [No message received yet]\n";
    }
    RCLCPP_INFO(get_logger(), "%s\n", ss.str().c_str());
}
void PX4Offboard::offboard_loop_callback()
{
    offboard_loop_count_++; // Counter to monitor the rate of the offboard loop (no lock, atomic variable)
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if (!offboard_active_) {
        return; // Do not publish anything else if not in OFFBOARD state
    }
    if (active_controller_func_ != nullptr) {
        OffboardControlMode offboard_mode;
        offboard_mode.timestamp = this->get_clock()->now().nanoseconds() / 1000; // Convert to microseconds
        active_controller_func_(offboard_mode); // Execute the cached controller, passing the mode by reference (to modify it and copy its timestamp)
        if (offboard_loop_count_ % std::max(1, (offboard_loop_frequency / 10)) == 0) {
            offboard_mode_pub_->publish(offboard_mode); // The OffboardControlMode should run at at least 2Hz (~10 in this implementation)
        }
    } else {
        RCLCPP_WARN(get_logger(), "Unknown controller requested: '%s', no reference will be published", active_controller_name_.c_str());
    }
}

// Controllers (reference generators)
void PX4Offboard::att_ref_test(OffboardControlMode& mode)
{
    mode.attitude = true;
    VehicleAttitudeSetpoint attitude_ref; // https://github.com/PX4/px4_msgs/blob/release/1.17/msg/VehicleAttitudeSetpoint.msg
    attitude_ref.timestamp = mode.timestamp;
    if (vehicle_type_ == 1) { // ROTARY_WING
        double pitch_rad = -5.0 * M_PI / 180.0; // Negative pitch to move forward (any duration, drops some altitude)
        // Get current yaw and desired pitch
        double cy = cos(heading_ / 2.0);
        double sy = sin(heading_ / 2.0);
        double cp = cos(pitch_rad / 2.0);
        double sp = sin(pitch_rad / 2.0);
        // Quaternion reference: Q_yaw * Q_pitch (the reference is in PX4 NED world frame)
        attitude_ref.q_d[0] = static_cast<float>(cy * cp);          // w
        attitude_ref.q_d[1] = static_cast<float>(-sy * sp);         // x
        attitude_ref.q_d[2] = static_cast<float>(cy * sp);          // y
        attitude_ref.q_d[3] = static_cast<float>(sy * cp);          // z
        attitude_ref.thrust_body = {0.0, 0.0, -0.72};
    } else if (vehicle_type_ == 2) { // FIXED_WING
        double pitch_rad = -30.0 * M_PI / 180.0; // Negative pitch to dive
        attitude_ref.q_d[0] = static_cast<float>(cos(pitch_rad / 2.0)); // w
        attitude_ref.q_d[1] = 0;                                        // x
        attitude_ref.q_d[2] = static_cast<float>(sin(pitch_rad / 2.0)); // y
        attitude_ref.q_d[3] = 0;                                        // z
        attitude_ref.thrust_body = {0.15, 0.0, 0.0};
    } else {
        RCLCPP_WARN(get_logger(), "Unknown vehicle_type_ %d", vehicle_type_);
        return;
    }
    attitude_ref_pub_->publish(attitude_ref);
}
void PX4Offboard::ctbr_ref_test(OffboardControlMode& mode)
{
    mode.body_rate = true;
    VehicleRatesSetpoint rates_ref; // https://github.com/PX4/px4_msgs/blob/release/1.17/msg/VehicleRatesSetpoint.msg
    rates_ref.timestamp = mode.timestamp;
    if (vehicle_type_ == 1) { // ROTARY_WING
        rates_ref.roll= 0.0;
        rates_ref.pitch = 0.0;
        rates_ref.yaw = 1.0; // Spin on itself (any duration)
        rates_ref.thrust_body = {0.0, 0.0, -0.72};
    } else if (vehicle_type_ == 2) { // FIXED_WING
        rates_ref.roll= 4.0; // Roll (2sec maneuver 1 roll, 3sec double roll)
        rates_ref.pitch = 0.0;
        rates_ref.thrust_body = {0.39, 0.0, 0.0};
    } else {
        RCLCPP_WARN(get_logger(), "Unknown vehicle_type_ %d", vehicle_type_);
        return;
    }
    rates_ref_pub_->publish(rates_ref);
}
void PX4Offboard::traj_ref_test(OffboardControlMode& mode)
{
    TrajectorySetpoint trajectory_ref; // https://github.com/PX4/px4_msgs/blob/release/1.17/msg/TrajectorySetpoint.msg
    trajectory_ref.timestamp = mode.timestamp;
    if (vehicle_type_ == 1) { // ROTARY_WING
        mode.position = true;
        // mode.acceleration = true; // Enable acceleration feedforward
        trajectory_ref.position = {0.0, 0.0, -50.0}; // 50m above the home point
        trajectory_ref.velocity = {NAN, NAN, NAN}; // Unused
        trajectory_ref.acceleration = {NAN, NAN, NAN}; // Unused
        trajectory_ref.jerk = {NAN, NAN, NAN}; // Unused
        trajectory_ref.yaw = -3.14; // [-PI:PI]
    } else if (vehicle_type_ == 2) { // FIXED_WING
        mode.velocity = true;
        trajectory_ref.position = {NAN, NAN, NAN}; // Unused
        trajectory_ref.velocity = {20.0, 0.0, 0.0};
        trajectory_ref.acceleration = {NAN, NAN, NAN}; // Unused
        trajectory_ref.jerk = {NAN, NAN, NAN}; // Unused
    } else {
        RCLCPP_WARN(get_logger(), "Unknown vehicle_type_ %d", vehicle_type_);
        return;
    }
    trajectory_ref_pub_->publish(trajectory_ref);
}
void PX4Offboard::traj_ref_predictive_rendezvous(OffboardControlMode& mode)
{
    if (vehicle_type_ != 1) { // Publish nothing if the vehicle is not ROTARY_WING
        RCLCPP_WARN(get_logger(), "This controller is only for multicopters");
        return;
    }
    mode.position = true;
    TrajectorySetpoint trajectory_ref; // https://github.com/PX4/px4_msgs/blob/release/1.17/msg/TrajectorySetpoint.msg
    trajectory_ref.timestamp = mode.timestamp;
    trajectory_ref.acceleration = {NAN, NAN, NAN}; // Unused
    trajectory_ref.jerk = {NAN, NAN, NAN}; // Unused
    if (!std::isnan(traj_ref_east_) && !std::isnan(traj_ref_north_) && !std::isnan(traj_ref_up_)) {
        double dt = std::clamp((this->get_clock()->now() - last_track_time_).seconds(), 0.0, 2.0);
        double current_north = traj_ref_north_ + (target_vn_ * dt);
        double current_east  = traj_ref_east_  + (target_ve_ * dt);
        double current_down  = -traj_ref_up_   + (target_vd_ * dt);
        trajectory_ref.position = {static_cast<float>(current_north), static_cast<float>(current_east), static_cast<float>(current_down)};
        double d_north = current_north - x_;
        double d_east  = current_east - y_;
        trajectory_ref.yaw = static_cast<float>(std::atan2(d_east, d_north)); // [-PI:PI]
        if (!std::isnan(target_vn_) && !std::isnan(target_ve_) && !std::isnan(target_vd_)) {
            mode.velocity = true; // Enable velocity feedforward
            trajectory_ref.velocity = {static_cast<float>(target_vn_), static_cast<float>(target_ve_), static_cast<float>(target_vd_)};
            double dist_sq = (d_north * d_north) + (d_east * d_east);
            if (dist_sq > 1.0) {
                double vrel_n = target_vn_ - vx_;
                double vrel_e = target_ve_ - vy_;
                trajectory_ref.yawspeed = static_cast<float>((d_north * vrel_e - d_east * vrel_n) / dist_sq);
            } else {
                trajectory_ref.yawspeed = 0.0;
            }
        } else {
            trajectory_ref.velocity = {NAN, NAN, NAN};
            trajectory_ref.yawspeed = NAN;
        }
    } else { // Missing track, stay still
        mode.position = false;
        trajectory_ref.position = {NAN, NAN, NAN}; ;
        trajectory_ref.yaw = NAN;
        mode.velocity = true;
        trajectory_ref.velocity = {0.0, 0.0, 0.0};
        trajectory_ref.yawspeed = 0.0;
    }
    trajectory_ref_pub_->publish(trajectory_ref);
}

int main(int argc, char *argv[])
{    
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<PX4Offboard>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
