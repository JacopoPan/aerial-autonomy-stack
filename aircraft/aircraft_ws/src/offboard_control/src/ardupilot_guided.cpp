#include "ardupilot_guided.hpp"

ArdupilotGuided::ArdupilotGuided() : Node("ardupilot_guided"),
    own_id_(-1), offboard_active_(false), active_controller_name_(""), active_controller_func_(nullptr),
    offboard_loop_frequency(10), offboard_loop_count_(0), last_offboard_loop_count_(0),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    x_(NAN), y_(NAN), z_(NAN),  vx_(NAN), vy_(NAN), vz_(NAN), ve_(NAN), vn_(NAN), vu_(NAN),
    ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    true_airspeed_m_s_(NAN), heading_(NAN),
    ground_tracks_(nullptr), yolo_detections_(nullptr),
    desired_bearing_rad_(NAN), desired_elevation_rad_(NAN), closing_distance_(NAN),
    target_vn_(NAN), target_ve_(NAN), target_vd_(NAN),
    lemniscate_phase_rad_(0.0)
{
    RCLCPP_INFO(this->get_logger(), "ArduPilot guided referencing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
    // Grab own ID from the namespace
    std::string ns = this->get_namespace();
    size_t pos = ns.find("Drone");
    if (pos != std::string::npos) {
        try { own_id_ = std::stoi(ns.substr(pos + 5)); }
        catch (const std::exception & e) {
            RCLCPP_WARN(this->get_logger(), "stoi failed: %s", e.what());
        }
    }
    if (own_id_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "CRITICAL: Could not parse drone ID from namespace '%s'.", ns.c_str());
    }
    // Check and log whether simulation time is enabled or not
    if (this->get_parameter("use_sim_time").as_bool()) {
        RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
    } else {
        RCLCPP_INFO(this->get_logger(), "Simulation time is disabled.");
    }
    last_offboard_rate_check_time_ = this->get_clock()->now(); // Monitor the rate of offboard control loop
    // Initialize the arrays
    position_.fill(NAN);
    q_.fill(NAN);
    velocity_.fill(NAN);
    angular_velocity_.fill(NAN);
    kiss_position_.fill(NAN);
    kiss_q_.fill(NAN);

    // MAVROS Publishers
    rclcpp::QoS qos_profile_pub = rclcpp::SensorDataQoS(); // Match MAVROS setpoint subscribers (BEST_EFFORT + VOLATILE)
    setpoint_accel_pub_= this->create_publisher<Vector3Stamped>("/mavros/setpoint_accel/accel", qos_profile_pub);
    setpoint_vel_pub_= this->create_publisher<TwistStamped>("/mavros/setpoint_velocity/cmd_vel", qos_profile_pub);
    setpoint_raw_att_pub_ = this->create_publisher<AttitudeTarget>("/mavros/setpoint_raw/attitude", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_printout_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Strictly sequential callbacks
    callback_group_offboard_control_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Strictly sequential callbacks
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel

    // Timers
    ardupilot_interface_printout_timer_ = this->create_wall_timer( // Follow wall clock for printouts
        3s, // Timer period of 3 seconds
        std::bind(&ArdupilotGuided::ardupilot_interface_printout_callback, this),
        callback_group_printout_
    );
    offboard_control_loop_timer_ = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(1000000000 / offboard_loop_frequency),
        std::bind(&ArdupilotGuided::offboard_loop_callback, this),
        callback_group_offboard_control_
    );

    // Subscribers configuration
    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.callback_group = callback_group_subscriber_;
    rclcpp::QoS qos_profile_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile_sub.keep_last(10);  // History: KEEP_LAST with depth 10
    qos_profile_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // MAVROS subscribers
    mavros_global_position_global_sub_= this->create_subscription<NavSatFix>(
        "/mavros/global_position/global", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::global_position_global_sub_callback, this, std::placeholders::_1), subscriber_options);
    mavros_local_position_odom_sub_= this->create_subscription<Odometry>(
        "/mavros/local_position/odom", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::local_position_odom_callback, this, std::placeholders::_1), subscriber_options);
    mavros_global_position_local_sub_ = this->create_subscription<Odometry>(
        "/mavros/global_position/local", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::global_position_local_callback, this, std::placeholders::_1), subscriber_options);
    mavros_local_position_vel_local_sub_ = this->create_subscription<TwistStamped>(
        "/mavros/local_position/velocity_local", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::local_position_vel_local_callback, this, std::placeholders::_1), subscriber_options);
    mavros_vfr_hud_sub_ = this->create_subscription<VfrHud>(
        "/mavros/vfr_hud", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::vfr_hud_callback, this, std::placeholders::_1), subscriber_options);

    // Offboard flag subscriber
    offboard_flag_sub_ = this->create_subscription<autopilot_interface_msgs::msg::OffboardFlag>(
        "/offboard_flag", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::offboard_flag_callback, this, std::placeholders::_1), subscriber_options);

    // Perception subscribers
    ground_tracks_sub_ = this->create_subscription<ground_system_msgs::msg::SwarmObs>(
        "/tracks", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::ground_tracks_callback, this, std::placeholders::_1), subscriber_options);
    yolo_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections", qos_profile_sub, // 15Hz
        std::bind(&ArdupilotGuided::yolo_detections_callback, this, std::placeholders::_1), subscriber_options);
    kiss_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/kiss/odometry", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::kiss_odometry_callback, this, std::placeholders::_1), subscriber_options);

    // Controllers map
    // Examples
    controller_map_["att-test"] = std::bind(&ArdupilotGuided::att_ref_test, this);
    controller_map_["vel-test"] = std::bind(&ArdupilotGuided::vel_ref_test, this);
    controller_map_["acc-test"] = std::bind(&ArdupilotGuided::acc_ref_test, this);
    // Custom controllers
    controller_map_["vel-sk"] = std::bind(&ArdupilotGuided::vel_ref_stalk, this);
    controller_map_["vel-lp"] = std::bind(&ArdupilotGuided::vel_ref_lead_pursuit, this);
    controller_map_["acc-pn"] = std::bind(&ArdupilotGuided::acc_ref_proportional_navigation, this);
    // Lemniscate trajectories
    controller_map_["vel-s8"] = [this]() { vel_ref_lemniscate({{-25.0, 0.0}, {20.0, 90.0}, {40.0, 45.0}, 8.0}); }; // ENU {min, max}, speed ceiling
    controller_map_["vel-l8"] = [this]() { vel_ref_lemniscate({{-50.0, -15.0}, {20.0, 200.0}, {40.0, 50.0}, 10.0}); }; // ENU {min, max}, speed ceiling
}

// Callbacks for subscribers (reentrant group)
void ArdupilotGuided::global_position_global_sub_callback(const NavSatFix::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->latitude;
    lon_ = msg->longitude;
    alt_ellipsoid_ = msg->altitude; // Positive is above the WGS 84 ellipsoid
}
void ArdupilotGuided::local_position_odom_callback(const Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    position_[0] = msg->pose.pose.position.x; // ENU
    position_[1] = msg->pose.pose.position.y;
    position_[2] = msg->pose.pose.position.z;
    q_[0] = msg->pose.pose.orientation.w;
    q_[1] = msg->pose.pose.orientation.x;
    q_[2] = msg->pose.pose.orientation.y;
    q_[3] = msg->pose.pose.orientation.z;
    velocity_[0] = msg->twist.twist.linear.x; // Body frame
    velocity_[1] = msg->twist.twist.linear.y;
    velocity_[2] = msg->twist.twist.linear.z;
    angular_velocity_[0] = msg->twist.twist.angular.x; // TODO: double-check
    angular_velocity_[1] = msg->twist.twist.angular.y;
    angular_velocity_[2] = msg->twist.twist.angular.z;
    // See also topics /mavros/local_position/velocity_body, /mavros/local_position/velocity_local
}
void ArdupilotGuided::global_position_local_callback(const Odometry::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    // Position (ENU -> NED)
    x_ = msg->pose.pose.position.y;  // N <- E
    y_ = msg->pose.pose.position.x;  // E <- N
    z_ = -msg->pose.pose.position.z; // D <- -U
    // Velocity (ENU -> NED), NOTE: PX4's vx_, vy_, vz_ map to vn_, ve_, -vu_ instead
    vx_ = msg->twist.twist.linear.y;  // N <- E
    vy_ = msg->twist.twist.linear.x;  // E <- N
    vz_ = -msg->twist.twist.linear.z; // D <- -U
}
void ArdupilotGuided::local_position_vel_local_callback(const TwistStamped::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    // Velocity (World ENU)
    ve_ = msg->twist.linear.x;
    vn_ = msg->twist.linear.y;
    vu_ = msg->twist.linear.z;
}
void ArdupilotGuided::vfr_hud_callback(const VfrHud::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    alt_ = msg->altitude; // MSL
    heading_ = msg->heading; // degrees 0..360, also in /mavros/global_position/compass_hdg
    true_airspeed_m_s_ = msg->airspeed; // m/s
}
void ArdupilotGuided::offboard_flag_callback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg)
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
void ArdupilotGuided::ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ground_tracks_ = msg; // Save the smart pointer to the latest message
    last_track_time_ = this->get_clock()->now();

    // Invalidate the pursuit references: they will only be valid if fully recomputed below
    desired_bearing_rad_ = desired_elevation_rad_ = closing_distance_ = NAN;
    target_vn_ = target_ve_ = target_vd_ = NAN;

    // Verify ArduPilot own position
    double own_lat = lat_;
    double own_lon = lon_;
    double own_alt = alt_;
    if (std::isnan(own_lat) || std::isnan(own_lon) || std::isnan(own_alt)) {
        RCLCPP_WARN_ONCE(get_logger(), "Waiting for own position");
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

    // Ignore track if stale
    if (target_track.time_since_last_update_s > 2.0) { // TODO: parametrize
        RCLCPP_WARN(get_logger(), "Target track is stale");
        return;
    }
    // Also gate in any controller with a block like:
    //      if (!std::isnan(desired_bearing_rad_) && !std::isnan(desired_elevation_rad_) && !std::isnan(closing_distance_) &&
    //          !std::isnan(target_vn_) && !std::isnan(target_ve_) && !std::isnan(target_vd_) &&
    //          (this->get_clock()->now() - last_track_time_).seconds() < 2.0) {
    // In case topic /tracks goes silent an this callback does not run again

    // Save target velocities
    target_vn_ = target_track.velocity_n_m_s;
    target_ve_ = target_track.velocity_e_m_s;
    target_vd_ = target_track.velocity_d_m_s;

    // Predict LLA position of target
    const double PREDICTION_TIME_SEC = static_cast<double>(target_track.time_since_last_update_s); // Dead-reckon based on the (ground-side) telemetry age

    double target_ground_speed = std::hypot(target_track.velocity_n_m_s, target_track.velocity_e_m_s);
    double target_course_rad = std::atan2(target_track.velocity_e_m_s, target_track.velocity_n_m_s); // Azimuth from North
    double target_course_deg = target_course_rad * (180.0 / M_PI);
    double distance_traveled = target_ground_speed * PREDICTION_TIME_SEC;

    double future_lat = 0.0, future_lon = 0.0;
    geod.Direct(target_track.latitude_deg, target_track.longitude_deg, target_course_deg, distance_traveled,
                future_lat, future_lon);
    double future_alt = target_track.altitude_m - (target_track.velocity_d_m_s * PREDICTION_TIME_SEC);

    // Compute relative spherical position (bearing, elevation, distance) of the target from the ArduPilot vehicle
    double fw_azi = 0.0, bw_azi = 0.0; // forward and backward azimuth (in degrees, clockwise from North)
    geod.Inverse(own_lat, own_lon, future_lat, future_lon,
                closing_distance_, fw_azi, bw_azi);
    desired_bearing_rad_ = fw_azi * (M_PI / 180.0);
    desired_elevation_rad_ = std::atan2((future_alt - own_alt), closing_distance_);
}

void ArdupilotGuided::yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    if (msg->header.frame_id == "camera_frame_0") { // Only process the primary camera
        yolo_detections_ = msg; // Save the smart pointer to the latest message
    }
}

void ArdupilotGuided::kiss_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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
void ArdupilotGuided::ardupilot_interface_printout_callback()
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
void ArdupilotGuided::offboard_loop_callback()
{
    offboard_loop_count_++; // Counter to monitor the rate of the offboard loop (no lock, atomic variable)
    std::shared_lock<std::shared_mutex> lock(node_data_mutex_); // Use shared_lock for data reads
    if (!offboard_active_) {
        return; // Do not publish anything else if not in OFFBOARD state
    }
    if (active_controller_func_ != nullptr) {
        active_controller_func_(); // If offboard is active AND we have a valid controller, run it
    } else {
        RCLCPP_WARN(get_logger(), "Unknown controller requested: '%s', no reference will be published", active_controller_name_.c_str());
    }
}

// Utility
double ArdupilotGuided::normalize_heading(double angle_rad) {
    while (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
}

// Controllers (reference generators)
void ArdupilotGuided::att_ref_test()
{
    auto att_msg = mavros_msgs::msg::AttitudeTarget(); // https://docs.ros.org/en/noetic/api/mavros_msgs/html/msg/AttitudeTarget.html
    att_msg.header.stamp = this->get_clock()->now();
    att_msg.header.frame_id = "map"; // World frame
    double pitch_rad = 5.0 * M_PI / 180.0; // Positive pitch to move forward (any duration)
    // Get current yaw and desired pitch
    double yaw_enu_rad = (M_PI / 2.0) - (heading_ * M_PI / 180.0); // Convert VfrHud compass degrees to ENU radians
    double cy = std::cos(yaw_enu_rad / 2.0);
    double sy = std::sin(yaw_enu_rad / 2.0);
    double cp = std::cos(pitch_rad / 2.0);
    double sp = std::sin(pitch_rad / 2.0);
    // Quaternion reference: Q_yaw * Q_pitch (the reference is in ROS ENU world frame)
    att_msg.orientation.w = cy * cp;
    att_msg.orientation.x = -sy * sp;
    att_msg.orientation.y = cy * sp;
    att_msg.orientation.z = sy * cp;
    att_msg.thrust = 0.5; // When the 3rd bit of GUID_OPTIONS is not set (e.g. GUID_OPTIONS 0) this is only a [0,1] climb-rate target
    // Ignore roll/pitch/yaw rates
    att_msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ROLL_RATE |
                        mavros_msgs::msg::AttitudeTarget::IGNORE_PITCH_RATE |
                        mavros_msgs::msg::AttitudeTarget::IGNORE_YAW_RATE;
    setpoint_raw_att_pub_->publish(att_msg);
}
void ArdupilotGuided::vel_ref_test()
{
    auto vel_msg = geometry_msgs::msg::TwistStamped(); // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    vel_msg.header.stamp = this->get_clock()->now();
    vel_msg.header.frame_id = "map"; // World frame, without automatic yaw alignment
    vel_msg.twist.linear.x = 3.0; // m/s East
    vel_msg.twist.linear.y = 3.0; // m/s North
    vel_msg.twist.linear.z = 0.0; // m/s Up
    // Computed yaw rate for alignment
    const double Kp_yaw = 1.5;
    double heading_error = normalize_heading(std::atan2(vel_msg.twist.linear.y, vel_msg.twist.linear.x) - ((M_PI / 2.0) - (heading_ * M_PI / 180.0)));
    vel_msg.twist.angular.z = Kp_yaw * heading_error; // rad/s Yaw rate
    setpoint_vel_pub_->publish(vel_msg);
    // Alternatively, use the unstamped topic: ros2 topic pub --rate 10 --times 50 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}}'
}
void ArdupilotGuided::acc_ref_test()
{
    auto accel_msg = geometry_msgs::msg::Vector3Stamped(); // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
    accel_msg.header.stamp = this->get_clock()->now();
    accel_msg.header.frame_id = "map"; // World frame, with automatic yaw alignment
    accel_msg.vector.x = 0.4; // m/s^2 East
    accel_msg.vector.y = 0.4; // m/s^2 North
    accel_msg.vector.z = 0.0; // m/s^2 Up
    setpoint_accel_pub_->publish(accel_msg);
}
void ArdupilotGuided::vel_ref_stalk()
{
    // Tunable parameters
    constexpr double STANDOFF_M = 10.0;   // m, desired horizontal distance to the target
    constexpr double ALT_OFFSET_M = 10.0; // m, fly this much above the target
    constexpr double DEADBAND_M = 2.0;    // m, ignore range errors below this (absorbs link delay/jitter)
    constexpr double KP_RANGE = 0.3;      // 1/s, range error to correction speed gain
    constexpr double MAX_CORR = 3.0;      // m/s, maximum correction speed (catch-up speed = target speed + correction)
    constexpr double V_MAX = 12.0;        // m/s, horizontal speed limit
    constexpr double KP_ALT = 0.4;        // 1/s, altitude error to climb rate gain
    constexpr double MAX_VZ = 1.5;        // m/s, climb/descent limit
    constexpr double KP_YAW = 0.8;        // 1/s, heading error to yaw rate gain
    constexpr double MAX_YAW_RATE = 0.5;  // rad/s, maximum yaw rate
    constexpr double LINK_STALE_S = 2.0;  // s, timeout after which to stop if topic /tracks goes silent

    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header.stamp = this->get_clock()->now();
    vel_msg.header.frame_id = "map"; // World frame, without automatic yaw alignment
    if (!std::isnan(desired_bearing_rad_) && !std::isnan(desired_elevation_rad_) && !std::isnan(closing_distance_) &&
        !std::isnan(target_vn_) && !std::isnan(target_ve_) && !std::isnan(target_vd_) &&
        (this->get_clock()->now() - last_track_time_).seconds() < LINK_STALE_S) {

        // Horizontal LOS unit vector in ENU (bearing is clockwise from North)
        double u_E = std::sin(desired_bearing_rad_);
        double u_N = std::cos(desired_bearing_rad_);
        // Horizontal range error with continuous deadband (>0: too far, <0: too close, back away)
        double range_error = closing_distance_ - STANDOFF_M;
        range_error = (std::abs(range_error) < DEADBAND_M) ? 0.0 : range_error - std::copysign(DEADBAND_M, range_error);
        // Target velocity feedforward (tracks arbitrary speed) + bounded range correction along the LOS
        double v_corr = std::clamp(KP_RANGE * range_error, -MAX_CORR, MAX_CORR);
        double v_E = target_ve_ + (v_corr * u_E);
        double v_N = target_vn_ + (v_corr * u_N);
        double v_norm = std::hypot(v_E, v_N);
        if (v_norm > V_MAX) { // Limit speed as a safety measure
            v_E *= V_MAX / v_norm;
            v_N *= V_MAX / v_norm;
        }
        vel_msg.twist.linear.x = v_E; // m/s East
        vel_msg.twist.linear.y = v_N; // m/s North

        // Decoupled altitude hold ALT_OFFSET_M above the target
        double dz_m = closing_distance_ * std::tan(desired_elevation_rad_);
        vel_msg.twist.linear.z = std::clamp(-target_vd_ + KP_ALT * (dz_m + ALT_OFFSET_M), -MAX_VZ, MAX_VZ); // m/s Up

        // Yaw rate to point at the target
        double heading_error = normalize_heading(((M_PI / 2.0) - desired_bearing_rad_) - ((M_PI / 2.0) - (heading_ * M_PI / 180.0)));
        vel_msg.twist.angular.z = std::isnan(heading_error) ? 0.0 : std::clamp(KP_YAW * heading_error, -MAX_YAW_RATE, MAX_YAW_RATE); // rad/s
    } else { // Invalid target info or the link is silent; zero-initialized vel_msg stops the vehicle
        RCLCPP_WARN(get_logger(), "No valid target info, entering vel_ref_stalk else branch");
    }
    setpoint_vel_pub_->publish(vel_msg);
}
void ArdupilotGuided::vel_ref_lead_pursuit()
{
    auto vel_msg = geometry_msgs::msg::TwistStamped(); // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
    vel_msg.header.stamp = this->get_clock()->now();
    vel_msg.header.frame_id = "map"; // World frame, without automatic yaw alignment
    if (!std::isnan(desired_bearing_rad_) && !std::isnan(desired_elevation_rad_) && !std::isnan(closing_distance_) &&
        !std::isnan(target_vn_) && !std::isnan(target_ve_) && !std::isnan(target_vd_) &&
        (this->get_clock()->now() - last_track_time_).seconds() < 2.0) { // TODO: parametrize

        // Calculate unit line-of-sight (LOS) vector in ENU
        double u_E = std::cos(desired_elevation_rad_) * std::sin(desired_bearing_rad_);
        double u_N = std::cos(desired_elevation_rad_) * std::cos(desired_bearing_rad_);
        double u_U = std::sin(desired_elevation_rad_);
        // Project target ENU velocity along the LOS (parallel, escape speed) and across it (perp, lateral drift speed)
        double vt_parallel_mag = (target_ve_ * u_E) + (target_vn_ * u_N) + (-target_vd_ * u_U);
        double vt_perp_E = target_ve_ - (vt_parallel_mag * u_E);
        double vt_perp_N = target_vn_ - (vt_parallel_mag * u_N);
        double vt_perp_U = -target_vd_ - (vt_parallel_mag * u_U);
        double vt_perp_mag_sq = (vt_perp_E * vt_perp_E) + (vt_perp_N * vt_perp_N) + (vt_perp_U * vt_perp_U);
        // Distance-based desired closing speed
        constexpr double MIN_RANGE_M = 10.0; // m, range below which the closing speed holds at its terminal value
        constexpr double MIN_CLOSING_SPEED_MS = 3.0; // m/s, terminal closing speed (0 for rendezvous; greater than 0 for hit)
        constexpr double K_CLOSE = 0.14; // 1/s, closing speed per meter beyond MIN_RANGE_M
        constexpr double V_MAX_MS = 10.0; // m/s, speed limit
        double slant_range_m = closing_distance_ / std::max(0.1, std::cos(desired_elevation_rad_)); // 3D range
        double max_closing_rate = std::sqrt(std::max(0.0, V_MAX_MS*V_MAX_MS - vt_perp_mag_sq)) - vt_parallel_mag; // Closing rate achievable at V_MAX_MS after matching drift and escape; negative if the target outruns V_MAX_MS
        double base_closing_speed = std::min(MIN_CLOSING_SPEED_MS + K_CLOSE * std::max(0.0, slant_range_m - MIN_RANGE_M), max_closing_rate);
        // Total desired speed along the LOS: target's escape speed + closing speed
        double vd_parallel_mag = vt_parallel_mag + base_closing_speed;
        // Final velocity reference: escape speed + closing speed + match perpendicular drift (based on pursuit type)
        const double K_pursuit = 1.0; //  1.0 = lead pursuit (match drift, intercept target)
                                        //  0.0 = pure pursuit (point directly at target, tail-chase)
                                        // -0.5 = lag pursuit (fall behind target's path)
        vel_msg.twist.linear.x = (K_pursuit * vt_perp_E) + (vd_parallel_mag * u_E); // m/s East
        vel_msg.twist.linear.y = (K_pursuit * vt_perp_N) + (vd_parallel_mag * u_N); // m/s North
        vel_msg.twist.linear.z = (K_pursuit * vt_perp_U) + (vd_parallel_mag * u_U); // m/s Up
    } else { // Missing track, stay still
        vel_msg.twist.linear.x = 0.0;
        vel_msg.twist.linear.y = 0.0;
        vel_msg.twist.linear.z = 0.0;
    }
    // Computed yaw rate for alignment
    const double Kp_yaw = 1.5;
    double cmd_speed = std::hypot(vel_msg.twist.linear.x, vel_msg.twist.linear.y);
    double heading_error = (cmd_speed > 0.1) ? normalize_heading(std::atan2(vel_msg.twist.linear.y, vel_msg.twist.linear.x) - ((M_PI / 2.0) - (heading_ * M_PI / 180.0))) : 0.0;
    vel_msg.twist.angular.z = std::isnan(heading_error) ? 0.0 : std::clamp(Kp_yaw * heading_error, -1.0, 1.0); // rad/s Yaw rate
    setpoint_vel_pub_->publish(vel_msg);
}
void ArdupilotGuided::acc_ref_proportional_navigation()
{
    auto accel_msg = geometry_msgs::msg::Vector3Stamped(); // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
    accel_msg.header.stamp = this->get_clock()->now();
    accel_msg.header.frame_id = "map"; // World frame, with automatic yaw alignment
    if (!std::isnan(desired_bearing_rad_) && !std::isnan(desired_elevation_rad_) && !std::isnan(closing_distance_) &&
        !std::isnan(target_vn_) && !std::isnan(target_ve_) && !std::isnan(target_vd_) &&
        !std::isnan(ve_) && !std::isnan(vn_) && !std::isnan(vu_) &&
        (this->get_clock()->now() - last_track_time_).seconds() < 2.0) { // TODO: parametrize

        // Calculate ENU error vector
        double r_E = closing_distance_ * std::sin(desired_bearing_rad_);
        double r_N = closing_distance_ * std::cos(desired_bearing_rad_);
        double r_U = closing_distance_ * std::tan(desired_elevation_rad_);
        double distance_3d = std::max(0.1, std::hypot(closing_distance_, r_U)); // Prevent divide-by-zero

        // Unit LOS Vector in ENU
        double u_E = r_E / distance_3d;
        double u_N = r_N / distance_3d;
        double u_U = r_U / distance_3d;

        // Relative ENU Velocity (target - own)
        double vrel_E = target_ve_ - ve_;
        double vrel_N = target_vn_ - vn_;
        double vrel_U = -target_vd_ - vu_;

        // Closing velocity (Vc = -(r dot vrel) / |r|)
        double r_dot_vrel = (r_E * vrel_E) + (r_N * vrel_N) + (r_U * vrel_U);
        double Vc = -r_dot_vrel / distance_3d;

        // Distance-based desired closing speed
        constexpr double MIN_RANGE_M = 10.0; // m, range below which the closing speed holds at its terminal value
        constexpr double MIN_CLOSING_SPEED_MS = 3.0; // m/s, terminal closing speed (0 for rendezvous; greater than 0 for hit)
        constexpr double K_CLOSE = 0.14; // 1/s, closing speed per meter beyond MIN_RANGE_M
        constexpr double V_MAX_MS = 10.0; // m/s, speed limit
        double vt_parallel_mag = (target_ve_ * u_E) + (target_vn_ * u_N) + (-target_vd_ * u_U); // Target escape speed along the LOS
        double vt_perp_mag_sq = (target_ve_*target_ve_ + target_vn_*target_vn_ + target_vd_*target_vd_) - (vt_parallel_mag * vt_parallel_mag);
        double max_closing_rate = std::sqrt(std::max(0.0, V_MAX_MS*V_MAX_MS - vt_perp_mag_sq)) - vt_parallel_mag; // Closing rate achievable at V_MAX_MS after matching drift and escape; negative if the target outruns V_MAX_MS
        double desired_Vc = std::min(MIN_CLOSING_SPEED_MS + K_CLOSE * std::max(0.0, distance_3d - MIN_RANGE_M), max_closing_rate);
        // Catch-Up acceleration (HORIZONTAL ONLY)
        double a_fwd_mag = std::clamp(0.5 * (desired_Vc - Vc), -2.0, 3.0);
        accel_msg.vector.x = a_fwd_mag * u_E;
        accel_msg.vector.y = a_fwd_mag * u_N;

        if (Vc > 0) { // Target is closing
            // LOS angular rate vector (omega = (r x vrel) / |r|^2)
            double r_sq = distance_3d * distance_3d;
            double omega_E = (r_N * vrel_U - r_U * vrel_N) / r_sq;
            double omega_N = (r_U * vrel_E - r_E * vrel_U) / r_sq;
            double omega_U = (r_E * vrel_N - r_N * vrel_E) / r_sq;

            // PN steering acceleration (HORIZONTAL ONLY)
            const double N_gain = 3.0;
            double a_pn_E = N_gain * Vc * (omega_N * u_U - omega_U * u_N);
            double a_pn_N = N_gain * Vc * (omega_U * u_E - omega_E * u_U);

            accel_msg.vector.x += a_pn_E;
            accel_msg.vector.y += a_pn_N;
        }

        // Account for ArduPilot attitude control limits
        constexpr double ANGLE_MAX_CDEG = 3000.0; // Note: matches param ANGLE_MAX, ensure WPNAV_ACCEL is set to 500
        const double MAX_HORIZ_ACCEL = 9.81 * std::tan((ANGLE_MAX_CDEG / 100.0) * (M_PI / 180.0));
        double a_horiz_mag = std::hypot(accel_msg.vector.x, accel_msg.vector.y);
        if (a_horiz_mag > MAX_HORIZ_ACCEL) {
            double scale = MAX_HORIZ_ACCEL / a_horiz_mag;
            accel_msg.vector.x *= scale;
            accel_msg.vector.y *= scale;
        }

        // Decoupled z-axis PD altitude controller (clamped to bounds)
        const double Kp_Z = 0.3;
        const double Kd_Z = 1.5;
        accel_msg.vector.z = (Kp_Z * r_U) + (Kd_Z * vrel_U);
        accel_msg.vector.z = std::clamp(accel_msg.vector.z, -1.5, 1.5);

    } else { // Missing track, break
        const double K_brake = 1.0; // Braking gain (1.0 means try to stop in ~1 second)
        accel_msg.vector.x = std::isnan(ve_) ? 0.0 : -K_brake * ve_;
        accel_msg.vector.y = std::isnan(vn_) ? 0.0 : -K_brake * vn_;
        accel_msg.vector.z = std::isnan(vu_) ? 0.0 : -K_brake * vu_;
        double brake_mag = std::hypot(accel_msg.vector.x, accel_msg.vector.y);
        const double MAX_BRAKE_ACCEL = 3.0; // Clamp horizontal braking deceleration
        if (brake_mag > MAX_BRAKE_ACCEL) {
            double scale = MAX_BRAKE_ACCEL / brake_mag;
            accel_msg.vector.x *= scale;
            accel_msg.vector.y *= scale;
        }
        accel_msg.vector.z = std::clamp(accel_msg.vector.z, -1.0, 1.0); // Limit vertical deceleration
    }
    setpoint_accel_pub_->publish(accel_msg);
}
void ArdupilotGuided::vel_ref_lemniscate(const Lemniscate &loop)
{
    constexpr double SPEED_RATIO = 1.6;   // Speed ceiling ratio, as a multiple of what the tightest turn allows
    constexpr double A_LAT_MAX_MS2 = 3.0; // m/s^2, lateral acceleration limit
    constexpr double MAX_VZ_MS = 2.0;     // m/s, climb/descent limit
    constexpr double LOOKAHEAD_S = 0.6;   // s, lookahead in seconds (at the current speed)
    constexpr double SCAN_RAD = 0.5;      // rad, forward-only search window
    constexpr int SCAN_STEPS = 60;        // Samples in the window
    constexpr double KP_YAW = 0.8;        // 1/s, heading error to yaw rate gain
    constexpr double MAX_YAW_RATE = 0.5;  // rad/s, maximum yaw rate

    const double east_center_m = 0.5 * (loop.east_m[1] + loop.east_m[0]);
    const double east_amplitude_m = 0.5 * (loop.east_m[1] - loop.east_m[0]);
    const double north_center_m = 0.5 * (loop.north_m[1] + loop.north_m[0]);
    const double north_amplitude_m = 0.5 * (loop.north_m[1] - loop.north_m[0]);
    const double alt_center_m = 0.5 * (loop.alt_m[1] + loop.alt_m[0]);
    const double alt_amplitude_m = 0.5 * (loop.alt_m[1] - loop.alt_m[0]);

    // Scale the speed ceiling with the sharper between the radius at the lobe tips or at the lobe sides (not the true minimum)
    const double r_min_m = std::min(north_amplitude_m * north_amplitude_m / (8.0 * east_amplitude_m),
                                    4.0 * east_amplitude_m * east_amplitude_m / north_amplitude_m);
    const double v_ceiling_ms = std::min(loop.v_max_ms, SPEED_RATIO * std::sqrt(A_LAT_MAX_MS2 * r_min_m));
    // The path: phase 0 is the south tip, east cycles twice per lap and that is the crossing
    auto path_point = [&](double phase_rad) {
        return std::array<double, 3>{east_center_m + east_amplitude_m * std::sin(2.0 * phase_rad),
                                     north_center_m - north_amplitude_m * std::cos(phase_rad),
                                     alt_center_m + alt_amplitude_m * std::sin(phase_rad)};
    };

    auto vel_msg = geometry_msgs::msg::TwistStamped();
    vel_msg.header.stamp = this->get_clock()->now();
    vel_msg.header.frame_id = "map"; // World frame, without automatic yaw alignment
    if (!std::isnan(position_[0]) && !std::isnan(position_[1]) && !std::isnan(position_[2])) {

        // Slide the phase onto the nearest path point ahead of the last one
        double best_phase = lemniscate_phase_rad_, best_dist_sq = -1.0;
        for (int i = 0; i <= SCAN_STEPS; ++i) {
            double phase = lemniscate_phase_rad_ + (SCAN_RAD * i / SCAN_STEPS);
            auto p = path_point(phase);
            double dx = p[0] - position_[0], dy = p[1] - position_[1]; // Horizontal only
            double dist_sq = (dx * dx) + (dy * dy);
            if (best_dist_sq < 0.0 || dist_sq < best_dist_sq) { best_dist_sq = dist_sq; best_phase = phase; }
        }
        lemniscate_phase_rad_ = std::fmod(best_phase, 2.0 * M_PI); // Keep the phase bounded over long flights

        double s = std::sin(lemniscate_phase_rad_);
        double c = std::cos(lemniscate_phase_rad_);
        // Tangent to the path
        double dE = 2.0 * east_amplitude_m * std::cos(2.0 * lemniscate_phase_rad_);
        double dN = north_amplitude_m * s;
        double dU = alt_amplitude_m * c;
        double dpath = std::max(1e-3, std::sqrt(dE*dE + dN*dN + dU*dU)); // Path length per radian of phase, floored to prevent divide-by-zero
        double turn_rate = 2.0 * east_amplitude_m * north_amplitude_m * std::abs(c) * (1.0 + 2.0 * s * s); // |E'N'' - N'E''|: denominator of the curvature formula
        double r_curv = std::pow(dE*dE + dN*dN, 1.5) / std::max(1e-3, turn_rate); // Horizontal radius, the crossing is an inflection point where turn_rate is zero: prevent divide-by-zero
        double v_ref = std::min(std::sqrt(A_LAT_MAX_MS2 * r_curv), v_ceiling_ms); // Speed set by the local turn: slow in the lobes, fast through the crossing

        // Fly at a lookahead point further along the path
        auto lookahead_point = path_point(lemniscate_phase_rad_ + (LOOKAHEAD_S * v_ref / dpath));
        double to_E = lookahead_point[0] - position_[0];
        double to_N = lookahead_point[1] - position_[1];
        double to_U = lookahead_point[2] - position_[2];
        double to_norm = std::max(1e-3, std::sqrt(to_E*to_E + to_N*to_N + to_U*to_U)); // Floored to prevent divide-by-zero
        vel_msg.twist.linear.x = v_ref * to_E / to_norm; // m/s East
        vel_msg.twist.linear.y = v_ref * to_N / to_norm; // m/s North
        vel_msg.twist.linear.z = std::clamp(v_ref * to_U / to_norm, -MAX_VZ_MS, MAX_VZ_MS); // m/s Up

        // Yaw rate to point along the commanded horizontal velocity
        double heading_error = normalize_heading(std::atan2(vel_msg.twist.linear.y, vel_msg.twist.linear.x) - ((M_PI / 2.0) - (heading_ * M_PI / 180.0)));
        vel_msg.twist.angular.z = std::isnan(heading_error) ? 0.0 : std::clamp(KP_YAW * heading_error, -MAX_YAW_RATE, MAX_YAW_RATE); // rad/s
    } else { // No local position; zero-initialized vel_msg stops the vehicle
        RCLCPP_WARN(get_logger(), "No valid local position, entering vel_ref_lemniscate else branch");
    }
    setpoint_vel_pub_->publish(vel_msg);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<ArdupilotGuided>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
