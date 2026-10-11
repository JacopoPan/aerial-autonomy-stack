#include "betaflight_rc_override.hpp"

BetaflightRcOverride::BetaflightRcOverride() : Node("betaflight_rc_override"),
    offboard_active_(false), active_controller_name_(""),
    offboard_loop_frequency(50), offboard_loop_count_(0), last_offboard_loop_count_(0), own_id_(-1),
    lat_(NAN), lon_(NAN), alt_(NAN), ve_(NAN), vn_(NAN), vu_(NAN), heading_(NAN),
    ground_tracks_(nullptr), yolo_detections_(nullptr),
    desired_bearing_rad_(NAN), desired_elevation_rad_(NAN), closing_distance_(NAN),
    target_vn_(NAN), target_ve_(NAN), target_vd_(NAN),
    detect_az_rad_(NAN), detect_el_rad_(NAN), detect_fix_count_(0),
    active_controller_func_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "Betaflight RC override referencing!");
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
    angular_velocity_.fill(NAN);
    kiss_position_.fill(NAN);
    kiss_q_.fill(NAN);
    detect_fix_enu_.fill(NAN);

    // Parameters
    search_classes_ = this->declare_parameter<std::vector<std::string>>("search_classes", {"car", "truck"});
    camera_extrinsics_ = this->declare_parameter<std::vector<double>>("camera_extrinsics", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (camera_extrinsics_.size() != 6) { camera_extrinsics_.assign(6, 0.0); RCLCPP_ERROR(this->get_logger(), "camera_extrinsics needs 6 values, falling back to a level camera"); }
    RCLCPP_INFO(this->get_logger(), "Camera extrinsics: x %.2f y %.2f z %.2f - roll %.1f pitch %.1f yaw %.1f (deg)",
                camera_extrinsics_[0], camera_extrinsics_[1], camera_extrinsics_[2], camera_extrinsics_[3], camera_extrinsics_[4], camera_extrinsics_[5]);

    // betaflight_interface publisher
    rc_override_pub_ = this->create_publisher<Joy>("rc_override", 10); // Reliable, as betaflight_interface's subscriber

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_printout_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Strictly sequential callbacks
    callback_group_offboard_control_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive); // Strictly sequential callbacks
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel

    // Timers
    betaflight_interface_printout_timer_ = this->create_wall_timer( // Follow wall clock for printouts
        3s, // Timer period of 3 seconds
        std::bind(&BetaflightRcOverride::betaflight_interface_printout_callback, this),
        callback_group_printout_
    );
    offboard_control_loop_timer_ = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(1000000000 / offboard_loop_frequency),
        std::bind(&BetaflightRcOverride::offboard_loop_callback, this),
        callback_group_offboard_control_
    );

    // Subscribers configuration
    auto subscriber_options = rclcpp::SubscriptionOptions();
    subscriber_options.callback_group = callback_group_subscriber_;
    rclcpp::QoS qos_profile_sub(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    qos_profile_sub.keep_last(10);  // History: KEEP_LAST with depth 10
    qos_profile_sub.reliability(rclcpp::ReliabilityPolicy::BestEffort);

    // betaflight_interface subscribers
    global_position_sub_ = this->create_subscription<NavSatFix>(
        "global_position", qos_profile_sub, // 50Hz
        std::bind(&BetaflightRcOverride::global_position_callback, this, std::placeholders::_1), subscriber_options);
    local_position_sub_ = this->create_subscription<PointStamped>(
        "local_position", qos_profile_sub, // 50Hz, from the first arming
        std::bind(&BetaflightRcOverride::local_position_callback, this, std::placeholders::_1), subscriber_options);
    velocity_sub_ = this->create_subscription<TwistStamped>(
        "velocity", qos_profile_sub, // 50Hz
        std::bind(&BetaflightRcOverride::velocity_callback, this, std::placeholders::_1), subscriber_options);
    attitude_sub_ = this->create_subscription<Vector3Stamped>(
        "attitude", qos_profile_sub, // 50Hz
        std::bind(&BetaflightRcOverride::attitude_callback, this, std::placeholders::_1), subscriber_options);
    imu_sub_ = this->create_subscription<Imu>(
        "imu", qos_profile_sub, // 50Hz
        std::bind(&BetaflightRcOverride::imu_callback, this, std::placeholders::_1), subscriber_options);

    // Offboard flag subscriber
    offboard_flag_sub_ = this->create_subscription<autopilot_interface_msgs::msg::OffboardFlag>(
        "/offboard_flag", qos_profile_sub, // 10Hz
        std::bind(&BetaflightRcOverride::offboard_flag_callback, this, std::placeholders::_1), subscriber_options);

    // Perception subscribers
    ground_tracks_sub_ = this->create_subscription<ground_system_msgs::msg::SwarmObs>(
        "/tracks", qos_profile_sub, // 10Hz
        std::bind(&BetaflightRcOverride::ground_tracks_callback, this, std::placeholders::_1), subscriber_options);
    yolo_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections", qos_profile_sub, // 15Hz
        std::bind(&BetaflightRcOverride::yolo_detections_callback, this, std::placeholders::_1), subscriber_options);
    kiss_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/kiss/odometry", qos_profile_sub, // 10Hz
        std::bind(&BetaflightRcOverride::kiss_odometry_callback, this, std::placeholders::_1), subscriber_options);

    // Controllers map
    // Examples, one per flight mode
    controller_map_["ctbr-test"] = std::bind(&BetaflightRcOverride::ctbr_ref_test, this);
    controller_map_["att-test"] = std::bind(&BetaflightRcOverride::att_ref_test, this);
    controller_map_["vel-test"] = std::bind(&BetaflightRcOverride::vel_ref_test, this);
}

// Callbacks for subscribers (reentrant group)
void BetaflightRcOverride::global_position_callback(const NavSatFix::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->latitude;
    lon_ = msg->longitude;
    alt_ = msg->altitude; // MSL, 1m steps
}
void BetaflightRcOverride::local_position_callback(const PointStamped::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    position_[0] = msg->point.x; // ENU from the arming point
    position_[1] = msg->point.y;
    position_[2] = msg->point.z;
}
void BetaflightRcOverride::velocity_callback(const TwistStamped::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    // Velocity (World ENU)
    ve_ = msg->twist.linear.x;
    vn_ = msg->twist.linear.y;
    vu_ = msg->twist.linear.z;
}
void BetaflightRcOverride::attitude_callback(const Vector3Stamped::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    heading_ = msg->vector.z; // Clockwise from North
    // FLU body to ENU world quaternion from roll (right wing down positive), pitch (nose down positive), and the ENU yaw of the heading
    double cr = std::cos(msg->vector.x / 2.0), sr = std::sin(msg->vector.x / 2.0);
    double cp = std::cos(msg->vector.y / 2.0), sp = std::sin(msg->vector.y / 2.0);
    double cy = std::cos(((M_PI / 2.0) - heading_) / 2.0), sy = std::sin(((M_PI / 2.0) - heading_) / 2.0);
    q_[0] = (cr * cp * cy) + (sr * sp * sy); // w
    q_[1] = (sr * cp * cy) - (cr * sp * sy); // x
    q_[2] = (cr * sp * cy) + (sr * cp * sy); // y
    q_[3] = (cr * cp * sy) - (sr * sp * cy); // z
}
void BetaflightRcOverride::imu_callback(const Imu::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    angular_velocity_[0] = msg->angular_velocity.x; // FLU
    angular_velocity_[1] = msg->angular_velocity.y;
    angular_velocity_[2] = msg->angular_velocity.z;
}
void BetaflightRcOverride::offboard_flag_callback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg)
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
void BetaflightRcOverride::ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ground_tracks_ = msg; // Save the smart pointer to the latest message
    last_track_time_ = this->get_clock()->now();

    // Invalidate the pursuit references: they will only be valid if fully recomputed below
    desired_bearing_rad_ = desired_elevation_rad_ = closing_distance_ = NAN;
    target_vn_ = target_ve_ = target_vd_ = NAN;

    // Verify Betaflight own position
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

    // Compute relative spherical position (bearing, elevation, distance) of the target from the Betaflight vehicle
    double fw_azi = 0.0, bw_azi = 0.0; // forward and backward azimuth (in degrees, clockwise from North)
    geod.Inverse(own_lat, own_lon, future_lat, future_lon,
                closing_distance_, fw_azi, bw_azi);
    desired_bearing_rad_ = fw_azi * (M_PI / 180.0);
    desired_elevation_rad_ = std::atan2((future_alt - own_alt), closing_distance_);
}

void BetaflightRcOverride::yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    if (msg->header.frame_id == "camera_frame_0") { // Only process the primary camera
        yolo_detections_ = msg; // Save the smart pointer to the latest message

        // Track the most confident box in search_classes_ and fix it to the ground
        constexpr double MIN_LOOK_DOWN_DEG = 10.0;  // deg, the shallowest look-down angle that still gives a usable range
        constexpr double FIX_MATCH_RADIUS_M = 15.0; // m, a new fix within this radius is the same object, outside it is a different one
        double best_score = 0.0;
        for (const auto& detection : msg->detections) {
            if (detection.results.empty()) { continue; }
            const auto& detection_result = detection.results.front(); // YOLO26 is NMS-free and argmax'd, so there is exactly one hypothesis per box
            if (detection_result.hypothesis.score <= best_score || std::find(search_classes_.begin(), search_classes_.end(), detection_result.hypothesis.class_id) == search_classes_.end()) { continue; }
            best_score = detection_result.hypothesis.score;
            detect_az_rad_ = detection_result.pose.pose.position.x * (M_PI / 180.0);
            detect_el_rad_ = detection_result.pose.pose.position.y * (M_PI / 180.0);
            std::array<double, 3> los = camera_bearings_to_enu(detect_az_rad_, detect_el_rad_);
            last_detect_time_ = this->get_clock()->now(); // Stamped before the fix is attempted, so the angles stay usable when it is not
            double look_down_rad = -std::asin(los[2]); // Angle of the line of sight below the horizon, positive when it points at the ground
            if (!(look_down_rad > (MIN_LOOK_DOWN_DEG * M_PI / 180.0)) || !(position_[2] > 0.0)) { continue; } // Negated so a NAN, which compares false either way, is rejected rather than let through
            double range_m = position_[2] / std::sin(look_down_rad); // The ground is assumed flat at zero altitude, the arming point where betaflight_interface puts the local frame origin
            double fix_E = position_[0] + (range_m * los[0]), fix_N = position_[1] + (range_m * los[1]);
            if (std::isnan(detect_fix_enu_[0]) || std::hypot(fix_E - detect_fix_enu_[0], fix_N - detect_fix_enu_[1]) > FIX_MATCH_RADIUS_M) {
                detect_fix_enu_ = {fix_E, fix_N, 0.0}; detect_fix_count_ = 1; // First sighting, or a different object: start a new hypothesis at ground level
            } else {
                detect_fix_count_++;
                detect_fix_enu_[0] += (fix_E - detect_fix_enu_[0]) / detect_fix_count_;
                detect_fix_enu_[1] += (fix_N - detect_fix_enu_[1]) / detect_fix_count_;
            }
        }
    }
}

void BetaflightRcOverride::kiss_odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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
void BetaflightRcOverride::betaflight_interface_printout_callback()
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
                ss << "  Id " << track.id
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
    if (detect_fix_count_ > 0) {
        ss << "Target fix: E " << std::fixed << std::setprecision(1) << detect_fix_enu_[0] << " N " << detect_fix_enu_[1] << " - sightings: " << detect_fix_count_ << " - age: " << (now - last_detect_time_).seconds() << "s\n";
    }
    RCLCPP_INFO(get_logger(), "%s\n", ss.str().c_str());
}
void BetaflightRcOverride::offboard_loop_callback()
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
double BetaflightRcOverride::normalize_heading(double angle_rad) {
    while (angle_rad > M_PI) {
        angle_rad -= 2.0 * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0 * M_PI;
    }
    return angle_rad;
}
std::array<double, 3> BetaflightRcOverride::camera_bearings_to_enu(double az_rad, double el_rad)
{
    double pitch_rad = camera_extrinsics_[4] * (M_PI / 180.0); // Mount pitch, positive is boresight down
    double cx = std::cos(el_rad) * std::cos(az_rad), cz = std::sin(el_rad);
    double bx = (cx * std::cos(pitch_rad)) + (cz * std::sin(pitch_rad)); // Pitch the boresight down onto the body frame
    double by = -std::cos(el_rad) * std::sin(az_rad); // Azimuth is positive right, FLU y is positive left, and the mount pitch leaves it alone
    double bz = (cz * std::cos(pitch_rad)) - (cx * std::sin(pitch_rad));
    // Rotate body FLU into world ENU with the attitude at detection time, which removes airframe pitch and bank
    double tx = 2.0 * ((q_[2] * bz) - (q_[3] * by)), ty = 2.0 * ((q_[3] * bx) - (q_[1] * bz)), tz = 2.0 * ((q_[1] * by) - (q_[2] * bx));
    return {bx + (q_[0] * tx) + ((q_[2] * tz) - (q_[3] * ty)),  // East
            by + (q_[0] * ty) + ((q_[3] * tx) - (q_[1] * tz)),  // North
            bz + (q_[0] * tz) + ((q_[1] * ty) - (q_[2] * tx))}; // Up
}
void BetaflightRcOverride::publish_rc_override(double roll, double pitch, double throttle, double yaw, RcMode mode)
{
    Joy rc_override; // Sticks in [-1, 1] and the flight mode switch position, see betaflight_interface.cpp
    rc_override.header.stamp = this->get_clock()->now();
    rc_override.axes = {static_cast<float>(std::clamp(roll, -1.0, 1.0)), static_cast<float>(std::clamp(pitch, -1.0, 1.0)),
                        static_cast<float>(std::clamp(throttle, -1.0, 1.0)), static_cast<float>(std::clamp(yaw, -1.0, 1.0))};
    rc_override.buttons = {static_cast<int>(mode)};
    rc_override_pub_->publish(rc_override);
}
void BetaflightRcOverride::publish_rates_ref(double roll_rate, double pitch_rate, double yaw_rate, double throttle)
{
    // ACRO: FLU body rates in rad/s (roll right wing down, pitch nose down, yaw counterclockwise positive), throttle in [0, 1]
    publish_rc_override(roll_rate / MAX_RATE_RAD_S, pitch_rate / MAX_RATE_RAD_S, (2.0 * throttle) - 1.0, -yaw_rate / MAX_RATE_RAD_S, RcMode::ACRO); // Yaw stick right turns clockwise
}
void BetaflightRcOverride::publish_attitude_ref(double roll, double pitch, double yaw_rate, double throttle)
{
    // ANGLE: FLU roll and pitch in rad (right wing down, nose down positive), yaw rate in rad/s (counterclockwise positive), throttle in [0, 1]
    publish_rc_override(roll / ANGLE_LIMIT_RAD, pitch / ANGLE_LIMIT_RAD, (2.0 * throttle) - 1.0, -yaw_rate / MAX_RATE_RAD_S, RcMode::ANGLE);
}
void BetaflightRcOverride::publish_velocity_ref(double v_east, double v_north, double v_up, double yaw_rate)
{
    // ALTHOLD + POS HOLD: ENU velocity in m/s, yaw rate in rad/s (counterclockwise positive)
    double v_forward = (v_east * std::sin(heading_)) + (v_north * std::cos(heading_)); // Pitch and roll sticks are forward and right of the heading, POS HOLD rotates them back
    double v_right = (v_east * std::cos(heading_)) - (v_north * std::sin(heading_));
    publish_rc_override(v_right / FULL_STICK_SPEED_MS, v_forward / FULL_STICK_SPEED_MS, v_up / FULL_STICK_CLIMB_MS, -yaw_rate / MAX_RATE_RAD_S, RcMode::VELOCITY);
}

// Controllers (reference generators)
void BetaflightRcOverride::ctbr_ref_test()
{
    publish_rates_ref(0.0, 0.0, -1.0, HOVER_THROTTLE); // Spin clockwise on itself (any duration), as PX4's FRD yaw rate 1.0, ACRO does not level the drone
}
void BetaflightRcOverride::att_ref_test()
{
    double pitch_rad = 5.0 * M_PI / 180.0; // Positive pitch to move forward (any duration, drops some altitude)
    publish_attitude_ref(0.0, pitch_rad, 0.0, HOVER_THROTTLE);
}
void BetaflightRcOverride::vel_ref_test()
{
    if (std::isnan(heading_)) {
        RCLCPP_WARN(get_logger(), "No heading yet, no reference will be published");
        return;
    }
    double v_east = 3.0; // m/s
    double v_north = 3.0; // m/s
    // Computed yaw rate for alignment
    const double Kp_yaw = 1.5;
    double heading_error = normalize_heading(std::atan2(v_north, v_east) - ((M_PI / 2.0) - heading_)); // ENU yaw of the velocity minus the current one
    publish_velocity_ref(v_east, v_north, 0.0, Kp_yaw * heading_error);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<BetaflightRcOverride>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
