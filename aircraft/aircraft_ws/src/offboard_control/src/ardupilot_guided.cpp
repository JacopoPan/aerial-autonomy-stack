#include "ardupilot_guided.hpp"

ArdupilotGuided::ArdupilotGuided() : Node("ardupilot_guided"),
    offboard_flag_(0),
    offboard_loop_frequency(10), offboard_loop_count_(0), last_offboard_loop_count_(0),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    x_(NAN), y_(NAN), z_(NAN),  vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    true_airspeed_m_s_(NAN), heading_(NAN),
    ground_tracks_(nullptr), yolo_detections_(nullptr),
    desired_bearing_rad(NAN), desired_elevation_rad_(NAN), closing_distance_(NAN)
{
    RCLCPP_INFO(this->get_logger(), "ArduPilot guided referencing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
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
    rclcpp::QoS qos_profile_pub(10);  // Depth of 10
    qos_profile_pub.durability(rclcpp::DurabilityPolicy::TransientLocal);  // Or rclcpp::DurabilityPolicy::Volatile
    setpoint_accel_pub_= this->create_publisher<Vector3Stamped>("/mavros/setpoint_accel/accel", qos_profile_pub);
    setpoint_vel_pub_= this->create_publisher<TwistStamped>("/mavros/setpoint_velocity/cmd_vel", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Timed callbacks in parallel
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel

    // Timers
    ardupilot_interface_printout_timer_ = this->create_wall_timer( // Follow wall clock for printouts
        3s, // Timer period of 3 seconds
        std::bind(&ArdupilotGuided::ardupilot_interface_printout_callback, this),
        callback_group_timer_
    );
    offboard_control_loop_timer_ = rclcpp::create_timer(this, this->get_clock(),
        std::chrono::nanoseconds(1000000000 / offboard_loop_frequency),
        std::bind(&ArdupilotGuided::offboard_loop_callback, this),
        callback_group_timer_
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
    mavros_vfr_hud_sub_ = this->create_subscription<VfrHud>(
        "/mavros/vfr_hud", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::vfr_hud_callback, this, std::placeholders::_1), subscriber_options);

    // Offboard flag subscriber
    offboard_flag_sub_ = this->create_subscription<autopilot_interface_msgs::msg::OffboardFlag>(
        "/offboard_flag", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::offboard_flag_callaback, this, std::placeholders::_1), subscriber_options);

    // Perception subscribers
    ground_tracks_sub_ = this->create_subscription<ground_system_msgs::msg::SwarmObs>(
        "/tracks", qos_profile_sub, // 1Hz
        std::bind(&ArdupilotGuided::ground_tracks_callback, this, std::placeholders::_1), subscriber_options);
    yolo_detections_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
        "/detections", qos_profile_sub, // 15Hz
        std::bind(&ArdupilotGuided::yolo_detections_callback, this, std::placeholders::_1), subscriber_options);
    kiss_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/kiss/odometry", qos_profile_sub, // 10Hz
        std::bind(&ArdupilotGuided::kiss_odometry_callback, this, std::placeholders::_1), subscriber_options);
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
    angular_velocity_[0] = msg->twist.twist.angular.x; // TODO: double check
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
    // Velocity (ENU -> NED)
    vx_ = msg->twist.twist.linear.y;  // N <- E
    vy_ = msg->twist.twist.linear.x;  // E <- N
    vz_ = -msg->twist.twist.linear.z; // D <- -U
}
void ArdupilotGuided::vfr_hud_callback(const VfrHud::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    alt_ = msg->altitude; // MSL
    heading_ = msg->heading; // degrees 0..360, also in /mavros/global_position/compass_hdg
    true_airspeed_m_s_ = msg->airspeed; // m/s
}
void ArdupilotGuided::offboard_flag_callaback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    offboard_flag_ = msg->offboard_flag;
}
void ArdupilotGuided::ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ground_tracks_ = msg; // Save the smart pointer to the latest message

    double label48_lat = 0.0;
    double label48_lon = 0.0;
    double label48_alt = 0.0;
    double label48_vn = 0.0;
    double label48_ve = 0.0;
    double label48_vd = 0.0;
    bool label48_found = false;
    for (const auto& track : ground_tracks_->tracks) {
        if (track.label == 48) {
            label48_lat = track.latitude_deg;
            label48_lon = track.longitude_deg;
            label48_alt = track.altitude_m;
            label48_vn = track.velocity_n_m_s;
            label48_ve = track.velocity_e_m_s;
            label48_vd = track.velocity_d_m_s;
            label48_found = true;
            break;
        }
    }
    if (!label48_found) {
        RCLCPP_WARN_ONCE(get_logger(), "Label 48 not found in tracks.");
        return;
    }
    double own_lat = lat_;
    double own_lon = lon_;
    double own_alt = alt_;
    if (std::isnan(own_lat) || std::isnan(own_lon)) {
        RCLCPP_WARN_ONCE(get_logger(), "Waiting for own position");
        return;
    }
    // Predict position
    const double prediction_time_sec = 0.0; // TODO: enable prediction
    double target_ground_speed = std::sqrt(label48_vn * label48_vn + label48_ve * label48_ve);
    double target_course_rad = std::atan2(label48_ve, label48_vn); // Azimuth from North
    double target_course_deg = target_course_rad * 180.0 / M_PI;
    double distance_traveled = target_ground_speed * prediction_time_sec;
    double future_lat, future_lon;
    geod.Direct(label48_lat, label48_lon, target_course_deg, distance_traveled, future_lat, future_lon);
    double future_alt = label48_alt - (label48_vd * prediction_time_sec) + 2.0; // HARDCODED: track from above the target to avoid collisions
    // Compute bearing and elevation
    double fw_azi, bw_azi; // forward azimuth (in degrees, clockwise from North)
    geod.Inverse(own_lat, own_lon, future_lat, future_lon, closing_distance_, fw_azi, bw_azi);        
    desired_bearing_rad = fw_azi * M_PI / 180.0; // TODO: altitude is not considered
    desired_elevation_rad_ = std::atan2((future_alt - own_alt), closing_distance_);
}

void ArdupilotGuided::yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    yolo_detections_ = msg; // Save the smart pointer to the latest message
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
                "  Offboard flag:\t%d\n"
                "  Offboard loop rate:\t%.2f Hz",
                this->get_clock()->now().seconds(),
                kiss_position_[0], kiss_position_[1], kiss_position_[2],
                offboard_flag_.load(),
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
    if (offboard_flag_ == 0) {
        return; // Do not publish anything else if not in an OFFBOARD state
    // TODO: implement custom offboard control logic here
    } else if (offboard_flag_ == 7) { // Velocity setpoint
        auto vel_msg = geometry_msgs::msg::TwistStamped(); // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html
        vel_msg.header.stamp = this->get_clock()->now();
        vel_msg.header.frame_id = "map"; // World frame, without automatic yaw alignment
        vel_msg.twist.linear.x = 0.0; // m/s East
        vel_msg.twist.linear.y = 5.0; // m/s North
        vel_msg.twist.linear.z = 0.0; // m/s Up
        double distance_fraction = (closing_distance_ - 3.0) / 50.0; // Factor to add speed if further than 3m, up to ~50m
        distance_fraction = std::clamp(distance_fraction, 0.0, 1.0);
        double desired_speed = 3.0 + distance_fraction * (5.0); // m/s base speed of 3.0 m/s, increasing up to 8.0 m/s when far (~50m or more) from the target
        if (!std::isnan(desired_bearing_rad) && !std::isnan(desired_elevation_rad_)) {
            double horizontal_speed = desired_speed * std::cos(desired_elevation_rad_);
            double vertical_speed = desired_speed * std::sin(desired_elevation_rad_);
            vel_msg.twist.linear.x = horizontal_speed * std::sin(desired_bearing_rad); // m/s East
            vel_msg.twist.linear.y = horizontal_speed * std::cos(desired_bearing_rad); // m/s North
            vel_msg.twist.linear.z = vertical_speed; // m/s Up
        }
        // Computed yaw rate for alignment
        const double Kp_yaw = 1.5;
        double heading_error = normalize_heading(std::atan2(vel_msg.twist.linear.y, vel_msg.twist.linear.x) - ((M_PI / 2.0) - (heading_ * M_PI / 180.0)));
        vel_msg.twist.angular.z = Kp_yaw * heading_error; // rad/s Yaw rate
        setpoint_vel_pub_->publish(vel_msg);
        // Alternatively, use the unstamped topic: ros2 topic pub --rate 10 --times 50 /mavros/setpoint_velocity/cmd_vel_unstamped geometry_msgs/msg/Twist '{linear: {x: 2.0, y: 0.0, z: 0.0}}'
    } else if (offboard_flag_ == 8) { // Acceleration setpoint
        auto accel_msg = geometry_msgs::msg::Vector3Stamped(); // https://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Vector3.html
        accel_msg.header.stamp = this->get_clock()->now();
        accel_msg.header.frame_id = "map"; // World frame, with automatic yaw alignment
        accel_msg.vector.x = 0.0; // m/s^2 East
        accel_msg.vector.y = 1.5; // m/s^2 North
        accel_msg.vector.z = 0.0; // m/s^2 Up
        setpoint_accel_pub_->publish(accel_msg);
    } else {
        RCLCPP_WARN(get_logger(), "Unexpected offboard_flag value: %d", offboard_flag_.load());
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
