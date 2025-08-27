#include "px4_offboard.hpp"

PX4Offboard::PX4Offboard() : Node("px4_offboard"), 
    offboard_flag_(0),
    offboard_loop_frequency(50), offboard_loop_count_(0), last_offboard_loop_count_(0),
    lat_(NAN), lon_(NAN), alt_(NAN), alt_ellipsoid_(NAN),
    xy_valid_(false), z_valid_(false), v_xy_valid_(false), v_z_valid_(false), xy_global_(false), z_global_(false),
    x_(NAN), y_(NAN), z_(NAN), heading_(NAN), vx_(NAN), vy_(NAN), vz_(NAN), ref_lat_(NAN), ref_lon_(NAN), ref_alt_(NAN),
    pose_frame_(-1), velocity_frame_(-1), true_airspeed_m_s_(NAN),
    ground_tracks_(nullptr), yolo_detections_(nullptr)
{
    RCLCPP_INFO(this->get_logger(), "PX4 offboard referencing!");
    RCLCPP_INFO(this->get_logger(), "namespace: %s", this->get_namespace());
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
    attitude_ref_pub_ = this->create_publisher<VehicleAttitudeSetpoint>("fmu/in/vehicle_attitude_setpoint", qos_profile_pub);
    rates_ref_pub_ = this->create_publisher<VehicleRatesSetpoint>("fmu/in/vehicle_rates_setpoint", qos_profile_pub);
    trajectory_ref_pub_ = this->create_publisher<TrajectorySetpoint>("fmu/in/trajectory_setpoint", qos_profile_pub);

    // Create callback groups (Reentrant or MutuallyExclusive)
    callback_group_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Timed callbacks in parallel
    callback_group_subscriber_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant); // Listen to subscribers in parallel

    // Timers
    px4_interface_printout_timer_ = this->create_wall_timer(
        3s, // Timer period of 3 seconds
        std::bind(&PX4Offboard::px4_interface_printout_callback, this),
        callback_group_timer_
    );
    offboard_control_loop_timer_ = this->create_wall_timer(
        std::chrono::nanoseconds(1000000000 / offboard_loop_frequency),
        std::bind(&PX4Offboard::offboard_loop_callback, this),
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
        std::bind(&PX4Offboard::global_position_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_local_position_sub_= this->create_subscription<VehicleLocalPosition>(
        "fmu/out/vehicle_local_position", qos_profile_sub, // 100Hz
        std::bind(&PX4Offboard::local_position_callback, this, std::placeholders::_1), subscriber_options);
    vehicle_odometry_sub_= this->create_subscription<VehicleOdometry>(
        "fmu/out/vehicle_odometry", qos_profile_sub, // 100Hz
        std::bind(&PX4Offboard::odometry_callback, this, std::placeholders::_1), subscriber_options);
    airspeed_validated_sub_ = this->create_subscription<AirspeedValidated>(
        "fmu/out/airspeed_validated", qos_profile_sub, // 10Hz
        std::bind(&PX4Offboard::airspeed_callback, this, std::placeholders::_1), subscriber_options);

    // Offboard flag subscriber
    offboard_flag_sub_ = this->create_subscription<autopilot_interface_msgs::msg::OffboardFlag>(
        "/offboard_flag", qos_profile_sub, // 10Hz
        std::bind(&PX4Offboard::offboard_flag_callaback, this, std::placeholders::_1), subscriber_options);

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
}

// Callbacks for subscribers (reentrant group)
void PX4Offboard::global_position_callback(const VehicleGlobalPosition::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    lat_ = msg->lat;
    lon_ = msg->lon;
    alt_ = msg->alt; // AMSL
    alt_ellipsoid_ = msg->alt_ellipsoid; // TODO: double-check
    // New to v1.16
    // bool lat_lon_valid
    // bool alt_valid
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
void PX4Offboard::offboard_flag_callaback(const autopilot_interface_msgs::msg::OffboardFlag::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    offboard_flag_ = msg->offboard_flag;
}
void PX4Offboard::ground_tracks_callback(const ground_system_msgs::msg::SwarmObs::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    ground_tracks_ = msg; // Save the smart pointer to the latest message
}

void PX4Offboard::yolo_detections_callback(const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
    std::unique_lock<std::shared_mutex> lock(node_data_mutex_); // Use unique_lock for data writes
    yolo_detections_ = msg; // Save the smart pointer to the latest message
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
                << " alt: " << std::fixed << std::setprecision(2) << track.altitude_m << "\n";
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
                    ss << "  Label: " << result.hypothesis.class_id
                    << " - conf: " << std::fixed << std::setprecision(2) << result.hypothesis.score << "\n";
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

    uint64_t current_time_us = this->get_clock()->now().nanoseconds() / 1000;  // Convert to microseconds
    OffboardControlMode offboard_mode;
    offboard_mode.timestamp = current_time_us;
    if (offboard_flag_ == 0) {
        return; // Do not publish anything else if not in an OFFBOARD state
    // TODO: implement custom offboard control logic here
    // https://docs.px4.io/v1.15/en/flight_modes/offboard.html
    } else if (offboard_flag_ == 1) { // Quad attitude reference
        offboard_mode.attitude = true;
        VehicleAttitudeSetpoint attitude_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/VehicleAttitudeSetpoint.msg
        attitude_ref.timestamp = current_time_us;
        double pitch_rad = -5.0 * M_PI / 180.0; // Pitch to move forward (any duration, drops some altitude)
        attitude_ref.q_d[0] = cos(pitch_rad / 2.0); // w
        attitude_ref.q_d[1] = 0;                    // x
        attitude_ref.q_d[2] = sin(pitch_rad / 2.0); // y
        attitude_ref.q_d[3] = 0;                    // z
        attitude_ref.thrust_body = {0.0, 0.0, -0.72};
        attitude_ref_pub_->publish(attitude_ref);
    } else if (offboard_flag_ == 2) { // VTOL attitude reference
        offboard_mode.attitude = true;
        VehicleAttitudeSetpoint attitude_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/VehicleAttitudeSetpoint.msg
        attitude_ref.timestamp = current_time_us;
        double pitch_rad = -30.0 * M_PI / 180.0; // Pitch to dive
        attitude_ref.q_d[0] = cos(pitch_rad / 2.0); // w
        attitude_ref.q_d[1] = 0;                    // x
        attitude_ref.q_d[2] = sin(pitch_rad / 2.0); // y
        attitude_ref.q_d[3] = 0;                    // z
        attitude_ref.thrust_body = {0.15, 0.0, 0.0};
        attitude_ref_pub_->publish(attitude_ref);
    } else if (offboard_flag_ == 3) { // Quad rates reference
        offboard_mode.body_rate = true;
        VehicleRatesSetpoint rates_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/VehicleRatesSetpoint.msg
        rates_ref.timestamp = current_time_us;
        rates_ref.roll= 0.0;
        rates_ref.pitch = 0.0;
        rates_ref.yaw = 1.0; // Spin on itself (any duration)
        rates_ref.thrust_body = {0.0, 0.0, -0.72};
        rates_ref_pub_->publish(rates_ref);
    } else if (offboard_flag_ == 4) { // VTOL rates reference
        offboard_mode.body_rate = true;
        VehicleRatesSetpoint rates_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/VehicleRatesSetpoint.msg
        rates_ref.timestamp = current_time_us;
        rates_ref.roll= 4.0; // Roll (2sec maneuver 1 roll, 3sec double roll)
        rates_ref.pitch = 0.0;
        rates_ref.thrust_body = {0.39, 0.0, 0.0};
        rates_ref_pub_->publish(rates_ref);
    } else if (offboard_flag_ == 5) { // Quad trajectory (position) reference
        TrajectorySetpoint trajectory_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/TrajectorySetpoint.msg
        trajectory_ref.timestamp = current_time_us;
        offboard_mode.position = true;
        trajectory_ref.position = {0.0, 0.0, -20.0};
        trajectory_ref.yaw = -3.14; // [-PI:PI]
        // offboard_mode.acceleration = true;
        // trajectory_ref.acceleration = {0.0, 0.0, -5.0};
        trajectory_ref_pub_->publish(trajectory_ref);
    } else if (offboard_flag_ == 6) { // VTOL trajectory (velocity) reference
        TrajectorySetpoint trajectory_ref; // https://github.com/PX4/px4_msgs/blob/release/1.16/msg/TrajectorySetpoint.msg
        trajectory_ref.timestamp = current_time_us;
        offboard_mode.velocity = true;
        trajectory_ref.velocity = {20.0, 0.0, 0.0};
        trajectory_ref_pub_->publish(trajectory_ref);
    } else {
        RCLCPP_WARN(get_logger(), "Unexpected offboard_flag value: %d", offboard_flag_.load());
    }

    if (offboard_loop_count_ % std::max(1, (offboard_loop_frequency / 10)) == 0) {
        offboard_mode_pub_->publish(offboard_mode); // The OffboardControlMode should run at at least 2Hz (~10 in this implementation)
    }
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
