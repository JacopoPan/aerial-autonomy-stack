/*
Minimal Betaflight interface: takeoff, land and offboard actions, reposition service, and autopilot state from MSP replies

Betaflight FCU interface:
- tcp://host:port for the SITL (UARTn listens on 5760 + n + port offset)
- serial port on hardware (e.g., /dev/ttyACM0)
Unlike the PX4 and ArduPilot interfaces that rely on an external ROS2 middleware (uXRCE-DDS and MAVROS), this node directly handles MSP

RC channels sent at 50 Hz with MSP_SET_RAW_RC, which Betaflight applies to CH1-6 while the radio's MSP OVERRIDE switch is on (see betaflight-2026.6.cli):
    CH1-4 roll, pitch, throttle, yaw sticks; CH5 ARM; CH6 flight mode; CH7-8 unused
    CH6 low = ACRO (rates), mid = ANGLE (attitude), high = ANGLE + ALTHOLD + POS HOLD (velocity)
In ALTHOLD + POS HOLD, once armed with the throttle raised, the sticks are velocity commands and centered sticks hold position and altitude:
    roll, pitch = horizontal velocity (full stick = ap_max_velocity), throttle = climb rate (around ap_hover_throttle), yaw = yaw rate

Actions and services (no orbit or speed), positions are from the arming point, altitudes above it:
    takeoff_action: arm, climb to takeoff_altitude
    set_reposition: fly to east, north, altitude (the service returns once the target is accepted)
    land_action: fly back over the arming point at landing_altitude, descend, disarm on touchdown
    offboard_action: fly the sticks and flight mode from rc_override for max_duration_sec, then hover

Use with:
    ros2 action send_goal /Drone${DRONE_ID}/takeoff_action autopilot_interface_msgs/action/Takeoff "{takeoff_altitude: 10.0}"
    ros2 service call /Drone${DRONE_ID}/set_reposition autopilot_interface_msgs/srv/SetReposition "{east: 20.0, north: 10.0, altitude: 15.0}"
    ros2 action send_goal /Drone${DRONE_ID}/offboard_action autopilot_interface_msgs/action/Offboard "{controller_name: 'vel-test', max_duration_sec: 10.0}"
    ros2 action send_goal /Drone${DRONE_ID}/land_action autopilot_interface_msgs/action/Land "{landing_altitude: 10.0}"

Offboard: while offboard_action runs, /offboard_flag names the controller that betaflight_rc_override (offboard_control) runs
    It publishes rc_override (sensor_msgs/Joy): axes[0..3] = roll, pitch, throttle, yaw sticks in [-1, 1], buttons[0] = CH6 (0 low, 1 mid, 2 high)
    The throttle axis is the collective thrust in ACRO and ANGLE (-1 idle), the climb rate in ALTHOLD + POS HOLD (0 holds the altitude)
    Without rc_override in the last 0.5 s, the sticks center in ALTHOLD + POS HOLD and the drone holds its position

MSP budget: Betaflight answers one MSP request per port per serial task run, serial_update_rate_hz times per second
    (default 100Hz, set to 500Hz in betaflight-2026.6.cli), this node sends 100/s (50 MSP_SET_RAW_RC, 50 MSP_MULTIPLE_MSP)

Topics published at 50 Hz, from one MSP_MULTIPLE_MSP request (five replies) every timer tick:
    attitude (geometry_msgs/Vector3Stamped): roll, pitch in rad (FLU: right wing down, nose down positive), heading in rad (clockwise from north)
    imu (sensor_msgs/Imu): FLU angular velocity (rad/s) and acceleration (m/s^2), from MSP_RAW_IMU raw sensor units
    global_position (sensor_msgs/NavSatFix): latitude, longitude, MSL altitude (1 m steps)
    local_position (geometry_msgs/PointStamped): east, north of the arming point and altitude above it, in m, from the first arming
    velocity (geometry_msgs/TwistStamped): ENU, from the GPS ground speed and course, and the estimated vertical speed
And /offboard_flag (autopilot_interface_msgs/OffboardFlag) at 10 Hz, as px4_interface and ardupilot_interface
*/
#include "betaflight_interface.hpp"

BetaflightInterface::BetaflightInterface() : Node("betaflight_interface"),
    state_cmds_({MSP_RAW_IMU, MSP_ATTITUDE, MSP_ALTITUDE, MSP_RAW_GPS, MSP_STATUS_EX}), fd_(-1), acc_1g_(2048),
    status_received_(false), armed_(false), gps_fix_(false), hover_read_(false), arming_disable_flags_(0), hover_pwm_(0),
    latitude_(0.0), longitude_(0.0), altitude_(0.0), vertical_speed_(0.0), ground_speed_(0.0), heading_(0.0),
    state_(State::INIT), state_since_(0, 0, RCL_ROS_TIME), descended_(false), quiet_since_(0, 0, RCL_ROS_TIME), home_set_(false),
    target_({0.0, 0.0, 0.0}), takeoff_altitude_(0.0), offboard_duration_(0.0), rc_override_time_(0, 0, RCL_ROS_TIME)
{
    // Parameters
    const std::string device = this->declare_parameter<std::string>("device", "tcp://127.0.0.1:5762");
    acc_1g_ = this->declare_parameter<int>("acc_1g", 2048); // MSP_RAW_IMU accelerometer units per g: 2048 on most FC IMUs (16 g range), 256 in the SITL

    fd_ = open_device(device);
    RCLCPP_INFO(this->get_logger(), "MSP on %s", device.c_str());

    // Publishers
    attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude", 10); // Roll, pitch, heading in rad
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
    global_position_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("global_position", 10);
    local_position_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("local_position", 10);
    velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
    offboard_flag_pub_ = this->create_publisher<autopilot_interface_msgs::msg::OffboardFlag>("/offboard_flag", 10);

    // Subscribers
    rc_override_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("rc_override", 10, [this](const sensor_msgs::msg::Joy &msg) {
        rc_override_ = msg;
        rc_override_time_ = this->now();
    });

    // Actions and services
    takeoff_server_ = rclcpp_action::create_server<Takeoff>(this, "takeoff_action",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Takeoff::Goal>) { return accept_takeoff(); },
        [](GoalHandle<Takeoff>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](GoalHandle<Takeoff> goal) { start_takeoff(goal); });
    land_server_ = rclcpp_action::create_server<Land>(this, "land_action",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Land::Goal>) { return accept_land(); },
        [](GoalHandle<Land>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](GoalHandle<Land> goal) { start_land(goal); });
    offboard_server_ = rclcpp_action::create_server<Offboard>(this, "offboard_action",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Offboard::Goal>) { return accept_offboard(); },
        [](GoalHandle<Offboard>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](GoalHandle<Offboard> goal) { start_offboard(goal); });
    reposition_service_ = this->create_service<SetReposition>("set_reposition",
        [this](const std::shared_ptr<SetReposition::Request> request, std::shared_ptr<SetReposition::Response> response) {
            reposition(*request, *response);
        });

    // Timers
    timer_ = rclcpp::create_timer(this, this->get_clock(), 20ms, [this]() { step(); }); // 50 Hz on the node clock (sim time in simulation)
    offboard_flag_timer_ = rclcpp::create_timer(this, this->get_clock(), 100ms, [this]() {
        autopilot_interface_msgs::msg::OffboardFlag msg;
        msg.is_active = state_ == State::OFFBOARD;
        msg.controller_name = msg.is_active ? offboard_controller_ : "";
        offboard_flag_pub_->publish(msg);
    });
}
BetaflightInterface::~BetaflightInterface() { close(fd_); }

int BetaflightInterface::open_device(const std::string &device)
{
    int fd = -1;
    if (device.rfind("tcp://", 0) == 0) {
        const std::string host_port = device.substr(6);
        const size_t colon = host_port.rfind(':');
        addrinfo hints{}, *res = nullptr;
        hints.ai_socktype = SOCK_STREAM;
        if (getaddrinfo(host_port.substr(0, colon).c_str(), host_port.substr(colon + 1).c_str(), &hints, &res) == 0) {
            fd = socket(res->ai_family, res->ai_socktype, 0);
            if (connect(fd, res->ai_addr, res->ai_addrlen) != 0) {
                close(fd);
                fd = -1;
            }
            freeaddrinfo(res);
        }
    } else {
        fd = open(device.c_str(), O_RDWR | O_NOCTTY);
        termios tty{};
        if (fd >= 0 && tcgetattr(fd, &tty) == 0) { // Raw 8N1 at 115200 (USB VCPs ignore the baud rate)
            cfmakeraw(&tty);
            cfsetspeed(&tty, B115200);
            tcsetattr(fd, TCSANOW, &tty);
        }
    }
    if (fd < 0) {
        throw std::runtime_error("Cannot open " + device);
    }
    fcntl(fd, F_SETFL, O_NONBLOCK); // step() reads whatever is available
    return fd;
}

void BetaflightInterface::send_msp(uint8_t cmd, const std::vector<uint8_t> &payload) // MSP v1 request: $M< size cmd payload checksum
{
    std::vector<uint8_t> frame = {'$', 'M', '<', static_cast<uint8_t>(payload.size()), cmd};
    frame.insert(frame.end(), payload.begin(), payload.end());
    uint8_t checksum = 0;
    for (size_t i = 3; i < frame.size(); i++) {
        checksum ^= frame[i];
    }
    frame.push_back(checksum);
    if (write(fd_, frame.data(), frame.size()) != static_cast<ssize_t>(frame.size())) {
        throw std::runtime_error("MSP write failed");
    }
}

void BetaflightInterface::step()
{
    uint8_t buf[256];
    ssize_t n = 0;
    while ((n = read(fd_, buf, sizeof(buf))) > 0) {
        rx_.insert(rx_.end(), buf, buf + n);
    }
    if (n == 0) {
        throw std::runtime_error("MSP link closed");
    }
    const std::string header = "$M>"; // MSP v1 reply: $M> size cmd payload checksum
    while (true) {
        auto start = std::search(rx_.begin(), rx_.end(), header.begin(), header.end());
        rx_.erase(rx_.begin(), start == rx_.end() ? rx_.end() - static_cast<std::ptrdiff_t>(std::min<size_t>(rx_.size(), 2)) : start); // Keep a split header
        if (rx_.size() < 6 || rx_.size() < 6u + rx_[3]) {
            break;
        }
        uint8_t checksum = 0; // XOR of size, cmd and payload, as in send_msp()
        for (size_t i = 3; i < 5u + rx_[3]; i++) {
            checksum ^= rx_[i];
        }
        if (checksum == rx_[5u + rx_[3]]) { // Skip corrupted replies: e.g., a status read as disarmed would make this node send ARM low
            handle(rx_[4], &rx_[5], rx_[3]);
        }
        rx_.erase(rx_.begin(), rx_.begin() + 6 + rx_[3]);
    }
    update_state();
    if (state_ != State::INIT) { // Until Betaflight's first status, sending ARM low could disarm a drone in flight
        send_msp(MSP_SET_RAW_RC, rc_channels());
    }
    if (!hover_read_) {
        send_msp(MSP_GPS_RESCUE); // Once, for ap_hover_throttle
    }
    send_msp(MSP_MULTIPLE_MSP, state_cmds_);
}

void BetaflightInterface::handle(uint8_t cmd, const uint8_t *d, size_t size) // Publish or store the payload of an MSP reply
{
    auto u16 = [d](size_t k) { return static_cast<uint16_t>(d[k] | d[k + 1] << 8); };
    auto s16 = [&u16](size_t k) { return static_cast<int16_t>(u16(k)); };
    auto u32 = [&u16](size_t k) { return u16(k) | static_cast<uint32_t>(u16(k + 2)) << 16; };
    auto s32 = [&u32](size_t k) { return static_cast<int32_t>(u32(k)); };
    if (cmd == MSP_MULTIPLE_MSP) { // The replies to state_cmds_, each as size and payload
        for (size_t i = 0, k = 0; k < state_cmds_.size() && i < size && i + 1 + d[i] <= size; i += 1 + d[i], k++) {
            handle(state_cmds_[k], d + i + 1, d[i]);
        }
    } else if (cmd == MSP_RAW_IMU && size >= 12) { // Acceleration in acc_1g units per g, angular velocity in 16.384 units per deg/s (2000 deg/s range), FLU
        sensor_msgs::msg::Imu msg;
        msg.header.stamp = this->now();
        msg.orientation_covariance[0] = -1.0; // No orientation
        msg.linear_acceleration.x = s16(0) * 9.80665 / acc_1g_;
        msg.linear_acceleration.y = s16(2) * 9.80665 / acc_1g_;
        msg.linear_acceleration.z = s16(4) * 9.80665 / acc_1g_;
        msg.angular_velocity.x = s16(6) / 16.384 * M_PI / 180.0;
        msg.angular_velocity.y = s16(8) / 16.384 * M_PI / 180.0;
        msg.angular_velocity.z = s16(10) / 16.384 * M_PI / 180.0;
        imu_pub_->publish(msg);
    } else if (cmd == MSP_ATTITUDE && size >= 6) { // Roll and pitch in 0.1 deg, yaw (heading) in deg
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = this->now();
        msg.vector.x = s16(0) * M_PI / 1800.0;
        msg.vector.y = s16(2) * M_PI / 1800.0;
        msg.vector.z = s16(4) * M_PI / 180.0;
        heading_ = msg.vector.z;
        attitude_pub_->publish(msg);
    } else if (cmd == MSP_ALTITUDE && size >= 6) { // Altitude in cm (from the arming point once armed), vertical speed in cm/s
        altitude_ = s32(0) / 100.0;
        vertical_speed_ = s16(4) / 100.0;
    } else if (cmd == MSP_RAW_GPS && size >= 16) { // Fix, satellites, lat, lon in 1e-7 deg, MSL altitude in m, speed in cm/s, course in 0.1 deg
        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp = this->now();
        fix.status.status = d[0] ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        fix.latitude = s32(2) / 1e7;
        fix.longitude = s32(6) / 1e7;
        fix.altitude = u16(10);
        global_position_pub_->publish(fix);
        gps_fix_ = d[0];
        latitude_ = fix.latitude;
        longitude_ = fix.longitude;
        if (home_set_) {
            geometry_msgs::msg::PointStamped local; // ENU, in the frame of set_reposition
            local.header.stamp = fix.header.stamp;
            const auto [east, north, up] = position();
            local.point.x = east;
            local.point.y = north;
            local.point.z = up;
            local_position_pub_->publish(local);
        }
        geometry_msgs::msg::TwistStamped vel; // ENU, as MAVROS's local_position/velocity_local
        vel.header.stamp = fix.header.stamp;
        const double speed = u16(12) / 100.0, course = u16(14) * M_PI / 1800.0;
        ground_speed_ = speed;
        vel.twist.linear.x = speed * std::sin(course); // East
        vel.twist.linear.y = speed * std::cos(course); // North
        vel.twist.linear.z = vertical_speed_; // Up
        velocity_pub_->publish(vel);
    } else if (cmd == MSP_STATUS_EX && size >= 16 && size >= 21u + d[15]) { // Active modes (ARM first), extra mode bytes, arming disable flags
        armed_ = d[6] & 1;
        if (u32(17 + d[15]) != arming_disable_flags_ && !armed_) {
            RCLCPP_INFO(this->get_logger(), "Arming disable flags: 0x%08X", u32(17 + d[15]));
        }
        arming_disable_flags_ = u32(17 + d[15]);
        status_received_ = true;
    } else if (cmd == MSP_GPS_RESCUE && size >= 14) { // ap_hover_throttle at byte 12
        hover_read_ = true;
        hover_pwm_ = u16(12);
        if (hover_pwm_ == 0) {
            RCLCPP_ERROR(this->get_logger(), "ap_hover_throttle is 0 (auto): set it in the CLI, the throttle stick is centered on it");
        }
    }
}

void BetaflightInterface::update_state() // State transitions and action results, once per tick
{
    if (takeoff_goal_ && takeoff_goal_->is_canceling()) {
        finish(takeoff_goal_, false, "Takeoff canceled");
        if (armed_) {
            hover_here();
        } else {
            set_state(State::STARTED);
        }
    }
    if (land_goal_ && land_goal_->is_canceling()) {
        finish(land_goal_, false, "Landing canceled");
        hover_here();
    }
    if (offboard_goal_ && offboard_goal_->is_canceling()) {
        finish(offboard_goal_, false, "Offboard canceled");
        hover_here();
    }
    if (state_ == State::INIT && status_received_) {
        if (armed_) {
            home_.Reset(latitude_, longitude_, 0.0);
            home_set_ = true;
            RCLCPP_WARN(this->get_logger(), "Betaflight is already armed: hovering, with home at the current position");
            hover_here();
        } else {
            set_state(State::STARTED);
        }
    } else if (state_ == State::ARMING && armed_) {
        home_.Reset(latitude_, longitude_, 0.0);
        home_set_ = true;
        target_ = {0.0, 0.0, takeoff_altitude_};
        set_state(State::REPOSITIONING);
        feedback(takeoff_goal_, "Armed, climbing to " + std::to_string(takeoff_altitude_) + " m");
    } else if (state_ == State::ARMING && (this->now() - state_since_).seconds() > ARMING_TIMEOUT) {
        finish(takeoff_goal_, false, "Arming failed, arming disable flags: " + hex(arming_disable_flags_));
        set_state(State::STARTED);
    } else if (state_ == State::REPOSITIONING && arrived()) {
        if (land_goal_) {
            set_state(State::LANDING);
            descended_ = false;
            feedback(land_goal_, "Over the arming point, descending");
        } else {
            set_state(State::HOVER);
            if (takeoff_goal_) {
                finish(takeoff_goal_, true, "Takeoff completed");
            }
        }
    } else if (state_ == State::LANDING) { // Touchdown: the descent began, then the drone stopped, vertically and horizontally
        const bool descending = vertical_speed_ < -0.25 * LAND_SPEED;
        descended_ = descended_ || descending;
        if (descending || ground_speed_ > ARRIVAL_SPEED) {
            quiet_since_ = this->now();
        } else if (descended_ && (this->now() - quiet_since_).seconds() > TOUCHDOWN_TIME) {
            finish(land_goal_, true, "Landed");
            set_state(State::STARTED); // ARM low disarms
        }
    } else if (state_ == State::OFFBOARD && (this->now() - state_since_).seconds() > offboard_duration_) {
        finish(offboard_goal_, true, "Offboard completed");
        hover_here();
    }
    if (!armed_ && (state_ == State::REPOSITIONING || state_ == State::HOVER || state_ == State::LANDING || state_ == State::OFFBOARD)) { // Disarmed, not by this node (e.g., failsafe)
        if (takeoff_goal_) {
            finish(takeoff_goal_, false, "Disarmed by Betaflight");
        }
        if (land_goal_) {
            finish(land_goal_, false, "Disarmed by Betaflight");
        }
        if (offboard_goal_) {
            finish(offboard_goal_, false, "Disarmed by Betaflight");
        }
        set_state(State::STARTED);
    }
}

std::vector<uint8_t> BetaflightInterface::rc_channels() const // MSP_SET_RAW_RC payload for the current state
{
    std::array<double, 8> ch = {1500.0, 1500.0, 1000.0, 1500.0, 1000.0, 1000.0, 1000.0, 1000.0}; // Sticks centered, throttle and switches low
    if (state_ != State::STARTED) {
        ch[4] = 2000.0; // ARM (ALTHOLD and POS HOLD stay off until armed, as they disable arming)
    }
    if (state_ == State::REPOSITIONING || state_ == State::HOVER || state_ == State::LANDING || state_ == State::OFFBOARD) {
        ch[5] = 2000.0; // ANGLE + ALTHOLD + POS HOLD
        ch[2] = hold_throttle(0.0); // With centered sticks, Betaflight holds the position and altitude
    }
    if (state_ == State::REPOSITIONING || state_ == State::HOVER || state_ == State::LANDING) { // Also steer in HOVER: after a move, Betaflight's own hold can settle meters away
        const auto [east, north, up] = offset();
        const double gain = std::min(POSITION_GAIN, FULL_STICK_SPEED / std::max(std::hypot(east, north), 1e-6)); // Speed proportional to the distance, up to full stick
        const double ve = gain * east, vn = gain * north; // m/s
        ch[0] = 1500.0 + 500.0 * (ve * std::cos(heading_) - vn * std::sin(heading_)) / FULL_STICK_SPEED; // Roll (right)
        ch[1] = 1500.0 + 500.0 * (ve * std::sin(heading_) + vn * std::cos(heading_)) / FULL_STICK_SPEED; // Pitch (forward)
        const double climb = state_ == State::LANDING ? -LAND_SPEED : std::clamp(ALTITUDE_GAIN * up, -MAX_CLIMB, MAX_CLIMB); // m/s
        ch[2] = hold_throttle(climb / FULL_STICK_CLIMB);
    } else if (state_ == State::OFFBOARD && rc_override_valid()) { // The offboard controller's sticks and flight mode
        const auto &axes = rc_override_.axes;
        const int mode = rc_override_.buttons[0]; // 0 ACRO, 1 ANGLE, 2 ANGLE + ALTHOLD + POS HOLD
        ch[0] = 1500.0 + 500.0 * axes[0];
        ch[1] = 1500.0 + 500.0 * axes[1];
        ch[2] = mode == 2 ? hold_throttle(axes[2]) : 1500.0 + 500.0 * axes[2]; // Climb rate in ALTHOLD, collective thrust otherwise
        ch[3] = 1500.0 + 500.0 * axes[3];
        ch[5] = 1000.0 + 500.0 * mode;
    }
    std::vector<uint8_t> payload;
    for (const double value : ch) {
        const auto pwm = static_cast<uint16_t>(std::clamp(value, 1000.0, 2000.0));
        payload.push_back(pwm & 0xFF);
        payload.push_back(pwm >> 8);
    }
    return payload;
}

double BetaflightInterface::hold_throttle(double climb) const // Throttle PWM for a climb rate in [-1, 1] of FULL_STICK_CLIMB, 0 holds the altitude
{
    const double command = hover_pwm_ + climb * (climb > 0.0 ? 2000.0 - hover_pwm_ : hover_pwm_ - 1000.0); // ALTHOLD centers it on ap_hover_throttle
    return MIN_CHECK + (command - 1000.0) * (2000.0 - MIN_CHECK) / 1000.0; // Undo the rescaling from [min_check, 2000] to [1000, 2000]
}

std::array<double, 3> BetaflightInterface::position() const // East, north of the arming point and altitude above it, in m
{
    double east = 0.0, north = 0.0, up = 0.0;
    home_.Forward(latitude_, longitude_, 0.0, east, north, up);
    return {east, north, altitude_};
}

std::array<double, 3> BetaflightInterface::offset() const // East, north, up from the current position to the target, in m
{
    const auto [east, north, up] = position();
    return {target_[0] - east, target_[1] - north, target_[2] - up};
}

void BetaflightInterface::hover_here()
{
    target_ = position();
    set_state(State::HOVER);
}

bool BetaflightInterface::arrived() const // At the target and nearly stopped (Betaflight's velocity tracking can overshoot)
{
    const auto [east, north, up] = offset();
    return std::hypot(east, north) < ARRIVAL_DISTANCE && std::abs(up) < ARRIVAL_DISTANCE
        && ground_speed_ < ARRIVAL_SPEED && std::abs(vertical_speed_) < ARRIVAL_SPEED;
}

bool BetaflightInterface::rc_override_valid() const // Recent, with four finite sticks and a CH6 position
{
    const auto &axes = rc_override_.axes;
    const auto &buttons = rc_override_.buttons;
    return (this->now() - rc_override_time_).seconds() < RC_OVERRIDE_TIMEOUT
        && axes.size() >= 4 && std::all_of(axes.begin(), axes.begin() + 4, [](float a) { return std::isfinite(a); })
        && buttons.size() >= 1 && buttons[0] >= 0 && buttons[0] <= 2;
}

rclcpp_action::GoalResponse BetaflightInterface::accept_takeoff()
{
    std::string reason;
    if (state_ != State::STARTED || takeoff_goal_ || land_goal_) {
        reason = std::string("state is ") + STATE_NAMES[static_cast<int>(state_)] + ", not STARTED";
    } else if (!gps_fix_) {
        reason = "no GPS fix";
    } else if (hover_pwm_ == 0) {
        reason = "ap_hover_throttle not read yet, or 0";
    } else if (arming_disable_flags_) {
        reason = "arming disable flags: " + hex(arming_disable_flags_);
    }
    if (!reason.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Takeoff rejected, %s", reason.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void BetaflightInterface::start_takeoff(const GoalHandle<Takeoff> &goal)
{
    takeoff_goal_ = goal;
    takeoff_altitude_ = goal->get_goal()->takeoff_altitude;
    set_state(State::ARMING);
    feedback(takeoff_goal_, "Arming");
}

rclcpp_action::GoalResponse BetaflightInterface::accept_land()
{
    if ((state_ != State::HOVER && state_ != State::REPOSITIONING) || takeoff_goal_ || land_goal_) {
        RCLCPP_ERROR(this->get_logger(), "Landing rejected, BetaflightInterface is not hovering or repositioning");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void BetaflightInterface::start_land(const GoalHandle<Land> &goal)
{
    land_goal_ = goal;
    target_ = {0.0, 0.0, goal->get_goal()->landing_altitude};
    set_state(State::REPOSITIONING);
    feedback(land_goal_, "Returning over the arming point at " + std::to_string(target_[2]) + " m");
}

rclcpp_action::GoalResponse BetaflightInterface::accept_offboard()
{
    if (state_ != State::HOVER || takeoff_goal_ || land_goal_ || offboard_goal_) {
        RCLCPP_ERROR(this->get_logger(), "Offboard rejected, BetaflightInterface is not hovering");
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

void BetaflightInterface::start_offboard(const GoalHandle<Offboard> &goal)
{
    offboard_goal_ = goal;
    offboard_controller_ = goal->get_goal()->controller_name;
    offboard_duration_ = goal->get_goal()->max_duration_sec;
    set_state(State::OFFBOARD);
    feedback(offboard_goal_, "Offboarding with controller: " + offboard_controller_);
}

void BetaflightInterface::reposition(const SetReposition::Request &request, SetReposition::Response &response)
{
    response.success = (state_ == State::HOVER || state_ == State::REPOSITIONING) && !takeoff_goal_ && !land_goal_;
    if (!response.success) {
        response.message = "Set reposition rejected, BetaflightInterface is not hovering or repositioning";
        RCLCPP_ERROR(this->get_logger(), "%s", response.message.c_str());
        return;
    }
    target_ = {request.east, request.north, request.altitude};
    set_state(State::REPOSITIONING);
    response.message = "set_reposition accepted";
    RCLCPP_INFO(this->get_logger(), "Repositioning to East-North %.2f %.2f Alt. %.2f", request.east, request.north, request.altitude);
}

void BetaflightInterface::set_state(State state)
{
    if (state != state_) {
        RCLCPP_INFO(this->get_logger(), "State %s -> %s", STATE_NAMES[static_cast<int>(state_)], STATE_NAMES[static_cast<int>(state)]);
    }
    state_ = state;
    state_since_ = this->now();
}

template <typename ActionT>
void BetaflightInterface::feedback(const GoalHandle<ActionT> &goal, const std::string &message)
{
    auto msg = std::make_shared<typename ActionT::Feedback>();
    msg->message = message;
    goal->publish_feedback(msg);
    RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
}

template <typename ActionT>
void BetaflightInterface::finish(GoalHandle<ActionT> &goal, bool success, const std::string &message) // Report the result and release the goal
{
    auto result = std::make_shared<typename ActionT::Result>();
    result->success = success;
    if (goal->is_canceling()) {
        goal->canceled(result);
    } else if (success) {
        goal->succeed(result);
    } else {
        goal->abort(result);
    }
    if (success) {
        RCLCPP_INFO(this->get_logger(), "%s", message.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "%s", message.c_str());
    }
    goal.reset();
}

std::string BetaflightInterface::hex(uint32_t value)
{
    char buf[11];
    std::snprintf(buf, sizeof(buf), "0x%08X", value);
    return buf;
}


int main(int argc, char **argv)
{
    signal(SIGPIPE, SIG_IGN); // A closed TCP link fails write() instead of killing the process
    rclcpp::init(argc, argv);
    int ret = 0;
    try {
        rclcpp::spin(std::make_shared<BetaflightInterface>());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(rclcpp::get_logger("betaflight_interface"), "%s", e.what());
        ret = 1; // Restarted by the tmux pane, e.g., while the SITL is not up yet
    }
    rclcpp::shutdown();
    return ret;
}
