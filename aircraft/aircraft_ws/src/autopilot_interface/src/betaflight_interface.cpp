/*
Minimal Betaflight interface: MSP Override of roll, pitch, throttle, yaw from sensor_msgs/Joy, autopilot state from MSP replies
The pilot (or betaflight_rc.py) arms and flips the MSP OVERRIDE switch (CH7, see betaflight-2026.6.cli), this node never touches ARM or modes

Betaflight FCU interface:
- tcp://host:port for the SITL (UARTn listens on 5760 + n + port offset)
- serial port on hardware (e.g., /dev/ttyACM0)

Using betaflight_rc.py, arm and set CH7 high for MSP OVERRIDE:
    rc> arm
    rc> ch 7 2000

Then, use with:
    ros2 topic pub -r 20 /Drone${DRONE_ID}/rc_override sensor_msgs/msg/Joy "{axes: [0.0, 0.0, 0.2, 0.1]}" # roll, pitch, throttle, yaw sticks in [-1, 1]

Joy axes are stick positions in [-1, 1], the active Betaflight flight mode decides what they mean
In ANGLE mode (always on, see betaflight-2026.6.cli):
    roll, pitch = tilt angle (+-60 deg at full stick), yaw = turn rate, throttle = thrust (-1 idle, ~0.19 hovers the simulated Iris)

MSP budget: Betaflight answers one MSP request per port per serial task run, serial_update_rate_hz times per second
    (default 100Hz, set to 500Hz in betaflight-2026.6.cli), this node sends 100/s (50 MSP_SET_RAW_RC, 50 MSP_MULTIPLE_MSP)

Topics published at 50 Hz, from one MSP_MULTIPLE_MSP request (four replies) every timer tick:
    attitude (geometry_msgs/Vector3Stamped): roll, pitch in rad (FLU: right wing down, nose down positive), heading in rad (clockwise from north)
    imu (sensor_msgs/Imu): FLU angular velocity (rad/s) and acceleration (m/s^2), from MSP_RAW_IMU raw sensor units
    global_position (sensor_msgs/NavSatFix): latitude, longitude, MSL altitude (1 m steps)
    velocity (geometry_msgs/TwistStamped): ENU, from the GPS ground speed and course, and the estimated vertical speed
*/
#include <fcntl.h>
#include <netdb.h>
#include <signal.h>
#include <sys/socket.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using namespace std::chrono_literals;

class BetaflightInterface : public rclcpp::Node
{
public:
    BetaflightInterface() : Node("betaflight_interface")
    {
        const std::string device = this->declare_parameter<std::string>("device", "tcp://127.0.0.1:5762");
        fd_ = open_device(device);
        RCLCPP_INFO(this->get_logger(), "MSP on %s", device.c_str());
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("rc_override", 10, [this](const sensor_msgs::msg::Joy &msg) {
            joy_ = msg;
            last_joy_ = this->now();
        });
        acc_1g_ = this->declare_parameter<int>("acc_1g", 2048); // MSP_RAW_IMU accelerometer units per g: 2048 on most FC IMUs (16 g range), 256 in the SITL
        attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude", 10); // Roll, pitch, yaw in rad
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu", 10);
        global_position_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("global_position", 10);
        velocity_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("velocity", 10);
        timer_ = rclcpp::create_timer(this, this->get_clock(), 20ms, [this]() { step(); }); // 50 Hz on the node clock (sim time in simulation, like Betaflight's scheduler) regardless of the command rate, as BetaflightFC's command thread
    }
    ~BetaflightInterface() { close(fd_); }

private:
    static int open_device(const std::string &device)
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

    void send_msp(uint8_t cmd, const std::vector<uint8_t> &payload = {}) // MSP v1 request: $M< size cmd payload checksum
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

    void step()
    {
        if (joy_.axes.size() >= 4 && (this->now() - last_joy_).seconds() < 0.5) { // Without fresh commands, Betaflight returns to the RC sticks after 300 ms
            std::vector<uint8_t> payload;
            for (size_t i = 0; i < 4; i++) { // Betaflight's AETR channel order: roll, pitch, throttle, yaw
                const auto pwm = static_cast<uint16_t>(1500 + 500 * std::clamp(joy_.axes[i], -1.0f, 1.0f));
                payload.push_back(pwm & 0xFF);
                payload.push_back(pwm >> 8);
            }
            send_msp(200, payload); // MSP_SET_RAW_RC
        }
        send_msp(230, state_cmds_); // MSP_MULTIPLE_MSP, every tick, i.e. 50/s, with MSP_SET_RAW_RC's 50/s -> 100 requests/s < serial_update_rate_hz in betaflight-2026.6.cli
        uint8_t buf[256];
        ssize_t n;
        while ((n = read(fd_, buf, sizeof(buf))) > 0) {
            rx_.insert(rx_.end(), buf, buf + n);
        }
        if (n == 0) {
            throw std::runtime_error("MSP link closed");
        }
        const std::string header = "$M>"; // MSP v1 reply: $M> size cmd payload checksum
        while (true) {
            auto start = std::search(rx_.begin(), rx_.end(), header.begin(), header.end());
            rx_.erase(rx_.begin(), start == rx_.end() ? rx_.end() - std::min<size_t>(rx_.size(), 2) : start); // Keep a split header
            if (rx_.size() < 6 || rx_.size() < 6u + rx_[3]) {
                break;
            }
            handle(rx_[4], &rx_[5], rx_[3]);
            rx_.erase(rx_.begin(), rx_.begin() + 6 + rx_[3]);
        }
    }

    void handle(uint8_t cmd, const uint8_t *d, size_t size) // Publish the payload of an MSP reply
    {
        auto u16 = [d](size_t k) { return static_cast<uint16_t>(d[k] | d[k + 1] << 8); };
        auto s16 = [&u16](size_t k) { return static_cast<int16_t>(u16(k)); };
        auto s32 = [&u16](size_t k) { return static_cast<int32_t>(u16(k) | static_cast<uint32_t>(u16(k + 2)) << 16); };
        if (cmd == 230) { // MSP_MULTIPLE_MSP: the replies to state_cmds_, each as size and payload
            for (size_t i = 0, k = 0; k < state_cmds_.size() && i < size && i + 1 + d[i] <= size; i += 1 + d[i], k++) {
                handle(state_cmds_[k], d + i + 1, d[i]);
            }
        } else if (cmd == 102 && size >= 12) { // MSP_RAW_IMU: acceleration in acc_1g units per g, angular velocity in 16.384 units per deg/s (2000 deg/s range), FLU
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
        } else if (cmd == 108 && size >= 6) { // MSP_ATTITUDE: roll and pitch in 0.1 deg, yaw (heading) in deg
            geometry_msgs::msg::Vector3Stamped msg;
            msg.header.stamp = this->now();
            msg.vector.x = s16(0) * M_PI / 1800.0;
            msg.vector.y = s16(2) * M_PI / 1800.0;
            msg.vector.z = s16(4) * M_PI / 180.0;
            attitude_pub_->publish(msg);
        } else if (cmd == 109 && size >= 6) { // MSP_ALTITUDE: altitude in cm, vertical speed in cm/s (for the velocity)
            vertical_speed_ = s16(4) / 100.0; // TODO: overestimated at higher RTFs
        } else if (cmd == 106 && size >= 16) { // MSP_RAW_GPS: fix, satellites, lat, lon in 1e-7 deg, MSL altitude in m, speed in cm/s, course in 0.1 deg
            sensor_msgs::msg::NavSatFix fix;
            fix.header.stamp = this->now();
            fix.status.status = d[0] ? sensor_msgs::msg::NavSatStatus::STATUS_FIX : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
            fix.latitude = s32(2) / 1e7;
            fix.longitude = s32(6) / 1e7;
            fix.altitude = u16(10);
            global_position_pub_->publish(fix);
            geometry_msgs::msg::TwistStamped vel; // ENU, as MAVROS's local_position/velocity_local
            vel.header.stamp = fix.header.stamp;
            const double speed = u16(12) / 100.0, course = u16(14) * M_PI / 1800.0;
            vel.twist.linear.x = speed * std::sin(course); // East
            vel.twist.linear.y = speed * std::cos(course); // North
            vel.twist.linear.z = vertical_speed_; // Up
            velocity_pub_->publish(vel);
        }
    }

    const std::vector<uint8_t> state_cmds_ = {102, 108, 109, 106}; // MSP_RAW_IMU, MSP_ATTITUDE, MSP_ALTITUDE, MSP_RAW_GPS: handled in this order, so velocity (published on MSP_RAW_GPS) uses this request's vertical speed (from MSP_ALTITUDE)
    int fd_ = -1;
    std::vector<uint8_t> rx_;
    int acc_1g_ = 2048;
    double vertical_speed_ = 0.0; // m/s, up
    sensor_msgs::msg::Joy joy_;
    rclcpp::Time last_joy_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr global_position_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

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
