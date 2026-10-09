/*
Minimal Betaflight interface: MSP Override of roll, pitch, throttle, yaw from sensor_msgs/Joy, attitude from MSP_ATTITUDE
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
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

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
            last_joy_ = std::chrono::steady_clock::now();
        });
        attitude_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("attitude", 10); // Roll, pitch, yaw in rad
        timer_ = this->create_wall_timer(20ms, [this]() { step(); }); // 50 Hz regardless of the command rate, as BetaflightFC's command thread
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
        if (joy_.axes.size() >= 4 && std::chrono::steady_clock::now() - last_joy_ < 500ms) { // Without fresh commands, Betaflight returns to the RC sticks after 300 ms
            std::vector<uint8_t> payload;
            for (size_t i = 0; i < 4; i++) { // Betaflight's AETR channel order: roll, pitch, throttle, yaw
                const auto pwm = static_cast<uint16_t>(1500 + 500 * std::clamp(joy_.axes[i], -1.0f, 1.0f));
                payload.push_back(pwm & 0xFF);
                payload.push_back(pwm >> 8);
            }
            send_msp(200, payload); // MSP_SET_RAW_RC
        }
        if (++tick_ % 5 == 0) {
            send_msp(108); // MSP_ATTITUDE, at 10 Hz
        }
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
            if (rx_[4] == 108 && rx_[3] >= 6) { // Roll and pitch in 0.1 deg, yaw in deg
                auto value = [this](int k) { return static_cast<int16_t>(rx_[5 + k] | rx_[6 + k] << 8); };
                geometry_msgs::msg::Vector3Stamped msg;
                msg.header.stamp = this->now();
                msg.vector.x = value(0) * M_PI / 1800.0;
                msg.vector.y = value(2) * M_PI / 1800.0;
                msg.vector.z = value(4) * M_PI / 180.0;
                attitude_pub_->publish(msg);
            }
            rx_.erase(rx_.begin(), rx_.begin() + 6 + rx_[3]);
        }
    }

    int fd_ = -1;
    unsigned tick_ = 0;
    std::vector<uint8_t> rx_;
    sensor_msgs::msg::Joy joy_;
    std::chrono::steady_clock::time_point last_joy_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr attitude_pub_;
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
