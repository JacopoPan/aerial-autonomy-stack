#ifndef GROUND_SYSTEM__GROUND_SYSTEM_HPP_
#define GROUND_SYSTEM__GROUND_SYSTEM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <thread>
#include <map>
#include <random>
#include <vector>
#include <atomic>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

#include "ground_system_msgs/msg/swarm_obs.hpp"
#include "ground_system_msgs/msg/drone_obs.hpp"

// MAVLink Headers, using the common set for GLOBAL_POSITION_INT
#include "mavlink/common/mavlink.h"

struct DroneData {
    double lat;
    double lon;
    double alt;
    double vx;
    double vy;
    double vz;
};

class GroundSystem : public rclcpp::Node
{
public:
    GroundSystem();
    ~GroundSystem();

private:
    // Parameters
    int num_drones_;
    std::string ip_;
    int base_port_;
    double publish_rate_;
    int target_id_;

    // Threading & Data
    std::mutex data_mutex_;
    std::map<int, DroneData> drone_obs_;
    std::vector<std::thread> listener_threads_;
    std::atomic<bool> keep_running_;

    // Random Number Generation
    std::default_random_engine rng_;

    // ROS Handles
    rclcpp::Publisher<ground_system_msgs::msg::SwarmObs>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Methods
    void mavlink_listener(int drone_id, int port);
    void publish_swarm_obs();
    double add_noise(double value, double std_dev);
};

#endif  // GROUND_SYSTEM__GROUND_SYSTEM_HPP
