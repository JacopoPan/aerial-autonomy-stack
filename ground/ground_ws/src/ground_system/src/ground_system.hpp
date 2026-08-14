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
#include <deque>
#include <algorithm>

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

struct DelayedSimObs {
    rclcpp::Time release;
    DroneData data;
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
    std::map<int, int> assignments_;

    // Threading & Data
    std::mutex data_mutex_;
    std::map<int, DroneData> drone_obs_;
    std::vector<std::thread> listener_threads_;
    std::atomic<bool> keep_running_;
    std::map<int, rclcpp::Time> last_seen_;
    double track_timeout_s_;

    // Random Number Generation
    std::default_random_engine rng_;

    // Simulated radio-link model active when use_sim_time and degrade_simulated_link (default: true) are both true
    bool simulate_link_degradation_;
    double simulated_link_delay_s_;
    double simulated_link_jitter_s_;
    double simulated_link_loss_prob_;
    double simulated_link_rate_;
    double simulated_link_outage_rate_;
    double simulated_link_outage_duration_s_;
    std::map<int, rclcpp::Time> sim_next_outage_;
    std::map<int, rclcpp::Time> sim_outage_until_;
    std::map<int, rclcpp::Time> last_sim_msg_;
    std::map<int, std::deque<DelayedSimObs>> delayed_sim_obs_buf_;

    // ROS Handles
    rclcpp::Publisher<ground_system_msgs::msg::SwarmObs>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Methods
    void mavlink_listener(int drone_id, int port, int thread_idx);
    void publish_swarm_obs();
    double add_noise(double value, double std_dev);
};

#endif  // GROUND_SYSTEM__GROUND_SYSTEM_HPP
