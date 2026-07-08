#include "ground_system.hpp"

GroundSystem::GroundSystem() : Node("ground_system"), keep_running_(true)
{
    // Declare Parameters
    this->declare_parameter("num_drones", 1);
    this->declare_parameter("ip", "0.0.0.0");
    this->declare_parameter("base_port", 18540);
    this->declare_parameter("rate", 10.0);
    this->declare_parameter<std::vector<std::string>>("assignments", std::vector<std::string>{});
    this->declare_parameter("track_timeout", 20.0); // s: drop a track after this long with no update (0 = never)
    // Simulated radio link (active when use_sim_time and degrade_simulated_link are both true)
    this->declare_parameter("degrade_simulated_link", true); // Default to true
    this->declare_parameter("simulated_link_delay", 0.12); // s: mean one-way latency added to ros2 topic
    this->declare_parameter("simulated_link_jitter", 0.04); // s: +/- uniform jitter on the latency
    this->declare_parameter("simulated_link_loss", 0.02); // packet-loss probability [0,1]
    this->declare_parameter("simulated_link_rate", 10.0); // Hz: max per-drone position rate over a real radio (see SRx_POSITION)

    // Get Parameters
    num_drones_ = static_cast<int>(this->get_parameter("num_drones").as_int());
    ip_ = this->get_parameter("ip").as_string();
    base_port_ = static_cast<int>(this->get_parameter("base_port").as_int());
    publish_rate_ = this->get_parameter("rate").as_double();
    track_timeout_s_ = this->get_parameter("track_timeout").as_double();
    auto assignment_strings = this->get_parameter("assignments").as_string_array();
    for (const auto& pair_str : assignment_strings) {
        size_t delimiter_pos = pair_str.find('-');
        if (delimiter_pos != std::string::npos) {
            try {
                int predator = std::stoi(pair_str.substr(0, delimiter_pos));
                int prey = std::stoi(pair_str.substr(delimiter_pos + 1));
                assignments_[predator] = prey;
                RCLCPP_INFO(this->get_logger(), "Assigned %d to %d", predator, prey);
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Skipping invalid assignment format: '%s'", pair_str.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Missing hyphen in assignment: '%s'", pair_str.c_str());
        }
    }
    simulate_link_degradation_ = this->get_parameter("use_sim_time").as_bool() && this->get_parameter("degrade_simulated_link").as_bool();
    simulated_link_delay_s_ = this->get_parameter("simulated_link_delay").as_double();
    simulated_link_jitter_s_ = this->get_parameter("simulated_link_jitter").as_double();
    simulated_link_loss_prob_ = this->get_parameter("simulated_link_loss").as_double();
    simulated_link_rate_ = this->get_parameter("simulated_link_rate").as_double();
    if (simulate_link_degradation_) {
        RCLCPP_WARN(this->get_logger(), "Simulated radio link (%.0fHz) ON: delay=%.0fms jitter=%.0fms loss=%.0f%%", simulated_link_rate_, simulated_link_delay_s_ * 1e3, simulated_link_jitter_s_ * 1e3, simulated_link_loss_prob_ * 1e2);
    }

    // Random Seed
    rng_.seed(std::random_device()());

    // Publisher
    publisher_ = this->create_publisher<ground_system_msgs::msg::SwarmObs>("/tracks", 10);

    // Timer
    timer_ = rclcpp::create_timer(this, this->get_clock(), std::chrono::duration<double>(1.0 / publish_rate_), std::bind(&GroundSystem::publish_swarm_obs, this));

    // Single listener thread, use base_port_ and pass drone_id = -1 to signal "auto-detect ID from message"
    listener_threads_.emplace_back(&GroundSystem::mavlink_listener, this, -1, base_port_, 0);
    RCLCPP_INFO(this->get_logger(), "Listening to the streams from %d drones on single port %d", num_drones_, base_port_);
    // To listen to separate UDP streams on separate ports, create multiple threads using:
    // listener_threads_.emplace_back(&GroundSystem::mavlink_listener, this, drone_id, port, thread_idx);
    // Make sure thread_idx < MAVLINK_COMM_NUM_BUFFERS
}

GroundSystem::~GroundSystem()
{
    keep_running_ = false;
    for (auto &t : listener_threads_) {
        if (t.joinable()) {
            t.join();
        }
    }
}

void GroundSystem::mavlink_listener(int drone_id, int port, int thread_idx)
{
    if (thread_idx < 0 || thread_idx >= MAVLINK_COMM_NUM_BUFFERS) {
        RCLCPP_ERROR(this->get_logger(), "Invalid thread_idx %d. Must be strictly less than %d", thread_idx, MAVLINK_COMM_NUM_BUFFERS);
        return;
    }

    std::mt19937 simulated_link_rng(std::random_device{}());
    std::uniform_real_distribution<double> simulated_link_unif(0.0, 1.0);

    // Setup UDP Socket
    int sockfd = -1;
    struct sockaddr_in servaddr;

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Socket creation failed for drone %d", drone_id);
        return;
    }

    // Set timeout for recvfrom so the thread can exit cleanly
    struct timeval read_timeout;
    read_timeout.tv_sec = 1;
    read_timeout.tv_usec = 0;
    setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &read_timeout, sizeof(read_timeout));

    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip_.c_str(), &servaddr.sin_addr) <= 0) {
        RCLCPP_ERROR(this->get_logger(), "Invalid IP address: %s", ip_.c_str());
        close(sockfd);
        return;
    }

    if (bind(sockfd, (const struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "Bind failed for drone %d on port %d", drone_id, port);
        close(sockfd);
        return;
    }

    uint8_t buffer[2048];
    mavlink_message_t msg;
    mavlink_status_t status;
    mavlink_channel_t channel = static_cast<mavlink_channel_t>(MAVLINK_COMM_0 + thread_idx);

    while (keep_running_ && rclcpp::ok()) {
        ssize_t len = recvfrom(sockfd, (char *)buffer, 2048, 0, NULL, NULL);
        // Prevention of CPU hogging is handled by recvfrom blocking/timeout
        if (len > 0) {
            // Parse bytes
            for (ssize_t i = 0; i < len; ++i) {
                if (mavlink_parse_char(channel, buffer[i], &msg, &status)) {

                    // In single-port/single-thread mode (drone_id == -1), detect ID from the message
                    int current_id = drone_id;
                    if (current_id == -1) {
                        current_id = msg.sysid;
                        if (current_id < 1 || current_id > num_drones_) {
                            continue; // Ignore out-of-bounds IDs
                        }
                    }
                    
                    if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) { // Handle GLOBAL_POSITION_INT message
                        mavlink_global_position_int_t pos;
                        mavlink_msg_global_position_int_decode(&msg, &pos);
                        DroneData obs;
                        obs.lat = pos.lat / 1e7;
                        obs.lon = pos.lon / 1e7;
                        obs.alt = pos.alt / 1000.0; // mm to m
                        obs.vx = pos.vx / 100.0;    // cm/s to m/s
                        obs.vy = pos.vy / 100.0;
                        obs.vz = pos.vz / 100.0;
                        {
                            std::lock_guard<std::mutex> lock(data_mutex_);
                            if (simulate_link_degradation_) {
                                // Simulated message rate: drop messages that arrive faster
                                if (simulated_link_rate_ > 0.0) {
                                    const rclcpp::Time t = this->now();
                                    auto it = last_sim_msg_.find(current_id);
                                    if (it != last_sim_msg_.end() && (t - it->second).seconds() < 1.0 / simulated_link_rate_) continue;
                                    last_sim_msg_[current_id] = t;
                                }
                                // Simulated message loss: skip observation
                                if (simulated_link_loss_prob_ > 0.0 && simulated_link_unif(simulated_link_rng) < simulated_link_loss_prob_) continue;
                                // Simulate latency and jitter: do not deliver now, schedule visibility for later
                                const double jitter = simulated_link_jitter_s_ > 0.0 ? (simulated_link_unif(simulated_link_rng) * 2.0 - 1.0) * simulated_link_jitter_s_ : 0.0;
                                const rclcpp::Time release = this->now() + rclcpp::Duration::from_seconds(std::max(0.0, simulated_link_delay_s_ + jitter));
                                delayed_sim_obs_buf_[current_id].push_back({release, obs});
                            } else { // Real world deployment: deliver immediately
                                drone_obs_[current_id] = obs;
                                last_seen_[current_id] = this->now();
                            }
                        }
                    }
                }
            }
        } else if (len < 0) { // Handle timeout or error
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                continue; // Just a timeout, continue loop to check keep_running_
            } else {
                char errbuf[256];
                const char *msg = strerror_r(errno, errbuf, sizeof(errbuf));
                RCLCPP_WARN(this->get_logger(), "Recv failed for drone %d: %s", drone_id, msg);
            }
        }
    }

    close(sockfd);
}

void GroundSystem::publish_swarm_obs()
{
    // TODO: add realistic noise model
    const double POS_STD_DEV_DEG = 0.0; // 1e-5;
    const double ALT_STD_DEV_M = 0.0; // 0.5;
    const double VEL_STD_DEV_MS = 0.0; // 0.1;

    ground_system_msgs::msg::SwarmObs swarm_msg;
    swarm_msg.header.stamp = this->now();

    // Copy data to minimize lock duration
    const rclcpp::Time now = this->now();
    std::map<int, DroneData> current_obs;
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (simulate_link_degradation_) { // In simulation only, release buffered observations whose link latency has elapsed
            for (auto &kv : delayed_sim_obs_buf_) {
                auto &q = kv.second;
                while (!q.empty() && q.front().release <= now) {
                    drone_obs_[kv.first] = q.front().data;
                    last_seen_[kv.first] = now;
                    q.pop_front();
                }
            }
        }
        // Drop stale tracks so a dead link disappears from /tracks instead of being republished forever as a frozen ghost
        if (track_timeout_s_ > 0.0) {
            for (auto it = drone_obs_.begin(); it != drone_obs_.end(); ) {
                auto seen = last_seen_.find(it->first);
                if (seen == last_seen_.end() || (now - seen->second).seconds() > track_timeout_s_) {
                    last_seen_.erase(it->first);
                    it = drone_obs_.erase(it);
                } else {
                    ++it;
                }
            }
        }
        current_obs = drone_obs_;
    }

    for (const auto &pair : current_obs) {
        int id = pair.first;
        DroneData track = pair.second;

        ground_system_msgs::msg::DroneObs drone_msg;
        drone_msg.id = id;
        if (assignments_.find(id) != assignments_.end()) {
            drone_msg.label = assignments_[id]; // Label with the target ID
        } else {
            drone_msg.label = 0; // If no assignment exists, default to 0 (unused ID)
        }

        // Add noise
        drone_msg.latitude_deg = add_noise(track.lat, POS_STD_DEV_DEG);
        drone_msg.longitude_deg = add_noise(track.lon, POS_STD_DEV_DEG);
        drone_msg.altitude_m = static_cast<float>(add_noise(track.alt, ALT_STD_DEV_M));
        drone_msg.velocity_n_m_s = static_cast<float>(add_noise(track.vx, VEL_STD_DEV_MS));
        drone_msg.velocity_e_m_s = static_cast<float>(add_noise(track.vy, VEL_STD_DEV_MS));
        drone_msg.velocity_d_m_s = static_cast<float>(add_noise(track.vz, VEL_STD_DEV_MS));

        swarm_msg.tracks.push_back(drone_msg);
    }

    if (!swarm_msg.tracks.empty()) {
        publisher_->publish(swarm_msg);
    }
}

double GroundSystem::add_noise(double value, double std_dev)
{
    if (std_dev <= 0.0) {
        return value; // Do not change the state of the random number generator rng_ if no noise is requested
    }
    std::normal_distribution<double> dist(0.0, std_dev);
    return value + dist(rng_);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GroundSystem>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
