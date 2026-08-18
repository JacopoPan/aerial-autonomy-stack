#include <chrono>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <cstdlib>
#include <string>
#include <cstring>
#include <sys/stat.h>
// ROS 2 Includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
// Gazebo Includes (libgz-transport13-*, libgz-msgs10-dev)
#include <gz/transport/Node.hh>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
// ZeroMQ Includes (requires cppzmq installed with libzmq3-dev)
#include <zmq.hpp> 

using namespace std::chrono_literals;

// Corresponds to Python struct.unpack('iI') (int32, uint32)
#pragma pack(push, 1)
struct ClockPayload {
    int32_t sec;
    uint32_t nanosec;
};
#pragma pack(pop)

class ZMQBridge : public rclcpp::Node {
public:
    ZMQBridge() : Node("zmq_bridge_node"), context_(1), socket_(context_, zmq::socket_type::rep) {

        this->declare_parameter("step_size", 250); // How many multiples of the timestep in the world SDF (250Hz/4ms for PX4, 500Hz/2ms for ArduPilot)
        step_size_ = this->get_parameter("step_size").as_int();
        this->declare_parameter("physics_dt", 0.004); // Size of the physics timestep in seconds (4ms for PX4, 2ms for ArduPilot)
        physics_dt_ = this->get_parameter("physics_dt").as_double();
        RCLCPP_INFO(this->get_logger(), "Config: step size = %d, physics dt = %.4f", step_size_, physics_dt_);
        this->declare_parameter("init_duration", 80.0); // Duration to run unpaused during reset (seconds)
        init_duration_ = this->get_parameter("init_duration").as_double();
        
        // // 1. Option A: ZMQ TCP Setup (see self.ZMQ_TRANSPORT in aas_env.py)
        // socket_.bind("tcp://*:5555"); // '*' binds to all interfaces (equivalent to 0.0.0.0)
        // RCLCPP_INFO(this->get_logger(), "ZMQ REP socket bound to port 5555");

        // 1. Option B: ZMQ IPC Setup (see self.ZMQ_TRANSPORT in aas_env.py)
        const char* env_instance = std::getenv("INSTANCE");
        if (env_instance == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Environment variable 'INSTANCE' was not set.");
        }
        std::string instance_id = env_instance;
        std::string ipc_address = "ipc:///tmp/aas_zmq_sockets/bridge_inst" + instance_id + ".ipc";
        try {
            socket_.bind(ipc_address);
            std::string socket_path = ipc_address.substr(6); // Set permissions to 777 (anyone can read/write) so that aas_env can connect
            chmod(socket_path.c_str(), 0777);
            RCLCPP_INFO(this->get_logger(), "ZMQ REP socket bound to: %s", ipc_address.c_str());
        } catch (zmq::error_t& e) {
            RCLCPP_ERROR(this->get_logger(), "ZMQ Bind Error: %s", e.what());
        }

        // 2. ROS 2 Setup
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/action", 10);
        
        subscription_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
            "/clock", 10, 
            std::bind(&ZMQBridge::clock_callback, this, std::placeholders::_1));

        const char* env_world = std::getenv("WORLD");
        if (env_world == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "Environment variable 'WORLD' was not set.");
        }
        std::string world_name = env_world;
        service_topic_ = "/world/" + world_name + "/control";

        // 3. Start ZMQ Thread
        running_ = true;
        zmq_thread_ = std::thread(&ZMQBridge::zmq_listener, this);
    }

    ~ZMQBridge() {
        running_ = false;
        clock_cv_.notify_all(); // Wake up the ZMQ thread if waiting for the clock
        if (zmq_thread_.joinable()) {
            zmq_thread_.join();
        }
        // Clean shutdown for ZMQ
        context_.close();
    }

private:
    zmq::context_t context_;
    zmq::socket_t socket_;
    
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr subscription_;
    
    gz::transport::Node gz_node_;
    std::string service_topic_;

    std::thread zmq_thread_;
    std::atomic<bool> running_;

    std::mutex clock_mutex_;
    std::condition_variable clock_cv_;
    bool clock_ready_ = false;
    ClockPayload current_clock_;
    double current_sim_time_ = 0.0;

    int step_size_;
    double physics_dt_;
    double init_duration_;

    void clock_callback(const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(clock_mutex_);
        
        current_clock_.sec = msg->clock.sec;
        current_clock_.nanosec = msg->clock.nanosec;
        current_sim_time_ = msg->clock.sec + (msg->clock.nanosec * 1e-9);
        
        clock_ready_ = true;
        clock_cv_.notify_all(); // Notify waiting threads
    }

    void set_gazebo_pause(bool pause) {
        gz::msgs::WorldControl req;
        req.set_pause(pause);
        gz::msgs::Boolean rep;
        bool result;
        gz_node_.Request(service_topic_, req, 1000, rep, result);
    }

    void step_gazebo() {
        gz::msgs::WorldControl req;
        req.set_pause(true);
        req.set_multi_step(static_cast<unsigned int>(step_size_));
        gz::msgs::Boolean rep;
        bool result;
        gz_node_.Request(service_topic_, req, 1000, rep, result);
    }

    void zmq_listener() {
        RCLCPP_INFO(this->get_logger(), "ZMQ listener thread started.");
        
        zmq::pollitem_t items[] = {
            { static_cast<void*>(socket_), 0, ZMQ_POLLIN, 0 }
        };

        while (rclcpp::ok() && running_) {
            zmq::poll(&items[0], 1, 100); // Poll with 100ms timeout to allow checking running_ flag

            if (items[0].revents & ZMQ_POLLIN) {
                try {
                    // 1. Receive Action
                    zmq::message_t request;
                    auto res = socket_.recv(request, zmq::recv_flags::none);
                    if (!res) continue;
                    double action = *static_cast<double*>(request.data()); // Get action (double)

                    bool received =  true; // Initialize and default to true

                    // 2. Logic Dispatch
                    if (std::abs(action - 9999.0) < 0.001) { // RESET MODE
                        RCLCPP_INFO(this->get_logger(), "Received reset action (9999.0). Running unpaused...");

                        // A. Unpause simulation
                        double reset_start;
                        {
                            std::lock_guard<std::mutex> lock(clock_mutex_);
                            reset_start = current_sim_time_;
                        }
                        set_gazebo_pause(false);
                        // B. Wait loop
                        while (rclcpp::ok() && running_) {
                            std::unique_lock<std::mutex> lock(clock_mutex_);
                            if (clock_cv_.wait_for(lock, 200ms, [this, reset_start]{
                                return current_sim_time_ >= (reset_start + init_duration_);
                            })) {
                                break;
                            }
                        }
                        // C. Pause simulation
                        set_gazebo_pause(true);
                        RCLCPP_INFO(this->get_logger(), "Initialization Complete. Paused.");

                    } else { // STEP MODE
                        // A. Publish Action
                        auto ros_msg = std_msgs::msg::Float64();
                        ros_msg.data = action;
                        publisher_->publish(ros_msg);
                        // B. Calculate Target Time
                        double target_time;
                        {
                            std::lock_guard<std::mutex> lock(clock_mutex_);
                            target_time = current_sim_time_ + (step_size_ * physics_dt_) - 0.0001;
                            clock_ready_ = false;
                        }
                        // C. Trigger Step
                        step_gazebo();
                        // D. Wait for Result
                        std::unique_lock<std::mutex> lock(clock_mutex_);
                        received = clock_cv_.wait_for(lock, 30000ms, [this, target_time]{
                            return current_sim_time_ >= target_time;
                        }); // Wait up to 30 seconds for clock_ready_ to become true
                    }

                    // 3. Send Reply
                    zmq::message_t reply(sizeof(ClockPayload));
                    ClockPayload snapshot;
                    {
                        std::lock_guard<std::mutex> lock(clock_mutex_);
                        snapshot = current_clock_;
                    }
                    if (!received) {
                        RCLCPP_WARN(this->get_logger(), "Update timeout!");
                        snapshot = {0, 0};
                    }
                    std::memcpy(reply.data(), &snapshot, sizeof(ClockPayload));
                    socket_.send(reply, zmq::send_flags::none);

                } catch (const std::exception &e) {
                    RCLCPP_ERROR(this->get_logger(), "ZMQ Error: %s", e.what());
                }
            }
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ZMQBridge>());
    rclcpp::shutdown();
    return 0;
}
