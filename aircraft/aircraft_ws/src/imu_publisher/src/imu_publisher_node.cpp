#include <rclcpp/rclcpp.hpp>
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include <sensor_msgs/msg/imu.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>

class ImuPublisherNode : public rclcpp::Node
{
public:
    ImuPublisherNode() : Node("imu_publisher_node")
    {
        this->declare_parameter<std::string>("autopilot", "px4");
        this->declare_parameter<int>("drone_id", 1);
        autopilot = this->get_parameter("autopilot").as_string();
        drone_id_ = this->get_parameter("drone_id").as_int();

        if (this->get_parameter("use_sim_time").as_bool()) {
            RCLCPP_INFO(this->get_logger(), "Simulation time is enabled.");
        } else {
            RCLCPP_INFO(this->get_logger(), "Simulation time is disabled.");
        }

        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            "/imu_data",
            rclcpp::SensorDataQoS()
        );

        if (autopilot == "px4") { // Subscribe to PX4's uXRCE-DDS agent topic (100Hz)
            px4_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
                "/Drone" + std::to_string(drone_id_) + "/fmu/out/sensor_combined",
                rclcpp::SensorDataQoS(),
                std::bind(&ImuPublisherNode::px4_callback, this, std::placeholders::_1));   
            RCLCPP_INFO(this->get_logger(), "imu_publisher_node started for PX4 (convert from FRD to FLU)");
        } else if (autopilot == "ardupilot") { // Subscribe to the MAVROS raw IMU topic (50Hz)
            ardupilot_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
                "/mavros/imu/data_raw",
                rclcpp::SensorDataQoS(),
                std::bind(&ImuPublisherNode::ardupilot_callback, this, std::placeholders::_1));   
            RCLCPP_INFO(this->get_logger(), "imu_publisher_node started for ArduPilot (passthrough)");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Unknown autopilot parameter: %s. Use 'px4' or 'ardupilot'.", autopilot.c_str());
        }
    }

private:
    int drone_id_;
    std::string autopilot;
    sensor_msgs::msg::Imu imu_msg_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr px4_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr ardupilot_sub_;

    void px4_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
    {     
        // Convert angular velocity and linear acceleration from PX4 FRD to ROS FLU
        imu_msg_.angular_velocity.x = msg->gyro_rad[0];
        imu_msg_.angular_velocity.y = -msg->gyro_rad[1];
        imu_msg_.angular_velocity.z = -msg->gyro_rad[2];
        imu_msg_.linear_acceleration.x = msg->accelerometer_m_s2[0];
        imu_msg_.linear_acceleration.y = -msg->accelerometer_m_s2[1];
        imu_msg_.linear_acceleration.z = -msg->accelerometer_m_s2[2];
        // Convert PX4 timestamp to ROS timestamp
        imu_msg_.header.stamp = rclcpp::Time(msg->timestamp * 1000ull);

        imu_msg_.header.frame_id = "imu_link";
        imu_pub_->publish(imu_msg_);
    }

    void ardupilot_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Just copy the MAVROS message
        imu_msg_ = *msg;

        imu_msg_.header.frame_id = "imu_link"; 
        imu_pub_->publish(imu_msg_);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor; // Or set num_threads with executor(rclcpp::ExecutorOptions(), 8);
    auto node = std::make_shared<ImuPublisherNode>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
