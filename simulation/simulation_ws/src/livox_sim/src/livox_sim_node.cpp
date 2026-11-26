#include <rclcpp/rclcpp.hpp>

#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/transport/Node.hh>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#pragma pack(push, 1)
struct LivoxPoint // Point of livox_ros_driver2's PointCloud2 (LivoxPointXyzrtlt, 26 bytes, no padding)
{
    float x;
    float y;
    float z;
    float intensity; // Reflectivity, 0 to 255
    uint8_t tag;
    uint8_t line;
    double timestamp; // Absolute time of the point, in ns
};
#pragma pack(pop)
static_assert(sizeof(LivoxPoint) == 26, "LivoxPoint must match livox_ros_driver2's point_step");

template<typename T>
T read_field(const char * point, uint32_t offset) // Unaligned read of one field of a packed point
{
    T value{};
    std::memcpy(&value, point + offset, sizeof(T));
    return value;
}

// Publish Gazebo's LiDAR as the Mid-360's livox_ros_driver2 does: /livox/lidar, sensor_msgs/PointCloud2 (xfer_format 0) or livox_ros_driver2/CustomMsg (1)
// Gazebo input (gpu_lidar fields x, y, z, intensity, ring): https://github.com/gazebosim/gz-sensors/blob/6d74c1fe96d7277c7dec96a54bbc79d5c752e63c/src/GpuLidarSensor.cc#L138-L141
// Output formats: https://github.com/Livox-SDK/livox_ros_driver2#32-livox-ros-driver-2-internal-main-parameter-configuration-instructions
// Livox PointCloud2 offsets (Lddc::InitPointcloud2MsgHeader): https://github.com/Livox-SDK/livox_ros_driver2/blob/21445540f0d100dc86a7e6df312dd70bbdb4afdf/src/lddc.cpp#L262
// A frame is built from sim_time_slices renders of Gazebo's gpu_lidar (which renders at sim_time_slices * update_rate):
// - sim_time_slices = 1, Gazebo's idealized LiDAR: each render is a frame, and all its points have the same time (thus, nothing to deskew)
// - sim_time_slices > 1, based on https://github.com/mathaimp/livox_gz_sim: a frame takes one band of rows from each of sim_time_slices + 1 consecutive renders (top band first),
//   with the points stamped at their render's time; the render closing a frame (bottom band) also opens the next (top band), so a frame spans one period,
//   like the real Mid-360's, whose points are spread over the frame (and motion during the frame skews it)
class LivoxSimNode : public rclcpp::Node
{
public:
    LivoxSimNode() : Node("livox_sim_node")
    {
        this->declare_parameter<std::string>("gz_topic", ""); // gpu_lidar's gz.msgs.PointCloudPacked topic
        this->declare_parameter<int>("xfer_format", 0); // As in livox_ros_driver2: 0 for PointCloud2, 1 for CustomMsg
        this->declare_parameter<int>("sim_time_slices", 1); // Renders per frame: 1 for Gazebo's idealized LiDAR, more to spread the points' times over the frame

        const std::string gz_topic = this->get_parameter("gz_topic").as_string();
        xfer_format_ = static_cast<int>(this->get_parameter("xfer_format").as_int());
        const auto sim_time_slices = static_cast<uint32_t>(std::max<int64_t>(this->get_parameter("sim_time_slices").as_int(), 1));

        bands_ = sim_time_slices == 1 ? 1 : sim_time_slices + 1; // One band (all rows), or one per render, the boundary render giving a frame's bottom band and the next's top

        if (xfer_format_ == 1) {
            custom_pub_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("/livox/lidar", 10);
        } else {
            points_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/lidar", 10);
        }
        if (!gz_node_.Subscribe(gz_topic, &LivoxSimNode::points_callback, this)) {
            RCLCPP_ERROR(this->get_logger(), "Cannot subscribe to Gazebo topic %s", gz_topic.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "livox_sim_node started (%s to /livox/lidar, xfer_format %d, %u time slices)", gz_topic.c_str(), xfer_format_, sim_time_slices);
    }

private:
    static constexpr uint32_t kLines = 4; // The Mid-360 has 4 laser lines

    int xfer_format_ = 0;
    uint32_t bands_ = 1; // Bands of rows per frame: 1 if sim_time_slices == 1; sim_time_slices + 1 otherwise
    uint32_t band_ = 0; // Band the next render adds to the frame, 0 is the top one
    std::vector<LivoxPoint> points_; // The frame being built
    builtin_interfaces::msg::Time frame_stamp_; // Time of the frame's first band
    uint64_t frame_stamp_ns_ = 0;
    uint64_t last_stamp_ns_ = 0;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_pub_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_pub_;
    gz::transport::Node gz_node_; // Last member, destroyed first: no callback runs on the members above while they are destroyed

    void points_callback(const gz::msgs::PointCloudPacked & msg) // Runs in a gz-transport thread, one render at a time
    {
        std::unordered_map<std::string, uint32_t> offsets; // gpu_lidar's fields: x, y, z, intensity (float32), ring (uint16)
        for (const auto & field : msg.field()) {
            offsets[field.name()] = field.offset();
        }
        const uint32_t x_offset = offsets.at("x");
        const uint32_t y_offset = offsets.at("y");
        const uint32_t z_offset = offsets.at("z");
        const uint32_t intensity_offset = offsets.at("intensity");
        const uint32_t ring_offset = offsets.at("ring");

        builtin_interfaces::msg::Time stamp; // Simulation time of the render
        stamp.sec = static_cast<int32_t>(msg.header().stamp().sec());
        stamp.nanosec = static_cast<uint32_t>(msg.header().stamp().nsec());
        const uint64_t stamp_ns = static_cast<uint64_t>(rclcpp::Time(stamp).nanoseconds());

        const uint32_t rows = std::max<uint32_t>(msg.height(), 1U); // The cloud is organized: one row per vertical sample, in field "ring"
        const auto add_band = [&](uint32_t band) { // Add this render's rows of a band to the frame, band 0 has the highest elevations
            for (uint32_t row = msg.height() * (bands_ - 1 - band) / bands_; row < msg.height() * (bands_ - band) / bands_; ++row) {
                for (uint32_t col = 0; col < msg.width(); ++col) {
                    const char * p = msg.data().data() + static_cast<size_t>(row) * msg.row_step() + static_cast<size_t>(col) * msg.point_step();
                    LivoxPoint point{};
                    point.x = read_field<float>(p, x_offset);
                    point.y = read_field<float>(p, y_offset);
                    point.z = read_field<float>(p, z_offset);
                    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
                        continue; // Rays without a return (+/- inf, see REP 117)
                    }
                    const float intensity = read_field<float>(p, intensity_offset);
                    point.intensity = std::isfinite(intensity) ? std::clamp(intensity, 0.0F, 255.0F) : 0.0F;
                    point.tag = 0; // Single return, no noise flags
                    point.line = static_cast<uint8_t>(std::min<uint32_t>(read_field<uint16_t>(p, ring_offset), rows - 1) * kLines / rows); // Rows grouped in 4 bands
                    point.timestamp = static_cast<double>(stamp_ns); // Time of the render that measured it
                    points_.push_back(point);
                }
            }
        };

        if (stamp_ns < last_stamp_ns_) {
            band_ = 0; // Simulation reset: drop the partial frame
        }
        last_stamp_ns_ = stamp_ns;
        if (band_ == 0) {
            start_frame(stamp, stamp_ns); // The frame's time is its first render's
        }
        add_band(band_); // With one band, the whole render
        if (++band_ < bands_) {
            return; // The frame needs the next renders
        }
        publish_frame();
        band_ = 0;
        if (bands_ > 1) { // This render's top band opens the next frame, at the time the last one closed
            start_frame(stamp, stamp_ns);
            add_band(0);
            band_ = 1;
        }
    }

    void start_frame(const builtin_interfaces::msg::Time & stamp, uint64_t stamp_ns)
    {
        points_.clear();
        frame_stamp_ = stamp;
        frame_stamp_ns_ = stamp_ns;
    }

    void publish_frame()
    {
        if (xfer_format_ == 1) {
            livox_ros_driver2::msg::CustomMsg livox_msg;
            livox_msg.header.stamp = frame_stamp_;
            livox_msg.header.frame_id = "livox_frame"; // Default frame_id of livox_ros_driver2's launch files
            livox_msg.timebase = frame_stamp_ns_; // Time of the first point
            livox_msg.point_num = static_cast<uint32_t>(points_.size());
            livox_msg.points.reserve(points_.size());
            for (const auto & point : points_) {
                livox_ros_driver2::msg::CustomPoint custom_point;
                custom_point.offset_time = static_cast<uint32_t>(point.timestamp - static_cast<double>(frame_stamp_ns_)); // timestamp - timebase
                custom_point.x = point.x;
                custom_point.y = point.y;
                custom_point.z = point.z;
                custom_point.reflectivity = static_cast<uint8_t>(point.intensity);
                custom_point.tag = point.tag;
                custom_point.line = point.line;
                livox_msg.points.push_back(custom_point);
            }
            custom_pub_->publish(livox_msg);
        } else {
            sensor_msgs::msg::PointCloud2 livox_msg;
            livox_msg.header.stamp = frame_stamp_; // Time of the first point
            livox_msg.header.frame_id = "livox_frame"; // Default frame_id of livox_ros_driver2's launch files
            sensor_msgs::PointCloud2Modifier modifier(livox_msg);
            modifier.setPointCloud2Fields(7, // Same fields and offsets (0, 4, 8, 12, 16, 17, 18) as livox_ros_driver2
                "x", 1, sensor_msgs::msg::PointField::FLOAT32,
                "y", 1, sensor_msgs::msg::PointField::FLOAT32,
                "z", 1, sensor_msgs::msg::PointField::FLOAT32,
                "intensity", 1, sensor_msgs::msg::PointField::FLOAT32,
                "tag", 1, sensor_msgs::msg::PointField::UINT8,
                "line", 1, sensor_msgs::msg::PointField::UINT8,
                "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64);
            modifier.resize(points_.size()); // height 1, width = number of points
            livox_msg.is_dense = true; // Rays without a return were dropped
            if (!points_.empty()) {
                std::memcpy(livox_msg.data.data(), points_.data(), points_.size() * sizeof(LivoxPoint));
            }
            points_pub_->publish(livox_msg);
        }
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LivoxSimNode>());
    rclcpp::shutdown();
    return 0;
}
