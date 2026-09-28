#include <rclcpp/rclcpp.hpp>

#include "mode.hpp"
#include <px4_ros2/components/node_with_mode.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<TemplateMode>>("px4_custom_mode_template", true));
    rclcpp::shutdown();
    return 0;
}
