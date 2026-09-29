#pragma once

#include <Eigen/Core>
#include <stdexcept>
#include <string>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_ros2/control/setpoint_types/experimental/rates.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/control/setpoint_types/multicopter/goto.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <rclcpp/rclcpp.hpp>

static const std::string kName = "AAS Template"; // The name shown in QGroundControl's flight mode selector

class TemplateMode : public px4_ros2::ModeBase
{
public:
    explicit TemplateMode(rclcpp::Node& node) : ModeBase(node, Settings{kName}.preventArming(true))
    {
        local_position_ = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

        const std::string setpoint_type = node.declare_parameter<std::string>("setpoint_type", "trajectory"); // Options: trajectory (default), goto, attitude, rates
        if (setpoint_type == "trajectory") {
            trajectory_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
        } else if (setpoint_type == "goto") {
            goto_ = std::make_shared<px4_ros2::MulticopterGotoSetpointType>(*this);
        } else if (setpoint_type == "attitude") {
            attitude_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);
        } else if (setpoint_type == "rates") {
            rates_ = std::make_shared<px4_ros2::RatesSetpointType>(*this);
        } else {
            throw std::runtime_error("Unknown setpoint_type '" + setpoint_type + "'");
        }
    }

    void onActivate() override // Called by PX4 when this mode is selected in QGroundControl
    {
        target_ned_m_ = local_position_->positionNed() + Eigen::Vector3f{TEMPLATE_NORTH_M, 0.0f, 0.0f};
        heading_rad_ = local_position_->heading();
    }

    void onDeactivate() override {} // Called when PX4 leaves this mode, release here whatever onActivate acquired

    void updateSetpoint(float dt_s) override // Polled on a timer at the desiredUpdateRateHz() of the setpoint type constructed above (trajectory 50Hz, goto 30Hz, attitude 100Hz, rates 200Hz)
    {
        if (trajectory_) {     // Position reference, PX4 runs its position, attitude, and rate loops
            trajectory_->updatePosition(target_ned_m_);
            if ((target_ned_m_ - local_position_->positionNed()).norm() < TEMPLATE_ARRIVED_M) {
                completed(px4_ros2::Result::Success);
            }
        } else if (goto_) {    // Position reference with PX4-side trajectory smoothing and a heading
            goto_->update(target_ned_m_, heading_rad_);
            if ((target_ned_m_ - local_position_->positionNed()).norm() < TEMPLATE_ARRIVED_M) {
                completed(px4_ros2::Result::Success);
            }
        } else if (attitude_) { // Attitude and collective thrust, PX4 runs the rate loop only
            attitude_->update(px4_ros2::eulerRpyToQuaternion(0.0f, TEMPLATE_PITCH_RAD, heading_rad_),
                              Eigen::Vector3f{0.0f, 0.0f, TEMPLATE_HOVER_THRUST});
        } else if (rates_) {   // Body rates and collective thrust, PX4 runs no outer loop
            rates_->update(Eigen::Vector3f{0.0f, 0.0f, TEMPLATE_YAW_RATE_RADS},
                           Eigen::Vector3f{0.0f, 0.0f, TEMPLATE_HOVER_THRUST});
        }
    }

private:
    static constexpr float TEMPLATE_NORTH_M = 20.0f;       // Distance (m) north of the activation point, for trajectory and goto
    static constexpr float TEMPLATE_ARRIVED_M = 0.5f;      // Distance (m) from the target to consider the mode complete, for trajectory and goto
    static constexpr float TEMPLATE_PITCH_RAD = -0.087f;   // Pitch (rad, -5deg) to move forward, for attitude
    static constexpr float TEMPLATE_YAW_RATE_RADS = 1.0f;  // Yaw rate (rad/s) to spin on itself, for rates
    static constexpr float TEMPLATE_HOVER_THRUST = -0.72f; // Collective thrust (normalized, body Z down), for attitude and rates (drops some altitude)

    Eigen::Vector3f target_ned_m_;
    float heading_rad_{0.0f};
    std::shared_ptr<px4_ros2::OdometryLocalPosition> local_position_;
    std::shared_ptr<px4_ros2::TrajectorySetpointType> trajectory_;
    std::shared_ptr<px4_ros2::MulticopterGotoSetpointType> goto_;
    std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_;
    std::shared_ptr<px4_ros2::RatesSetpointType> rates_;
};
