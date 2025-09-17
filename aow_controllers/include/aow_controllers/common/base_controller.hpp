#pragma once

#include <string>
#include <unordered_map>
#include <any>
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include <torch/torch.h>
#include <rclcpp/rclcpp.hpp>
#include "aow_controllers/utils/trajectory_utils.hpp"

struct ControllerConfig {
    double leg_joint_scale = 0.5;
    double wheel_joint_scale = 5.0;
    bool use_default_joint_positions_offset_observation = false;
    bool use_default_joint_positions_offset_action = true;
    Eigen::VectorXd default_joint_positions;
    std::string model_path;
    torch::Device device = torch::kCPU;
    // Note: Controller-specific parameters are now handled directly by each controller via ROS parameter server
};

struct ControllerObservations {
    Eigen::Vector3d base_linear_velocity;
    Eigen::Vector3d base_angular_velocity;
    Eigen::Vector3d projected_gravity;
    Eigen::VectorXd joint_positions;
    Eigen::VectorXd joint_velocities;
    Eigen::VectorXd last_actions;
    Eigen::VectorXd last_last_actions;
    
    std::vector<double> base_position;
    std::vector<double> base_orientation;
    std::vector<TrajectoryPose> trajectory;
    std::vector<double> velocity_commands;
};

struct ControllerOutput {
    Eigen::VectorXd joint_commands;
    Eigen::VectorXd raw_actions;
};

class BaseController {
public:
    virtual ~BaseController() = default;
    virtual bool initialize(rclcpp::Node* node) = 0;
    virtual ControllerOutput compute_control(const ControllerObservations& observations) = 0;
    virtual std::string getControllerType() const = 0;
    virtual void reset() = 0;  // Reset method for controller state
    virtual bool is_initialized() const = 0;  // Check if controller is initialized

protected:
    rclcpp::Node* node_ = nullptr;  // Access to ROS node for publishing
};
