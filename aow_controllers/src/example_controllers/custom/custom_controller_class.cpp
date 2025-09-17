#include "aow_controllers/example_controllers/custom/custom_controller_class.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <iostream>
#include <filesystem>
#include <algorithm>
#include <limits>
#include <cmath>

// CRITICAL: Disable gradients for entire file - prevents any backprop tree creation
static torch::NoGradGuard global_no_grad;

CustomController::CustomController(const ControllerConfig& config)
    : config_(config)
    , model_loaded_(false)
    , last_closest_idx_(0)
    , initialized_(false)
{
}

bool CustomController::initialize(rclcpp::Node* node)
{
    node_ = node;
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Custom controller initialized");
    }
    
    bool success = loadJITModel();
    initialized_ = success;  // Set initialization state
    return success;
}

bool CustomController::loadJITModel()
{
    // TODO: Implement model loading
    return true; 
}

ControllerOutput CustomController::compute_control(const ControllerObservations& observations)
{
    // TODO: Implement custom control logic here

    ControllerOutput output;
    
    if (!model_loaded_)
    {
        output.joint_commands = Eigen::VectorXd::Zero(16);
        output.raw_actions = Eigen::VectorXd::Zero(16);
        return output;
    }
    
    Eigen::VectorXd processed_joint_positions = preprocessObservations(observations);
    
    output.raw_actions = Eigen::VectorXd::Zero(16);
    output.joint_commands = postProcessActions(output.raw_actions);
    
    debug_visualization(output);
    
    return output;
}

Eigen::VectorXd CustomController::preprocessObservations(const ControllerObservations& observations)
{
    Eigen::VectorXd processed_joint_positions(16);
    for (Eigen::Index i = 0; i < observations.joint_positions.size(); ++i) {
        double processed_position = observations.joint_positions[i];
        
        if (config_.use_default_joint_positions_offset_observation) {
            processed_position -= config_.default_joint_positions[i];
        }
        
        processed_position = std::fmod(processed_position + 2 * M_PI, 4 * M_PI);
        if (processed_position < 0) {
            processed_position += 4 * M_PI;
        }
        processed_position -= 2 * M_PI;
        
        processed_joint_positions[i] = processed_position;
    }
    
    return processed_joint_positions;
}

Eigen::VectorXd CustomController::postProcessActions(const Eigen::VectorXd& raw_actions)
{
    // TODO: Implement custom post-processing logic here
    Eigen::VectorXd processed_actions = raw_actions;

    processed_actions.segment(0, 12) *= config_.leg_joint_scale;
    processed_actions.segment(12, 4) *= config_.wheel_joint_scale;

    if (config_.use_default_joint_positions_offset_action)
    {
        processed_actions += config_.default_joint_positions;
    }
    
    return processed_actions;
}

void CustomController::debug_visualization(const ControllerOutput& output)
{
    // TODO: Implement custom visualization
}

void CustomController::reset()
{
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Custom controller reset called");
    }
    
    // Set initialization state to false during reset
    initialized_ = false;
    
    // Reset any internal state variables here
    last_closest_idx_ = 0;
    
    // TODO: Add your custom reset logic here
    // This might include resetting neural network states, 
    // clearing internal buffers, resetting trajectory tracking state, etc.
    
    // For now, just set initialized back to true
    // In a real implementation, you might call loadJITModel() or other reset logic
    initialized_ = true;
}

bool CustomController::is_initialized() const
{
    return initialized_;
}
