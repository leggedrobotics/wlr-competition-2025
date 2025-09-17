#include "aow_controllers/example_controllers/pure_pursuit/pure_pursuit_controller_class.hpp"
#include "aow_controllers/utils/trajectory_utils.hpp"
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

PurePursuitController::PurePursuitController(const ControllerConfig& config)
    : config_(config)
    , model_loaded_(false)
    , last_closest_idx_(0)
    , initialized_(false)
{
}

bool PurePursuitController::initialize(rclcpp::Node* node)
{
    // Store node pointer for potential future publishing
    node_ = node;
    
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "PurePursuit controller node access established");
        
        // Declare and load controller-specific parameters
        try {
            node_->declare_parameter("lookahead_distance", 0.8);
            node_->declare_parameter("max_linear_velocity", 1.5);
            node_->declare_parameter("max_angular_velocity", 1.0);
            node_->declare_parameter("desired_speed", 1.0);
            RCLCPP_INFO(node_->get_logger(), "Pure Pursuit specific parameters declared");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
            RCLCPP_DEBUG(node_->get_logger(), "Pure Pursuit parameters already declared");
        }
        
        // Load parameters directly from ROS parameter server
        lookahead_distance_ = node_->get_parameter("lookahead_distance").as_double();
        max_linear_velocity_ = node_->get_parameter("max_linear_velocity").as_double();
        max_angular_velocity_ = node_->get_parameter("max_angular_velocity").as_double();
        desired_speed_ = node_->get_parameter("desired_speed").as_double();
        
        std::cout << "Pure Pursuit Controller Parameters:" << std::endl;
        std::cout << "  ↳ lookahead_distance: " << lookahead_distance_ << " m" << std::endl;
        std::cout << "  ↳ max_linear_velocity: " << max_linear_velocity_ << " m/s" << std::endl;
        std::cout << "  ↳ max_angular_velocity: " << max_angular_velocity_ << " rad/s" << std::endl;
        std::cout << "  ↳ desired_speed: " << desired_speed_ << " m/s" << std::endl;
        
        // Add pure pursuit specific publishers
        lookahead_point_pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(
            "/pure_pursuit/lookahead_point", 10);
    }
    
    bool success = loadJITModel();
    initialized_ = success;  // Set initialization state
    return success;
}

bool PurePursuitController::loadJITModel()
{
    try
    {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("aow_controllers");
        std::string full_model_path = (std::filesystem::path(package_share_dir) / config_.model_path).string();
        
        if (!std::filesystem::exists(full_model_path))
        {
            std::cerr << "Model file not found: " << full_model_path << std::endl;
            return false;
        }
        
        jit_module_ = torch::jit::load(full_model_path, config_.device);
        jit_module_.eval();
        
        // CRITICAL: Disable gradients for all model parameters
        for (auto param : jit_module_.parameters()) {
            param.set_requires_grad(false);
        }
        
        model_loaded_ = true;
        std::cout << "Successfully loaded PyTorch JIT model: " << config_.model_path << std::endl;
        std::cout << "Using device: " << (config_.device.is_cuda() ? "CUDA" : "CPU") << std::endl;
        
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "Failed to load PyTorch JIT model: " << e.what() << std::endl;
        model_loaded_ = false;
        return false;
    }
}

ControllerOutput PurePursuitController::compute_control(const ControllerObservations& observations)
{
    torch::InferenceMode guard;
    
    ControllerOutput output;
    
    if (!model_loaded_)
    {
        std::cerr << "Model not loaded - cannot compute control" << std::endl;
        output.joint_commands = Eigen::VectorXd::Zero(16);
        output.raw_actions = Eigen::VectorXd::Zero(16);
        return output;
    }

    Eigen::Vector3d velocity_command = getVelocityCommand(observations);
    
    ControllerObservations mutable_obs = observations;
    mutable_obs.velocity_commands = {velocity_command.x(), velocity_command.y(), velocity_command.z()};
    
    torch::Tensor base_lin_vel = torch::from_blob(
        const_cast<double*>(mutable_obs.base_linear_velocity.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);

    torch::Tensor base_ang_vel = torch::from_blob(
        const_cast<double*>(mutable_obs.base_angular_velocity.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);

    torch::Tensor projected_gravity = torch::from_blob(
        const_cast<double*>(mutable_obs.projected_gravity.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);

    torch::Tensor velocity_commands = torch::from_blob(
        const_cast<double*>(mutable_obs.velocity_commands.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);

    Eigen::VectorXd processed_joint_positions = preprocessObservations(mutable_obs);

    torch::Tensor joint_positions = torch::from_blob(
        processed_joint_positions.data(), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);

    torch::Tensor joint_velocities = torch::from_blob(
        const_cast<double*>(mutable_obs.joint_velocities.data()), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);

    torch::Tensor last_action = torch::from_blob(
        const_cast<double*>(mutable_obs.last_actions.data()), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    std::vector<torch::jit::IValue> inputs;
    
    
    auto obs_tensor = torch::cat({
        base_lin_vel,                       // [3]  - base linear velocity
        base_ang_vel,                       // [3]  - base angular velocity
        projected_gravity,                  // [3]  - projected gravity vector
        velocity_commands,                  // [3]  - velocity commands
        joint_positions,                    // [16] - current joint positions
        joint_velocities,                   // [16] - current joint velocities
        last_action                         // [16] - previous motor commands
    }, 0);  
    
    obs_tensor = obs_tensor.unsqueeze(0);
    inputs.push_back(obs_tensor);
    
    auto network_output = jit_module_.forward(inputs);
    torch::Tensor actions_tensor = network_output.toTensor();
    
    actions_tensor = actions_tensor.to(torch::kCPU);
    float *actions_data = actions_tensor.data_ptr<float>();
    
    output.raw_actions = Eigen::Map<Eigen::VectorXf>(actions_data, 16).cast<double>();
    output.joint_commands = postProcessActions(output.raw_actions);

    return output;
}

Eigen::Vector3d PurePursuitController::getVelocityCommand(const ControllerObservations& observations) {

    Eigen::Vector3d velocity_command = Eigen::Vector3d::Zero();
    Eigen::Vector3d lookahead_point = Eigen::Vector3d::Zero();

    getLookAheadPoint(
        observations.trajectory,
        observations.base_position,
        last_closest_idx_,
        observations,
        lookahead_point
    );
    
    debug_visualization(lookahead_point);
    
    double desired_speed = getDesiredSpeed();

    velocity_command = getDesiredVelocityCommand(lookahead_point, desired_speed);

    return velocity_command;
}

Eigen::Vector3d PurePursuitController::getDesiredVelocityCommand(
    const Eigen::Vector3d& lookahead_point,
    double desired_speed) const {

    Eigen::Vector3d velocity_command = Eigen::Vector3d::Zero();
    
    const double x = lookahead_point.x();  
    const double y = lookahead_point.y();  
    
    const double angle_to_target = std::atan2(y, x);
    const double distance_to_target = std::sqrt(x * x + y * y);
    
    if (distance_to_target < 0.01) {
        return velocity_command; 
    }

    const double angular_velocity = 2.0 * desired_speed * std::sin(angle_to_target) / distance_to_target;
    double forward_velocity = desired_speed;
    
    const double turn_reduction_factor = std::cos(angle_to_target);
    forward_velocity *= std::max(0.3, turn_reduction_factor); 
    
    velocity_command(0) = forward_velocity;
    velocity_command(1) = 0.0;
    velocity_command(2) = angular_velocity;
    
    const double max_linear_velocity = max_linear_velocity_;
    const double max_angular_velocity = max_angular_velocity_;
    
    velocity_command(0) = std::max(-max_linear_velocity, std::min(max_linear_velocity, velocity_command(0)));
    velocity_command(2) = std::max(-max_angular_velocity, std::min(max_angular_velocity, velocity_command(2)));

    return velocity_command;
}

double PurePursuitController::getDesiredSpeed() const {
    return desired_speed_;
}

void PurePursuitController::getLookAheadPoint(
    const std::vector<TrajectoryPose>& trajectory,
    const std::vector<double>& base_position,
    int& last_closest_idx,
    const ControllerObservations& observations,
    Eigen::Vector3d& lookahead_point)
{
    Eigen::MatrixX3d transformed_points = transformTrajectoryToRobotFrame(
        trajectory, 
        base_position, 
        observations.base_orientation
    );

    const int N = static_cast<int>(transformed_points.rows());
    
    double robot_x = observations.base_position[0];
    double robot_y = observations.base_position[1];

    // Use utility function to find and update closest trajectory index
    int closest_idx = TrajectoryUtils::findAndUpdateClosestTrajectoryIndex(
        trajectory, robot_x, robot_y, last_closest_idx, 100);  // 100 waypoint search range

    std::vector<double> cumlen(N, 0.0);
    for (int i = closest_idx + 1; i < N; ++i) {
        const double dx = transformed_points(i, 0) - transformed_points(i - 1, 0);
        const double dy = transformed_points(i, 1) - transformed_points(i - 1, 1);
        cumlen[i] = cumlen[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    const double lookahead_distance = lookahead_distance_;

    // Find lookahead point at the specified distance
    bool lookahead_found = false;
    for (int i = closest_idx; i < N; ++i) {
        if (cumlen[i] >= lookahead_distance) {
            lookahead_point = transformed_points.row(i);
            lookahead_found = true;
            break;
        }
    }
    
    // If no lookahead point found (near end of trajectory), use the last waypoint
    if (!lookahead_found) {
        lookahead_point = transformed_points.row(N - 1);
    }

}

Eigen::MatrixX3d PurePursuitController::transformTrajectoryToRobotFrame(
    const std::vector<TrajectoryPose>& trajectory,
    const std::vector<double>& base_position,
    const std::vector<double>& base_orientation) const
{
    const size_t n_points = trajectory.size();
    
    Eigen::MatrixX3d waypoints(n_points, 3);
    for (size_t i = 0; i < n_points; ++i) {
        waypoints(i, 0) = trajectory[i].x;
        waypoints(i, 1) = trajectory[i].y;
        waypoints(i, 2) = 0.0;
    }
    
    Eigen::Quaterniond quat(
        base_orientation[0],
        base_orientation[1],
        base_orientation[2],
        base_orientation[3]
    );
    
    Eigen::Matrix3d R_wb = quat.toRotationMatrix();
    Eigen::Matrix3d R_bw = R_wb.transpose();
    
    Eigen::Vector3d base_pos_3d(base_position[0], base_position[1], 0.0);
    waypoints.rowwise() -= base_pos_3d.transpose();
    
    Eigen::MatrixX3d transformed_points = (R_bw * waypoints.transpose()).transpose();
    
    return transformed_points;
}

Eigen::VectorXd PurePursuitController::preprocessObservations(const ControllerObservations& observations)
{
    Eigen::VectorXd processed_joint_positions(16);
    for (Eigen::Index i = 0; i < observations.joint_positions.size(); ++i) {
        double processed_position = observations.joint_positions[i];
        
        if (config_.use_default_joint_positions_offset_observation) {
            processed_position -= config_.default_joint_positions[i];
        }
        
        // Apply wrapping to [-2π, 2π] range (IsaacLab style)
        processed_position = std::fmod(processed_position + 2 * M_PI, 4 * M_PI);
        if (processed_position < 0) {
            processed_position += 4 * M_PI;
        }
        processed_position -= 2 * M_PI;
        
        processed_joint_positions[i] = processed_position;
    }
    
    return processed_joint_positions;
}

Eigen::VectorXd PurePursuitController::postProcessActions(const Eigen::VectorXd& raw_actions)
{
    // Post-process actions for hardware commands
    // joint order Isaaclab 
    // LF_HAA, LH_HAA, RF_HAA, RH_HAA, LF_HFE, LH_HFE, RF_HFE, RH_HFE, LF_KFE, LH_KFE, RF_KFE, RH_KFE, LF_WHEEL, LH_WHEEL, RF_WHEEL, RH_WHEEL
    Eigen::VectorXd processed_actions = raw_actions;

    // Scale joint commands for hardware limits using configured scale factors
    processed_actions.segment(0, 12) *= config_.leg_joint_scale;   // Scale leg joints (HAA, HFE, KFE)
    processed_actions.segment(12, 4) *= config_.wheel_joint_scale; // Scale wheel joints

    if (config_.use_default_joint_positions_offset_action)
    {
        processed_actions += config_.default_joint_positions;
    }
    
    return processed_actions;
}

void PurePursuitController::debug_visualization(const Eigen::Vector3d& lookahead_point)
{
    if (!lookahead_point_pub_ || !node_) {
        return;
    }

    geometry_msgs::msg::PointStamped lookahead_msg;
    lookahead_msg.header.frame_id = "base";
    lookahead_msg.header.stamp = node_->get_clock()->now();
    
    lookahead_msg.point.x = lookahead_point[0];
    lookahead_msg.point.y = lookahead_point[1];
    lookahead_msg.point.z = lookahead_point[2];
    
    lookahead_point_pub_->publish(lookahead_msg);
}

void PurePursuitController::reset()
{
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "Pure Pursuit controller reset called");
    }
    
    initialized_ = false;
    last_closest_idx_ = 0;
    
    bool success = loadJITModel();
    initialized_ = success;
}

bool PurePursuitController::is_initialized() const
{
    return initialized_;
}
