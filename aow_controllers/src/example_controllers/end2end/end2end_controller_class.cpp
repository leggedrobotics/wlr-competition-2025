#include "aow_controllers/example_controllers/end2end/end2end_controller_class.hpp"
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

End2EndController::End2EndController(const ControllerConfig& config)
    : config_(config)
    , model_loaded_(false)
    , last_closest_idx_(0)
    , initialized_(false)
{
}

bool End2EndController::initialize(rclcpp::Node* node)
{
    // Store node pointer for publishing
    node_ = node;
    
    // Create controller-specific publishers if node is available
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "End2End controller publishers initialized");
        
        // Declare and load controller-specific parameters
        try {
            node_->declare_parameter("num_waypoints", 10);
            node_->declare_parameter("search_range", 120);
            node_->declare_parameter("fixed_interval_m", 0.30);
            RCLCPP_INFO(node_->get_logger(), "End2End specific parameters declared");
        } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
            RCLCPP_DEBUG(node_->get_logger(), "End2End parameters already declared");
        }
        
        // Load parameters directly from ROS parameter server
        num_waypoints_ = node_->get_parameter("num_waypoints").as_int();
        search_range_ = node_->get_parameter("search_range").as_int();
        fixed_interval_m_ = node_->get_parameter("fixed_interval_m").as_double();
        
        std::cout << "End2End Controller Parameters:" << std::endl;
        std::cout << "  ↳ num_waypoints: " << num_waypoints_ << std::endl;
        std::cout << "  ↳ search_range: " << search_range_ << std::endl;
        std::cout << "  ↳ fixed_interval_m: " << fixed_interval_m_ << " m" << std::endl;
        
        waypoints_robot_pub_ = node_->create_publisher<geometry_msgs::msg::PoseArray>(
            "/debug/waypoints_robot_frame", 10);
    }
    
    bool success = loadJITModel();
    initialized_ = success;  // Set initialization state
    return success;
}

bool End2EndController::loadJITModel()
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
        
        // Enable deterministic behavior for reproducible results
        torch::manual_seed(42);  // Fixed seed for determinism
        if (config_.device.is_cuda()) {
            torch::cuda::manual_seed(42);
            torch::cuda::manual_seed_all(42);
            // Note: For full determinism, you may need to set environment variables:
            // CUBLAS_WORKSPACE_CONFIG=:4096:8 or CUBLAS_WORKSPACE_CONFIG=:16:8
            std::cout << "CUDA deterministic mode enabled. For full determinism, set CUBLAS_WORKSPACE_CONFIG=:4096:8" << std::endl;
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

ControllerOutput End2EndController::compute_control(const ControllerObservations& observations)
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
    
    // Get next waypoints for trajectory tracking
    std::vector<TrajectoryPose> next_waypoints = getNextWaypoints(
        observations.trajectory, 
        observations.base_position, 
        observations.base_orientation
    );
    
    torch::Tensor trajectory_tensor = createTrajectoryTensor(next_waypoints);
    
    Eigen::VectorXd processed_joint_positions = preprocessObservations(observations);
    
    torch::Tensor joint_positions = torch::from_blob(
        processed_joint_positions.data(), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    torch::Tensor joint_velocities = torch::from_blob(
        const_cast<double*>(observations.joint_velocities.data()), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    torch::Tensor projected_gravity = torch::from_blob(
        const_cast<double*>(observations.projected_gravity.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    torch::Tensor base_lin_vel = torch::from_blob(
        const_cast<double*>(observations.base_linear_velocity.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    torch::Tensor base_ang_vel = torch::from_blob(
        const_cast<double*>(observations.base_angular_velocity.data()), 
        {3}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    torch::Tensor last_actions = torch::from_blob(
        const_cast<double*>(observations.last_actions.data()), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    torch::Tensor last_last_actions = torch::from_blob(
        const_cast<double*>(observations.last_last_actions.data()), 
        {16}, torch::kFloat64
    ).to(config_.device).to(torch::kFloat32);
    
    std::vector<torch::jit::IValue> inputs;
    
    auto obs_tensor = torch::cat({
        joint_positions.slice(0, 0, 12),    // [12] - current joint positions (legs only)
        joint_velocities,                   // [16] - current joint velocities
        last_actions,                       // [16] - previous motor commands
        last_last_actions,                  // [16] - last last motor commands
        projected_gravity,                  // [3]  - projected gravity vector
        base_lin_vel,                       // [3]  - base linear velocity
        base_ang_vel,                       // [3]  - base angular velocity
        trajectory_tensor                   // [30] - flattened trajectory waypoints
    }, 0);
    
    obs_tensor = obs_tensor.unsqueeze(0);
    inputs.push_back(obs_tensor);
    
    auto network_output = jit_module_.forward(inputs);
    torch::Tensor actions_tensor = network_output.toTensor();
    
    actions_tensor = actions_tensor.to(torch::kCPU);
    float *actions_data = actions_tensor.data_ptr<float>();
    
    output.raw_actions = Eigen::Map<Eigen::VectorXf>(actions_data, 16).cast<double>();
    output.joint_commands = postProcessActions(output.raw_actions);

    debug_visualization(next_waypoints);
    
    return output;
}

std::vector<TrajectoryPose> End2EndController::getNextWaypoints(
    const std::vector<TrajectoryPose>& trajectory,
    const std::vector<double>& base_position,
    const std::vector<double>& base_orientation)
{
    std::vector<TrajectoryPose> next_waypoints;
    if (trajectory.empty()) {
        return next_waypoints;
    }

    const int    num_waypoints     = num_waypoints_;     
    const int    search_range      = search_range_;      
    const double fixed_interval_m  = fixed_interval_m_;  

    const double robot_x = base_position.size() > 0 ? base_position[0] : 0.0;
    const double robot_y = base_position.size() > 1 ? base_position[1] : 0.0;
    const double robot_z = base_position.size() > 2 ? base_position[2] : 0.0;

    // Quaternion order assumed: (w, x, y, z).
    const double qw = base_orientation[0];
    const double qx = base_orientation[1];
    const double qy = base_orientation[2];
    const double qz = base_orientation[3];

    tf2::Quaternion q_wb(qx, qy, qz, qw);  // world->base rotation (orientation of base in world)
    q_wb.normalize();
    tf2::Matrix3x3 R_wb(q_wb);
    tf2::Matrix3x3 R_bw = R_wb.transpose(); 

    // Find closest point within a forward window 
    const int N = static_cast<int>(trajectory.size());
    int closest_idx = std::max(0, last_closest_idx_);
    double min_distance = std::numeric_limits<double>::max();

    const int search_start = std::max(0, last_closest_idx_);
    const int search_end   = std::min(last_closest_idx_ + search_range, N - 1); // inclusive

    for (int i = search_start; i <= search_end; ++i) {
        const double dx = trajectory[i].x - robot_x;
        const double dy = trajectory[i].y - robot_y;
        double d = std::sqrt(dx * dx + dy * dy);
        
        if (i < last_closest_idx_) {
            d += 0.1 * (last_closest_idx_ - i);  // Small penalty for going backward
        }
        
        if (d < min_distance) {
            min_distance = d;
            closest_idx = i;
        }
    }
    // Ensure forward progress
    closest_idx = std::max(closest_idx, last_closest_idx_);
    last_closest_idx_ = closest_idx;

    // Build cumulative arclength (meters, in world XY)
    std::vector<double> cumlen(N, 0.0);
    for (int i = 1; i < N; ++i) {
        const double dx = trajectory[i].x - trajectory[i - 1].x;
        const double dy = trajectory[i].y - trajectory[i - 1].y;
        cumlen[i] = cumlen[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    const int last_idx = N - 1;

    const double s0        = cumlen[closest_idx];
    const double sEnd      = cumlen[last_idx];
    const double remaining = std::max(0.0, sEnd - s0);

    // interval = min(3 * remaining / (num_waypoints - 1), fixed_interval)
    const int denom_int      = std::max(1, num_waypoints - 1);
    const double compressed  = 3.0 * remaining / static_cast<double>(denom_int);
    const double interval_m  = std::min(compressed, fixed_interval_m);

    std::vector<int> idxs;
    idxs.reserve(num_waypoints);
    for (int k = 0; k < num_waypoints; ++k) {
        // goal is to find the index k with the smallest |cumlen[k] - target_s|
        const double target_s = s0 + static_cast<double>(k - 1) * interval_m;
        auto it_begin = cumlen.begin() + closest_idx;
        auto it_end   = cumlen.begin() + last_idx + 1;
        auto it       = std::lower_bound(it_begin, it_end, target_s);
        int id = static_cast<int>(it - cumlen.begin());

        if (it == it_end) {
            id = last_idx;
        } else if (id > closest_idx) {
            const double left  = target_s - cumlen[id - 1];
            const double right = cumlen[id] - target_s;
            if (left <= right) id -= 1;
        }

        idxs.push_back(id);
    }

    next_waypoints.reserve(num_waypoints);
    TrajectoryPose last_wp_tf{};
    bool have_last_tf = false;

    const tf2::Vector3 p0_w(robot_x, robot_y, robot_z);
    const tf2::Quaternion q_bw = q_wb.inverse();  

    for (int i = 0; i < num_waypoints; ++i) {
        const int waypoint_idx = idxs[i];
        const TrajectoryPose& wp = trajectory[waypoint_idx];

        const tf2::Vector3 pw(wp.x, wp.y, wp.z);

        tf2::Vector3 d = pw - p0_w;
        tf2::Vector3 pb = R_bw * d;

        tf2::Quaternion q_wp_world;
        q_wp_world.setRPY(0.0, 0.0, wp.yaw);

        tf2::Quaternion q_rel = q_bw * q_wp_world;
        q_rel.normalize();

        double rel_roll, rel_pitch, rel_yaw;
        tf2::Matrix3x3(q_rel).getRPY(rel_roll, rel_pitch, rel_yaw);

        TrajectoryPose wp_tf = wp;
        wp_tf.x   = pb.x();
        wp_tf.y   = pb.y();
        wp_tf.z   = pb.z();
        wp_tf.yaw = rel_yaw;

        next_waypoints.push_back(wp_tf);
        last_wp_tf = wp_tf;
        have_last_tf = true;

        if (waypoint_idx == last_idx) {
            for (int j = i + 1; j < num_waypoints; ++j) {
                next_waypoints.push_back(last_wp_tf);
            }
            break;
        }
    }

    while ((int)next_waypoints.size() < num_waypoints && have_last_tf) {
        next_waypoints.push_back(last_wp_tf);
    }

    return next_waypoints;
}

torch::Tensor End2EndController::createTrajectoryTensor(const std::vector<TrajectoryPose>& waypoints)
{
    // Create a flat tensor with 10 waypoints * 3 values (x, y, yaw) = 30 values
    std::vector<float> trajectory_data;
    trajectory_data.reserve(30);

    torch::Tensor trajectory_tensor;
    
    if (!waypoints.empty()) {
        std::vector<float> trajectory_data;
        trajectory_data.reserve(waypoints.size() * 3);
        
        for (const auto& waypoint : waypoints) {
            trajectory_data.push_back(static_cast<float>(waypoint.x));
            trajectory_data.push_back(static_cast<float>(waypoint.y));
            trajectory_data.push_back(static_cast<float>(waypoint.yaw));
        }
        
        // Create flattened 1D tensor [30] to match PyTorch concatenation
        trajectory_tensor = torch::from_blob(
            trajectory_data.data(), 
            {static_cast<long>(trajectory_data.size())}, 
            torch::kFloat32
        ).clone().to(config_.device).requires_grad_(false);
    } else {
        std::cerr << "Warning: No waypoints available - creating empty tensor" << std::endl;
        trajectory_tensor = torch::zeros({30}, torch::kFloat32).to(config_.device).requires_grad_(false); 
    }

    return trajectory_tensor;
}

Eigen::VectorXd End2EndController::preprocessObservations(const ControllerObservations& observations)
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

Eigen::VectorXd End2EndController::postProcessActions(const Eigen::VectorXd& raw_actions)
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

void End2EndController::debug_visualization(const std::vector<TrajectoryPose>& next_waypoints)
{
    if (!waypoints_robot_pub_ || !node_) {
        return;
    }

    geometry_msgs::msg::PoseArray waypoints_robot_msg;
    waypoints_robot_msg.header.frame_id = "base";
    waypoints_robot_msg.header.stamp = node_->get_clock()->now();

    for (const auto& wp_robot : next_waypoints) {
        geometry_msgs::msg::Pose pose;
        pose.position.x = wp_robot.x;
        pose.position.y = wp_robot.y;
        pose.position.z = wp_robot.z;

        tf2::Quaternion q_b;
        q_b.setRPY(0.0, 0.0, wp_robot.yaw);
        pose.orientation.x = q_b.x();
        pose.orientation.y = q_b.y();
        pose.orientation.z = q_b.z();
        pose.orientation.w = q_b.w();

        waypoints_robot_msg.poses.push_back(pose);
    }
    
    waypoints_robot_pub_->publish(waypoints_robot_msg);
}

void End2EndController::reset()
{
    if (node_) {
        RCLCPP_INFO(node_->get_logger(), "End2End controller reset called");
    }
    
    initialized_ = false;
    last_closest_idx_ = 0;
    
    bool success = loadJITModel();
    initialized_ = success;
}

bool End2EndController::is_initialized() const
{
    return initialized_;
}
