#include "aow_controllers/controller_executor.hpp"
#include "aow_controllers/utils/trajectory_utils.hpp"
#include "aow_controllers/example_controllers/end2end/end2end_controller_class.hpp"
#include "aow_controllers/example_controllers/pure_pursuit/pure_pursuit_controller_class.hpp"
#include "aow_controllers/example_controllers/custom/custom_controller_class.hpp"
#include <thread>
#include <chrono>


// ros2 service call /world/default/control ros_gz_interfaces/srv/ControlWorld "{world_control: {pause: true}}" this works

ControllerExecutor::ControllerExecutor()
    : Node("controller_executor"),
      trajectory_loaded_(false),
      joint_positions_(16),
      joint_velocities_(16),
      default_joint_positions_(16),
      last_actions_(16),
      last_last_actions_(16),
      marker_id_counter_(0),
      score_calculation_started_(false),
      trajectory_finished_(false),
      inference_running_(false),
      steps_taken_since_inference_(0),
      integrated_time_(0.0),
      total_off_path_time_(0.0),
      currently_off_path_(false),
      elimination_penalty_applied_(false)
{
    initializeParameters();

    base_lin_vel_ = geometry_msgs::msg::Vector3();
    base_ang_vel_ = geometry_msgs::msg::Vector3();
    projected_gravity_ = Eigen::Vector3d(0.0, 0.0, -1.0);
    joint_positions_ = Eigen::VectorXd::Zero(16);
    joint_velocities_ = Eigen::VectorXd::Zero(16);
    last_actions_ = Eigen::VectorXd::Zero(16);
    last_last_actions_ = Eigen::VectorXd::Zero(16);

    initializeController();

    initializeSubscribers();

    initializePublishers();

    initializeLockstep();

    loadTrajectory();

    auto inference_period = std::chrono::milliseconds(static_cast<int>(1000.0 / inference_hz_));
    auto command_period = std::chrono::milliseconds(static_cast<int>(1000.0 / command_hz_));

    // Always create normal simulation time timer for inference
    timer_ = this->create_timer(
        inference_period,
        std::bind(&ControllerExecutor::inference, this));
    
    if (use_lockstep_) {
        // Fast wall clock timer for responsive simulation stepping (200Hz = 5ms)
        auto lockstep_period = std::chrono::milliseconds(2);
        lockstep_timer_ = this->create_wall_timer(
            lockstep_period,
            std::bind(&ControllerExecutor::lockstepTimerCallback, this));
        // use lockstep if the sim runs faster than the inference can run, otherwise no need
        RCLCPP_DEBUG(this->get_logger(), "LOCKSTEP MODE: Inference at %.1f Hz, stepping at 200Hz for low latency", inference_hz_);
    } else {
        RCLCPP_DEBUG(this->get_logger(), "NORMAL MODE: Using simulation time timer for inference at %.1f Hz", inference_hz_);
    }

    timer_pub_ = this->create_timer(
        command_period,
        std::bind(&ControllerExecutor::publishJointCommands, this));

        debug_counter_ = 0.0;
    
    // Initialize frequency tracking vectors (they start empty)

    RCLCPP_INFO(this->get_logger(), "Controller Executor node initialized successfully.");
}

void ControllerExecutor::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (resetting_.load()) {
        return;
    }
    
    base_lin_vel_ = msg->twist.twist.linear;
    base_ang_vel_ = msg->twist.twist.angular;

    base_pos_world_.x = msg->pose.pose.position.x;
    base_pos_world_.y = msg->pose.pose.position.y;
    base_pos_world_.z = msg->pose.pose.position.z;

    Eigen::Quaterniond quat(msg->pose.pose.orientation.w,
                            msg->pose.pose.orientation.x,
                            msg->pose.pose.orientation.y,
                            msg->pose.pose.orientation.z);

    base_orientation_ = msg->pose.pose.orientation;

    // body to world
    R_wb_ = quat.toRotationMatrix();

    // calculate projected gravity (w_T_b).T * w_g = b_g
    Eigen::Vector3d gravity_world(0.0, 0.0, -1.0);
    projected_gravity_ = R_wb_.transpose() * gravity_world;
    
    odometry_received_ = true;
}

void ControllerExecutor::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    if (resetting_.load()) {
        return;
    }
    
    std::vector<double> stored_positions = msg->position;
    std::vector<double> stored_velocities = msg->velocity;

    // ROS joint order: LF_HAA, LF_HFE, LF_KFE, LF_WHEEL, LH_HAA, LH_HFE, LH_KFE, LH_WHEEL, RF_HAA, RF_HFE, RF_KFE, RF_WHEEL, RH_HAA, RH_HFE, RH_KFE, RH_WHEEL
    // Isaac joint order: LF_HAA, LH_HAA, RF_HAA, RH_HAA, LF_HFE, LH_HFE, RF_HFE, RH_HFE, LF_KFE, LH_KFE, RF_KFE, RH_KFE, LF_WHEEL, LH_WHEEL, RF_WHEEL, RH_WHEEL

    std::vector<double> reordered_positions(aow_robot_config::NUM_JOINTS);
    std::vector<double> reordered_velocities(aow_robot_config::NUM_JOINTS);

    for (size_t i = 0; i < aow_robot_config::NUM_JOINTS; ++i) {
        reordered_positions[i] = stored_positions[aow_robot_config::ROS_TO_ISAAC_MAPPING[i]];
        reordered_velocities[i] = stored_velocities[aow_robot_config::ROS_TO_ISAAC_MAPPING[i]];
    }

    joint_positions_ = Eigen::Map<Eigen::VectorXd>(reordered_positions.data(), reordered_positions.size());
    joint_velocities_ = Eigen::Map<Eigen::VectorXd>(reordered_velocities.data(), reordered_velocities.size());
    
    joint_states_received_ = true;
}

void ControllerExecutor::resetStateCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data) {
        resetting_.store(true);
        odometry_received_ = false;
        joint_states_received_ = false;
        RCLCPP_INFO(this->get_logger(), "Reset state: Starting reset process");
        
        if (controller_) {
            controller_->reset();
            
            if (controller_->is_initialized()) {
                RCLCPP_INFO(this->get_logger(), "Controller reset successful and initialized");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Controller reset failed - not initialized!");
            }
        }
        color_coded_path_markers_.markers.clear();
        marker_id_counter_ = 0;
        
        auto clear_markers = visualization_msgs::msg::MarkerArray();
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = "odom";
        clear_marker.header.stamp = this->get_clock()->now();
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        clear_markers.markers.push_back(clear_marker);
        color_coded_path_pub_->publish(clear_markers);
    
    } else {
        RCLCPP_INFO(this->get_logger(), "Reset state: Reset complete, resuming inference if all data received (odometry & joint states)");
        
        // Reset robot state data
        base_lin_vel_ = geometry_msgs::msg::Vector3();
        base_ang_vel_ = geometry_msgs::msg::Vector3();
        base_pos_world_ = geometry_msgs::msg::Vector3();
        base_orientation_ = geometry_msgs::msg::Quaternion();
        projected_gravity_ = Eigen::Vector3d(0.0, 0.0, -1.0);
        joint_positions_ = default_joint_positions_;
        joint_velocities_ = Eigen::VectorXd::Zero(16);
        last_actions_ = Eigen::VectorXd::Zero(16);
        last_last_actions_ = Eigen::VectorXd::Zero(16);
        R_wb_ = Eigen::Matrix3d::Identity();
        if (use_lockstep_) {
            inference_running_.store(false);
            first_inference_completed_ = false;  // Reset initialization flag
            steps_taken_since_inference_.store(0);  // Reset step counter
            step_completed_.store(true);  // Ready for first step
        }
        score_calculation_started_ = false;
        trajectory_finished_ = false;
        accumulated_lateral_error_squared_ = 0.0;
        lateral_error_samples_ = 0;
        final_score_ = 0.0;
        last_trajectory_index_ = 0;
        
        // Reset new time-based scoring variables
        integrated_time_ = 0.0;
        total_off_path_time_ = 0.0;
        currently_off_path_ = false;
        elimination_penalty_applied_ = false;
        
        if (trajectory_loaded_) {
            trajectory_loaded_ = false;
            loadTrajectory();
        }
        
        resetting_.store(false);
    }
}

void ControllerExecutor::initializeController()
{   
    ControllerConfig config;
    config.leg_joint_scale = leg_joint_scale_;
    config.wheel_joint_scale = wheel_joint_scale_;
    config.use_default_joint_positions_offset_observation = use_default_joint_positions_offset_observation_;
    config.use_default_joint_positions_offset_action = use_default_joint_positions_offset_action_;
    config.default_joint_positions = default_joint_positions_;
    config.model_path = model_path_;
    config.device = torch::cuda::is_available() ? torch::kCUDA : torch::kCPU;
    if (torch::cuda::is_available()) {
        RCLCPP_INFO(this->get_logger(), "Using GPU (CUDA) for neural network inference!");
        RCLCPP_INFO(this->get_logger(), "GPU Device: %d", config.device.index());
    } else {
        RCLCPP_WARN(this->get_logger(), "GPU not available, using CPU for neural network inference");
        RCLCPP_INFO(this->get_logger(), "For better performance, ensure CUDA is installed and GPU is available");
    }
    if (controller_type_ == "end2end") {
        controller_ = std::make_unique<End2EndController>(config);
        RCLCPP_INFO(this->get_logger(), "Creating End2End Controller");
    } else if (controller_type_ == "pure_pursuit") {
        controller_ = std::make_unique<PurePursuitController>(config);
        RCLCPP_INFO(this->get_logger(), "Creating Pure Pursuit Controller");
    } else if (controller_type_ == "custom") {
        controller_ = std::make_unique<CustomController>(config);
        RCLCPP_INFO(this->get_logger(), "Creating Custom Controller");
    } else {
        RCLCPP_ERROR(this->get_logger(), "Unknown controller type: %s. Supported types: end2end, pure_pursuit, custom", controller_type_.c_str());
        return;
    }
    
    if (!controller_->initialize(this)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize %s controller", controller_type_.c_str());
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "%s Controller initialized successfully", controller_type_.c_str());
}


ControllerObservations ControllerExecutor::getObservations()
{
    ControllerObservations obs;
    
    obs.base_linear_velocity = Eigen::Vector3d(
        base_lin_vel_.x, base_lin_vel_.y, base_lin_vel_.z);
    obs.base_angular_velocity = Eigen::Vector3d(
        base_ang_vel_.x, base_ang_vel_.y, base_ang_vel_.z);
            
    obs.projected_gravity = projected_gravity_;
    obs.joint_positions = joint_positions_;
    obs.joint_velocities = joint_velocities_;
    obs.last_actions = last_actions_;
    obs.last_last_actions = last_last_actions_;
    
    obs.base_position = {base_pos_world_.x, base_pos_world_.y, base_pos_world_.z};
    obs.base_orientation = {
        base_orientation_.w, base_orientation_.x, 
        base_orientation_.y, base_orientation_.z
    };
    
    obs.trajectory = trajectory_poses_;
    
    return obs;
}

void ControllerExecutor::loadTrajectory()
{
    trajectory_poses_ = TrajectoryUtils::loadTrajectory(trajectory_file_, this->get_logger());
    trajectory_loaded_ = !trajectory_poses_.empty();
    
    if (trajectory_loaded_) {
        auto path_msg = TrajectoryUtils::createTrajectoryVisualization(trajectory_poses_, this->get_clock());
        trajectory_pub_->publish(path_msg);
        RCLCPP_INFO(this->get_logger(), "Trajectory visualization published successfully");
    }
}

void ControllerExecutor::lockstepTimerCallback()
{
    if (resetting_.load()) {
        return;
    }
    
    // Handle initialization phase: Keep stepping until first inference is ready
    if (!first_inference_completed_) {
        bool inference_ready = controller_->is_initialized() && 
                              odometry_received_ && 
                              joint_states_received_ && 
                              trajectory_loaded_;
        
        if (!inference_ready) {
            // Only step if previous step completed - avoid flooding Gazebo!
            if (!step_completed_.load()) {
                return;  // Wait for current step to complete
            }
            
            // During initialization, keep stepping single steps to let Gazebo settle
            RCLCPP_INFO(this->get_logger(), "🔄 INIT: Free-running simulation (waiting for: %s%s%s%s)", 
                       !controller_->is_initialized() ? "controller " : "",
                       !odometry_received_ ? "odometry " : "",
                       !joint_states_received_ ? "joint_states " : "",
                       !trajectory_loaded_ ? "trajectory " : "");
                       
            auto request = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
            request->world_control.pause = true;
            request->world_control.step = true;
            request->world_control.multi_step = 1;  // Single step during init
            
            if (world_control_client_->service_is_ready()) {
                step_completed_.store(false);  // Mark step as in-progress
                world_control_client_->async_send_request(request);
            }
            
            // Initialize timing for when we switch to controlled lockstep
            current_sim_time_ = this->get_clock()->now();
            last_sim_step_time_ = current_sim_time_;
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "🎯 INIT: All systems ready → Switching to controlled lockstep mode");
            // Mark initialization as complete to prevent re-entering this block
            first_inference_completed_ = true;
            // Reset step counter for clean lockstep start
            steps_taken_since_inference_.store(0);
            // Don't return here - fall through to normal lockstep logic
        }
    }
    
    // Normal lockstep operation
    if (inference_running_.load()) {
        return;  // Inference running, don't step
    }

    current_sim_time_ = this->get_clock()->now();
    
    // Check if we've already taken enough steps for this inference cycle
    uint32_t current_steps = steps_taken_since_inference_.load();
    if (current_steps >= lockstep_steps_per_inference_) {
        return;  // Already took enough steps, waiting for next inference
    }
    
    // Check if simulation time has advanced (step completed)
    // Use seconds comparison to avoid time source issues
    if (current_sim_time_.seconds() > last_sim_step_time_.seconds()) {
        step_completed_.store(true);
    }
    
    // Take next step if ready
    if (step_completed_.load()) {
        uint32_t new_step_count = steps_taken_since_inference_.fetch_add(1) + 1;
        RCLCPP_DEBUG(this->get_logger(), "🚶 LOCKSTEP: Taking step %u/%u (sim time: %.3fs)", 
                   new_step_count, lockstep_steps_per_inference_, current_sim_time_.seconds());
        
        stepSimulationLockstep();
        step_completed_.store(false);
        last_sim_step_time_ = current_sim_time_;
        
        // Check if we've reached the step limit
        if (new_step_count >= lockstep_steps_per_inference_) {
            RCLCPP_DEBUG(this->get_logger(), "✅ LOCKSTEP: Completed %u/%u steps → Waiting for inference", 
                       new_step_count, lockstep_steps_per_inference_);
        }
    }
}

void ControllerExecutor::inference()
{
    uint32_t prev_steps = steps_taken_since_inference_.load();
    RCLCPP_DEBUG(this->get_logger(), "INFERENCE: Starting (completed %u steps, sim time: %.3fs)", 
               prev_steps, this->get_clock()->now().seconds());
    
    debug_counter_ = 0.0;
    inference_running_.store(true);
    steps_taken_since_inference_.store(0);  // Reset counter for new inference cycle
    
    // 5 second sleep for testing lockstep behavior
    // std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (resetting_.load() || !controller_->is_initialized() || !odometry_received_ || !joint_states_received_ || !trajectory_loaded_) {

        for (size_t i = 0; i < aow_robot_config::JOINT_NAMES.size(); ++i) {
            if (i < 12) {
                joint_commands_[aow_robot_config::JOINT_NAMES[i]] = default_joint_positions_[i];
            } else {
                joint_commands_[aow_robot_config::JOINT_NAMES[i]] = 0.0;
            }
        }
        inference_running_.store(false);
        return;
    }

    ControllerObservations observations = getObservations();

    ControllerOutput output = controller_->compute_control(observations);
    
    last_last_actions_ = last_actions_; 
    last_actions_ = output.raw_actions;
    for (size_t i = 0; i < aow_robot_config::JOINT_NAMES.size(); ++i)
    {
        joint_commands_[aow_robot_config::JOINT_NAMES[i]] = output.joint_commands[i];
    }
    
    // Update closest trajectory index using utility function
    int current_closest_idx = TrajectoryUtils::findAndUpdateClosestTrajectoryIndex(
        trajectory_poses_, observations.base_position[0], observations.base_position[1], 
        last_trajectory_index_, 100);
    
    // Calculate distance once and use for both score and visualization
    double distance_for_viz = TrajectoryUtils::calculateDistanceToClosestWaypoint(
        trajectory_poses_, observations.base_position[0], observations.base_position[1], 
        current_closest_idx, 100);
    
    calculateAndPublishScore(output, observations, distance_for_viz, current_closest_idx);
    
    publishColorCodedPathVisualization(
        observations.base_position[0],  
        observations.base_position[1],  
        distance_for_viz
    );
    
    // Mark first inference as completed (for lockstep initialization)
    if (!first_inference_completed_) {
        first_inference_completed_ = true;
        RCLCPP_DEBUG(this->get_logger(), "INFERENCE: First inference completed - lockstep synchronized");
    }
    
    // Clear the inference running flag to allow next lockstep steps
    inference_running_.store(false);
    RCLCPP_DEBUG(this->get_logger(), "INFERENCE: Completed → Lockstep can resume stepping");
}

void ControllerExecutor::stepSimulationLockstep()
{
    if (resetting_.load()) {
        return;
    }

    auto request = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    request->world_control.pause = true;
    request->world_control.step = true;
    request->world_control.multi_step = 1.0;
    
    if (world_control_client_->service_is_ready()) {
        world_control_client_->async_send_request(request);
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "⚠️  LOCKSTEP: World control service not ready - simulation may not be in lockstep mode");
    }
}

void ControllerExecutor::publishColorCodedPathVisualization(double robot_x, double robot_y, double distance_to_closest_waypoint)
{
    auto marker = TrajectoryUtils::createColorCodedPathMarker(
        robot_x, robot_y, distance_to_closest_waypoint, 
        marker_id_counter_++, this->get_clock());
    
    color_coded_path_markers_.markers.push_back(marker);
    if (color_coded_path_markers_.markers.size() > 5000) {
        color_coded_path_markers_.markers.erase(color_coded_path_markers_.markers.begin());
    }
    
    color_coded_path_pub_->publish(color_coded_path_markers_);
}

void ControllerExecutor::publishJointCommands()
{
    if (resetting_.load()) {
        return;
    }
    
    for (const auto &joint : aow_robot_config::JOINT_NAMES)
    {
        auto msg = std_msgs::msg::Float64();
        msg.data = joint_commands_[joint];
        joint_pubs_[joint]->publish(msg);
    }
}

void ControllerExecutor::initializeParameters()
{
    this->declare_parameter("controller_type", std::string("end2end"));
    controller_type_ = this->get_parameter("controller_type").as_string();
    
    this->declare_parameter("controller_config_file", std::string("controller_specific_config.yaml"));
    std::string controller_config_file = this->get_parameter("controller_config_file").as_string();
    
    this->declare_parameter("inference_hz", 50.0);
    inference_hz_ = this->get_parameter("inference_hz").as_double();
    
    this->declare_parameter("command_hz", 200.0);
    command_hz_ = this->get_parameter("command_hz").as_double();
    
    this->declare_parameter("leg_joint_scale", 0.5);
    leg_joint_scale_ = this->get_parameter("leg_joint_scale").as_double();
    
    this->declare_parameter("wheel_joint_scale", 5.0);
    wheel_joint_scale_ = this->get_parameter("wheel_joint_scale").as_double();

    this->declare_parameter("use_default_joint_positions_offset_observation", false);
    use_default_joint_positions_offset_observation_ = this->get_parameter("use_default_joint_positions_offset_observation").as_bool();
    
    this->declare_parameter("use_default_joint_positions_offset_action", true);
    use_default_joint_positions_offset_action_ = this->get_parameter("use_default_joint_positions_offset_action").as_bool();
    
    this->declare_parameter("use_lockstep", false);
    use_lockstep_ = this->get_parameter("use_lockstep").as_bool();
    
    // Calculate lockstep steps automatically from frequency ratio (command_hz / inference_hz)
    // This ensures simulation advances at the right rate to match command frequency
    lockstep_steps_per_inference_ = static_cast<uint32_t>(std::round(command_hz_ / inference_hz_));
    
    RCLCPP_INFO(this->get_logger(), "LOCKSTEP CONFIG: use_lockstep=%s, steps_per_inference=%u (auto-calculated from %.1f/%.1f)", 
                use_lockstep_ ? "TRUE" : "FALSE", lockstep_steps_per_inference_, command_hz_, inference_hz_);

    this->declare_parameter("model_path", std::string("example_controllers/end2end/end2end_policy/aow_end2end_policy_2.pt"));
    model_path_ = this->get_parameter("model_path").as_string();

    this->declare_parameter("trajectory_file", std::string("trajectory_generation/example_trajectory_csv/rsl_path_scale5.0_rounds2.csv"));
    trajectory_file_ = this->get_parameter("trajectory_file").as_string();

    this->declare_parameter("default_joint_positions", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.4, -0.4, 0.4, -0.4, -0.8, 0.8, -0.8, 0.8, 0.0, 0.0, 0.0, 0.0});
    auto default_positions = this->get_parameter("default_joint_positions").as_double_array();
    default_joint_positions_ = Eigen::Map<Eigen::VectorXd>(default_positions.data(), default_positions.size());

    try {
        this->declare_parameter("use_sim_time", true);
    } catch (const rclcpp::exceptions::ParameterAlreadyDeclaredException&) {
    }
    this->set_parameter(rclcpp::Parameter("use_sim_time", true));

    RCLCPP_INFO(this->get_logger(), "Parameters initialized successfully");
}

void ControllerExecutor::initializeSubscribers()
{
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry", 10,
        std::bind(&ControllerExecutor::odometryCallback, this, std::placeholders::_1));

    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ControllerExecutor::jointStateCallback, this, std::placeholders::_1));
    
    reset_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/reset_state", 10,
        std::bind(&ControllerExecutor::resetStateCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Subscribers initialized");
}

void ControllerExecutor::initializePublishers()
{
    // Create publishers for commands and debugging using hardcoded topic names
    // Create latched publisher for trajectory visualization (world frame - publish once)
    rclcpp::QoS trajectory_qos(1);  // Only need 1 message - we publish once
    trajectory_qos.transient_local();  // Enable latching - message persists for new subscribers
    trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("/trajectory_visualization", trajectory_qos);
    
    color_coded_path_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/color_coded_path", 10);

    score_pub_ = this->create_publisher<std_msgs::msg::Float64>("/score", 10);
    integrated_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/integrated_time", 10);
    off_path_time_pub_ = this->create_publisher<std_msgs::msg::Float64>("/off_path_time", 10);
    
    // Create world control service client for lockstep synchronization
    world_control_client_ = this->create_client<ros_gz_interfaces::srv::ControlWorld>("/world/default/control");
    
    // Create individual joint command publishers
    for (size_t i = 0; i < aow_robot_config::JOINT_NAMES.size(); ++i)
    {
        auto pub = this->create_publisher<std_msgs::msg::Float64>(aow_robot_config::JOINT_TOPICS[i], 10);
        joint_pubs_[aow_robot_config::JOINT_NAMES[i]] = pub;
        joint_commands_[aow_robot_config::JOINT_NAMES[i]] = 0.0;
    }
    
    RCLCPP_INFO(this->get_logger(), "Publishers initialized");
}

void ControllerExecutor::calculateAndPublishScore(const ControllerOutput& output, const ControllerObservations& observations, double distance_to_waypoint, int current_closest_idx)
{
    if (!score_calculation_started_) {
        start_time_ = this->get_clock()->now();
        score_calculation_started_ = true;
        last_trajectory_index_ = 0;
        accumulated_lateral_error_squared_ = 0.0;
        lateral_error_samples_ = 0;
        final_score_ = 0.0;
        integrated_time_ = 0.0;
        total_off_path_time_ = 0.0;
        currently_off_path_ = false;
        elimination_penalty_applied_ = false;
    }
    
    if (trajectory_finished_) {
        std_msgs::msg::Float64 score_msg;
        score_msg.data = final_score_;
        score_pub_->publish(score_msg);
        return;
    }
    
    auto current_time = this->get_clock()->now();
    double dt = (current_time - start_time_).seconds() - integrated_time_;
    integrated_time_ += dt;
    
    bool is_off_path = distance_to_waypoint > 0.3;
    
    if (is_off_path && !currently_off_path_) {
        currently_off_path_ = true;
        off_path_start_time_ = current_time;
    } else if (!is_off_path && currently_off_path_) {
        currently_off_path_ = false;
        double off_path_duration = (current_time - off_path_start_time_).seconds();
        total_off_path_time_ += off_path_duration;
        
        if (total_off_path_time_ > 5.0 && !elimination_penalty_applied_) {
            elimination_penalty_applied_ = true;
        }
    } else if (is_off_path && currently_off_path_) {
        double current_off_path_duration = (current_time - off_path_start_time_).seconds();
        double total_with_current = total_off_path_time_ + current_off_path_duration;
        
        if (total_with_current > 5.0 && !elimination_penalty_applied_) {
            elimination_penalty_applied_ = true;
        }
    }
    
    if (currently_off_path_ && !elimination_penalty_applied_) {
        double current_off_path_duration = (current_time - off_path_start_time_).seconds();
    }
    
    int current_idx = current_closest_idx;
    
    bool trajectory_completed = (current_idx >= static_cast<int>(trajectory_poses_.size() - 1));
    
    // === TIME-BASED SCORING SYSTEM ===
    // Score = integrated_time (in seconds)
    // If eliminated (>1m off path for >5s), add large penalty
    double score = integrated_time_;
    
    if (elimination_penalty_applied_) {
        score = std::numeric_limits<double>::infinity();
    }
    
    if (trajectory_completed && !trajectory_finished_) {
        trajectory_finished_ = true;
        if (currently_off_path_) {
            double remaining_off_path_time = (current_time - off_path_start_time_).seconds();
            total_off_path_time_ += remaining_off_path_time;
            currently_off_path_ = false;
        }
        final_score_ = score;
        RCLCPP_INFO(this->get_logger(), "Trajectory completed! Final time: %.2f seconds, Off-path time: %.2f seconds", 
                    integrated_time_, total_off_path_time_);
    }
    
    std_msgs::msg::Float64 score_msg;
    score_msg.data = trajectory_finished_ ? final_score_ : score;
    score_pub_->publish(score_msg);
    
    std_msgs::msg::Float64 time_msg;
    time_msg.data = integrated_time_;
    integrated_time_pub_->publish(time_msg);
    
    std_msgs::msg::Float64 off_path_msg;
    double current_total_off_path = total_off_path_time_;
    if (currently_off_path_) {
        current_total_off_path += (current_time - off_path_start_time_).seconds();
    }
    off_path_msg.data = current_total_off_path;
    off_path_time_pub_->publish(off_path_msg);
}

void ControllerExecutor::initializeLockstep()
{
    if (!use_lockstep_) {
        RCLCPP_INFO(this->get_logger(), "Lockstep disabled - simulation will run normally");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Initializing lockstep simulation control...");
    
    // Wait for world control service
    while (!world_control_client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Waiting for /world/default/control service...");
    }
    
    // Pause the simulation at startup
    auto request = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    request->world_control.pause = true;
    
    RCLCPP_INFO(this->get_logger(), "Pausing simulation for lockstep control...");
    world_control_client_->async_send_request(request);
    
    RCLCPP_INFO(this->get_logger(), "Simple lockstep initialized - wall timer steps simulation, inference timer waits for sim time");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControllerExecutor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
