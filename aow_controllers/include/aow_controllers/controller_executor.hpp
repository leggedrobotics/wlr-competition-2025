#ifndef CONTROLLER_EXECUTOR_HPP
#define CONTROLLER_EXECUTOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "ros_gz_interfaces/srv/control_world.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "aow_controllers/utils/aow_robot_config.hpp"
#include "aow_controllers/common/base_controller.hpp"
#include "aow_controllers/example_controllers/end2end/end2end_controller_class.hpp"

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <eigen3/Eigen/Dense>
#include <mutex>
#include <memory>
#include <atomic>
#include <vector>
#include <chrono>
#include <cmath>

class ControllerExecutor : public rclcpp::Node {
public:

    ControllerExecutor();

private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void resetStateCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void initializeController();           
    void loadTrajectory();
    void initializeParameters();
    void initializeSubscribers();
    void initializePublishers();
    void initializeLockstep();
    void lockstepTimerCallback();
    void stepSimulationLockstep();
    void publishColorCodedPathVisualization(double robot_x, double robot_y, double distance_to_closest_waypoint);
    void calculateAndPublishScore(const ControllerOutput& output, const ControllerObservations& observations, double distance_to_waypoint, int current_closest_idx);

    ControllerObservations getObservations();

    void inference();
    void publishJointCommands();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset_state_sub_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr trajectory_pub_;
    std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_pubs_;
    
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr color_coded_path_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr score_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr integrated_time_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr off_path_time_pub_;
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr world_control_client_;

    // Timers for periodic execution
    rclcpp::TimerBase::SharedPtr timer_;           // Inference timer (50Hz simulation time)
    rclcpp::TimerBase::SharedPtr timer_pub_;       // Command publishing timer (200Hz)
    rclcpp::TimerBase::SharedPtr lockstep_timer_;  // Lockstep stepping timer (50Hz wall time)

    std::map<std::string, double> joint_commands_;

    std::unique_ptr<BaseController> controller_;
    std::atomic<bool> resetting_{false};
    
    std::vector<TrajectoryPose> trajectory_poses_;
    bool trajectory_loaded_;
    
    bool odometry_received_{false};
    bool joint_states_received_{false};

    geometry_msgs::msg::Vector3 base_lin_vel_; 
    geometry_msgs::msg::Vector3 base_ang_vel_; 
    geometry_msgs::msg::Vector3 base_pos_world_; 
    geometry_msgs::msg::Quaternion base_orientation_;
    Eigen::Vector3d projected_gravity_;         
    Eigen::VectorXd joint_positions_;           
    Eigen::VectorXd joint_velocities_;          
    Eigen::VectorXd default_joint_positions_;   
    Eigen::VectorXd last_actions_;    
    Eigen::VectorXd last_last_actions_;

    Eigen::Matrix3d R_wb_;
    std::string controller_type_;
    double inference_hz_;
    double command_hz_;
    double leg_joint_scale_;
    double wheel_joint_scale_;
    bool use_default_joint_positions_offset_observation_;
    bool use_default_joint_positions_offset_action_;
    bool use_lockstep_;
    uint32_t lockstep_steps_per_inference_;
    
    std::atomic<bool> inference_running_;
    std::atomic<uint32_t> steps_taken_since_inference_;
    
    std::string model_path_;
    std::string trajectory_file_;
    visualization_msgs::msg::MarkerArray color_coded_path_markers_;
    int marker_id_counter_;
    
    rclcpp::Time start_time_;
    bool score_calculation_started_;
    bool trajectory_finished_;
    int last_trajectory_index_;
    double accumulated_lateral_error_squared_;
    int lateral_error_samples_;
    double final_score_;
    
    // New time-based scoring variables
    double integrated_time_;  // Total time accumulated
    rclcpp::Time off_path_start_time_;  // When robot went >1m off path
    double total_off_path_time_;  // Total time spent >1m off path
    bool currently_off_path_;  // Is robot currently >1m off path
    bool elimination_penalty_applied_;  // Has -inf penalty been applied

    double debug_counter_; 
    
    // Running average frequency tracking
    std::vector<double> inference_timestamps_;
    std::vector<double> joint_publish_timestamps_;
    static constexpr size_t FREQ_WINDOW_SIZE = 10;
    
    // Gazebo clock tracking for lockstep
    rclcpp::Time current_sim_time_;         // Time when we sent the step command
    rclcpp::Time last_sim_step_time_;
    rclcpp::Time current_time_;
    rclcpp::Time last_step_time_;
    bool step_command_sent_{false};       // Did we send the step command?
    bool first_inference_completed_{false}; // Track if we've completed first inference
    uint32_t failed_step_attempts_{0};    // Count failed service calls for retry logic
    static constexpr double SIM_STEP_SIZE = 0.005;  // 5ms simulation step size
    static constexpr uint32_t MAX_FAILED_ATTEMPTS = 100;  // Retry after 5 failed attempts
    std::atomic<bool> step_completed_{false};
};

#endif // CONTROLLER_EXECUTOR_HPP
