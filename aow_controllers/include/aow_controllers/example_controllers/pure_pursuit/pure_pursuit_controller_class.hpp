#ifndef PURE_PURSUIT_CONTROLLER_CLASS_HPP
#define PURE_PURSUIT_CONTROLLER_CLASS_HPP

#include <torch/torch.h>
#include <torch/script.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include "aow_controllers/utils/trajectory_utils.hpp"
#include "aow_controllers/common/base_controller.hpp"

class PurePursuitController : public BaseController {
public:

    explicit PurePursuitController(const ControllerConfig& config);
    
    ~PurePursuitController() = default;
    
    // BaseController interface implementation
    bool initialize(rclcpp::Node* node) override;
    ControllerOutput compute_control(const ControllerObservations& observations) override;
    std::string getControllerType() const override { return "PurePursuit"; }
    void reset() override;
    bool is_initialized() const override;
    
    // Pure pursuit specific methods
    bool loadJITModel();
    
    Eigen::VectorXd preprocessObservations(const ControllerObservations& observations);
    Eigen::VectorXd postProcessActions(const Eigen::VectorXd& raw_actions);
    Eigen::Vector3d getVelocityCommand(const ControllerObservations& observations);
    void getLookAheadPoint(
        const std::vector<TrajectoryPose>& trajectory,
        const std::vector<double>& base_position,
        int& last_closest_idx,
        const ControllerObservations& observations,
        Eigen::Vector3d& lookahead_point);
    
    // Debug visualization method
    void debug_visualization(const Eigen::Vector3d& lookahead_point);
    
    // Pure pursuit velocity calculation
    Eigen::Vector3d getDesiredVelocityCommand(
        const Eigen::Vector3d& lookahead_point,
        double desired_speed) const;
    
    // Get desired speed for pure pursuit
    double getDesiredSpeed() const;
    
    // Transform trajectory waypoints from world frame to robot base frame
    Eigen::MatrixX3d transformTrajectoryToRobotFrame(
        const std::vector<TrajectoryPose>& trajectory,
        const std::vector<double>& base_position,
        const std::vector<double>& base_orientation) const;



private:

    ControllerConfig config_;
    
    // Cached controller parameters (loaded once during initialization)
    double lookahead_distance_;
    double max_linear_velocity_;
    double max_angular_velocity_;
    double desired_speed_;
    
    torch::jit::script::Module jit_module_;
    bool model_loaded_;
    int last_closest_idx_;
    bool initialized_;  // Track initialization state for reset operations
    
    // Controller-specific publishers
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr lookahead_point_pub_;
    
};

#endif // PURE_PURSUIT_CONTROLLER_CLASS_HPP
