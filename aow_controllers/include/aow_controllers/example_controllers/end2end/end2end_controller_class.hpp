#ifndef END2END_CONTROLLER_CLASS_HPP
#define END2END_CONTROLLER_CLASS_HPP

#include <torch/torch.h>
#include <torch/script.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "aow_controllers/utils/trajectory_utils.hpp"
#include "aow_controllers/common/base_controller.hpp"

class End2EndController : public BaseController {
public:

    explicit End2EndController(const ControllerConfig& config);
    
    ~End2EndController() = default;
    
    // BaseController interface implementation
    bool initialize(rclcpp::Node* node) override;
    ControllerOutput compute_control(const ControllerObservations& observations) override;
    std::string getControllerType() const override { return "End2End"; }
    void reset() override;
    bool is_initialized() const override;
    
    // End2End specific methods
    bool loadJITModel();
    
    torch::Tensor createTrajectoryTensor(const std::vector<TrajectoryPose>& waypoints);
    Eigen::VectorXd preprocessObservations(const ControllerObservations& observations);
    Eigen::VectorXd postProcessActions(const Eigen::VectorXd& raw_actions);
    std::vector<TrajectoryPose> getNextWaypoints(const std::vector<TrajectoryPose>& trajectory, const std::vector<double>& base_position, const std::vector<double>& base_orientation);
    
    // Debug visualization method
    void debug_visualization(const std::vector<TrajectoryPose>& next_waypoints);

private:

    ControllerConfig config_;
    
    // Cached controller parameters (loaded once during initialization)
    int num_waypoints_;
    int search_range_;
    double fixed_interval_m_;
    
    torch::jit::script::Module jit_module_;
    bool model_loaded_;
    int last_closest_idx_;
    bool initialized_;  // Track initialization state for reset operations
    
    // Controller-specific publishers
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr waypoints_robot_pub_;
    
};

#endif // END2END_CONTROLLER_CLASS_HPP
