#ifndef CUSTOM_CONTROLLER_CLASS_HPP
#define CUSTOM_CONTROLLER_CLASS_HPP

#include <torch/torch.h>
#include <torch/script.h>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <string>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "aow_controllers/utils/trajectory_utils.hpp"
#include "aow_controllers/common/base_controller.hpp"

class CustomController : public BaseController {
public:
    explicit CustomController(const ControllerConfig& config);
    ~CustomController() = default;
    
    bool initialize(rclcpp::Node* node) override;
    ControllerOutput compute_control(const ControllerObservations& observations) override;
    std::string getControllerType() const override { return "Custom"; }
    void reset() override;
    bool is_initialized() const override;
    
    bool loadJITModel();
    Eigen::VectorXd preprocessObservations(const ControllerObservations& observations);
    Eigen::VectorXd postProcessActions(const Eigen::VectorXd& raw_actions);
    void debug_visualization(const ControllerOutput& output);

private:
    ControllerConfig config_;
    torch::jit::script::Module jit_module_;
    bool model_loaded_;
    int last_closest_idx_;
    bool initialized_;  // Track initialization state for reset operations
};

#endif // CUSTOM_CONTROLLER_CLASS_HPP
