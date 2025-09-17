#ifndef AOW_ROBOT_CONFIG_HPP
#define AOW_ROBOT_CONFIG_HPP

#include <vector>
#include <string>
#include <array>

namespace aow_robot_config {

    // Joint names in Isaac Lab order (expected by neural network)
    // Isaac order: LF_HAA, LH_HAA, RF_HAA, RH_HAA, LF_HFE, LH_HFE, RF_HFE, RH_HFE, LF_KFE, LH_KFE, RF_KFE, RH_KFE, LF_WHEEL, LH_WHEEL, RF_WHEEL, RH_WHEEL
    static const std::vector<std::string> JOINT_NAMES = {
        "LF_HAA", "LH_HAA", "RF_HAA", "RH_HAA",
        "LF_HFE", "LH_HFE", "RF_HFE", "RH_HFE",
        "LF_KFE", "LH_KFE", "RF_KFE", "RH_KFE",
        "LF_WHEEL", "LH_WHEEL", "RF_WHEEL", "RH_WHEEL"
    };

    // Corresponding topic names for joint commands (Isaac order)
    // Position control for leg joints (HAA, HFE, KFE), velocity control for wheels
    static const std::vector<std::string> JOINT_TOPICS = {
        "LF_HAA_position_cmd", "LH_HAA_position_cmd", 
        "RF_HAA_position_cmd", "RH_HAA_position_cmd",
        "LF_HFE_position_cmd", "LH_HFE_position_cmd", 
        "RF_HFE_position_cmd", "RH_HFE_position_cmd",
        "LF_KFE_position_cmd", "LH_KFE_position_cmd", 
        "RF_KFE_position_cmd", "RH_KFE_position_cmd",
        "LF_WHEEL_velocity_cmd", "LH_WHEEL_velocity_cmd", 
        "RF_WHEEL_velocity_cmd", "RH_WHEEL_velocity_cmd"
    };

    // Number of joints
    static constexpr size_t NUM_JOINTS = 16;

    // Joint order mappings between ROS and Isaac Lab
    // ROS joint order: LF_HAA, LF_HFE, LF_KFE, LF_WHEEL, LH_HAA, LH_HFE, LH_KFE, LH_WHEEL, RF_HAA, RF_HFE, RF_KFE, RF_WHEEL, RH_HAA, RH_HFE, RH_KFE, RH_WHEEL
    // Isaac joint order: LF_HAA, LH_HAA, RF_HAA, RH_HAA, LF_HFE, LH_HFE, RF_HFE, RH_HFE, LF_KFE, LH_KFE, RF_KFE, RH_KFE, LF_WHEEL, LH_WHEEL, RF_WHEEL, RH_WHEEL
    
    // Mapping from ROS index to Isaac index: For Isaac position i, take ROS position ROS_TO_ISAAC_MAPPING[i]
    static const std::array<int, 16> ROS_TO_ISAAC_MAPPING = {0, 4, 8, 12, 1, 5, 9, 13, 2, 6, 10, 14, 3, 7, 11, 15};

} // namespace aow_robot_config

#endif // AOW_ROBOT_CONFIG_HPP
