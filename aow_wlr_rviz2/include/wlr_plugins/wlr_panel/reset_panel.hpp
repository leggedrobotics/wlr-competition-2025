#ifndef RESET_PANEL_HPP__
#define RESET_PANEL_HPP__

#include <rviz_common/panel.hpp>

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>     // AsyncParametersClient
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/bool.hpp>           // for reset state signaling
#include <std_msgs/msg/string.hpp>         // for /robot_description topic

#include "ros_gz_interfaces/srv/control_world.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"

namespace aow_wlr_rviz2
{

class ResetPanel : public rviz_common::Panel
{
    Q_OBJECT

public:
    explicit ResetPanel(QWidget *parent = nullptr);
    ~ResetPanel();

private Q_SLOTS:
    void onResetButtonClicked();

private:
    void onInitialize() override;

    // Sequence: full reset (all=true) -> respawn
    void fullWorldReset();
    void spawnRobot();
    


    // Get URDF XML from parameter or topic
    bool fetchRobotDescriptionParam(std::string &xml_out);
    bool fetchRobotDescriptionTopic(std::string &xml_out,
                                    std::chrono::milliseconds timeout = std::chrono::milliseconds(1200));

    // Helper: discover FQN of robot_state_publisher by scanning services
    bool discover_rsp_fqn(std::string &fqn_out);

    // UI
    QLabel* title_label_{};
    QPushButton* reset_button_{};
    QLabel* status_label_{};


    // ROS
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr reset_publisher_;           // Legacy reset signal
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr reset_state_publisher_;     // Reset state: true=resetting, false=ready
    rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedPtr world_control_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedPtr   spawn_client_;


    // Config
    std::string world_name_{"default"};
    std::string robot_name_{"aow"};

    // Preferred source: parameter on robot_state_publisher
    std::string robot_description_param_{"robot_description"};

    // Secondary source: latched topic containing URDF XML
    std::string robot_description_topic_{"/robot_description"};

    // Fallback: path to a .urdf/.sdf file on disk (optional)
    std::string model_file_path_{};

    // Desired spawn pose (matches your launch defaults)
    double spawn_x_{0.0}, spawn_y_{0.0}, spawn_z_{0.7}, spawn_yaw_{0.0};
};

} // namespace aow_wlr_rviz2

#endif // RESET_PANEL_HPP__
