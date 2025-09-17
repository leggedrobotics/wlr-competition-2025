#include "wlr_plugins/wlr_panel/reset_panel.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFont>
#include <QSizePolicy>
#include <rviz_common/display_context.hpp>
#include <QMessageBox>
#include <QTimer>
#include <thread>
#include <future>
#include <regex>
#include <cmath>
#include <map>
#include <vector>
#include <string>

using namespace std::chrono_literals;

namespace aow_wlr_rviz2
{

ResetPanel::ResetPanel(QWidget *parent)
    : rviz_common::Panel(parent)
{
    // UI
    QVBoxLayout* main_layout = new QVBoxLayout;
    setLayout(main_layout);
    
    title_label_ = new QLabel("Full Reset + Respawn (Gazebo)");
    QFont title_font = title_label_->font();
    title_font.setPointSize(14);
    title_font.setBold(true);
    title_label_->setFont(title_font);
    title_label_->setAlignment(Qt::AlignCenter);
    main_layout->addWidget(title_label_);

    main_layout->addSpacing(10);

    reset_button_ = new QPushButton("Reset World (all=true) + Respawn Robot");
    reset_button_->setMinimumHeight(40);
    reset_button_->setStyleSheet(
        "QPushButton {"
        "  background-color: #ff6b6b; color: white; border: none;"
        "  border-radius: 5px; font-size: 12px; font-weight: bold;"
        "}"
        "QPushButton:hover { background-color: #ff5252; }"
        "QPushButton:pressed { background-color: #e53e3e; }"
    );
    main_layout->addWidget(reset_button_);

    main_layout->addSpacing(10);

    status_label_ = new QLabel("Ready");
    status_label_->setAlignment(Qt::AlignCenter);
    status_label_->setStyleSheet("QLabel { color: #4a5568; font-size: 10px; }");
    main_layout->addWidget(status_label_);

    main_layout->addStretch();

    connect(reset_button_, &QPushButton::clicked, this, &ResetPanel::onResetButtonClicked);
}

ResetPanel::~ResetPanel() = default;

void ResetPanel::onInitialize()
{
    // ROS node from RViz
    node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

    // Optional: notify your controller to clear internal state
    // reset_publisher_ = node_->create_publisher<std_msgs::msg::Empty>("/reset_controller", 10);
    
    // Reset state publisher: true=resetting, false=ready
    reset_state_publisher_ = node_->create_publisher<std_msgs::msg::Bool>("/reset_state", 10);



    // Gazebo services (bridged via ros_gz_bridge parameter_bridge)
    world_control_client_ = node_->create_client<ros_gz_interfaces::srv::ControlWorld>(
        "/world/" + world_name_ + "/control");

    spawn_client_ = node_->create_client<ros_gz_interfaces::srv::SpawnEntity>(
        "/world/" + world_name_ + "/create");

    status_label_->setText("Status: Waiting for Gazebo services...");

    // Wait in background so the UI doesn't block
    std::thread([this]() {
        while (!world_control_client_->wait_for_service(1s) || !spawn_client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for Gazebo services.");
                return;
            }
            RCLCPP_INFO(node_->get_logger(), "Gazebo services not available yet, waiting...");
        }
        RCLCPP_INFO(node_->get_logger(), "Gazebo services available.");
        QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                  Q_ARG(QString, "Status: Services ready. Click to reset & respawn."));
    }).detach();
}

void ResetPanel::onResetButtonClicked()
{
    // Signal that reset is starting
    if (reset_state_publisher_) {
        std_msgs::msg::Bool reset_state_msg;
        reset_state_msg.data = true;  // true = resetting in progress
        reset_state_publisher_->publish(reset_state_msg);
    }
    
    // Optional: tell your controller to reset (avoid immediate motion after respawn)
    // if (reset_publisher_) {
    //     std_msgs::msg::Empty msg;
    //     reset_publisher_->publish(msg);
    // }
    fullWorldReset();
}

void ResetPanel::fullWorldReset()
{
    if (!world_control_client_) {
        RCLCPP_ERROR(node_->get_logger(), "World control client not initialized.");
        return;
    }

    QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                              Q_ARG(QString, "Status: Full reset (all=true)..."));

    auto req = std::make_shared<ros_gz_interfaces::srv::ControlWorld::Request>();
    req->world_control.reset.all = true;       // rewind world to initial SDF state
    req->world_control.reset.model_only = false;
    req->world_control.reset.time_only = false;

    auto fut = world_control_client_->async_send_request(
        req,
        [this](rclcpp::Client<ros_gz_interfaces::srv::ControlWorld>::SharedFuture future)
        {
            try {
                auto resp = future.get();
                if (resp->success) {
                    RCLCPP_INFO(node_->get_logger(), "Full world reset succeeded. Respawning robot...");
                    QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                              Q_ARG(QString, "Status: Respawning robot..."));
                    // Small delay to let reset settle before we call /create
                    QTimer::singleShot(150, [this]() { spawnRobot(); });
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Full reset returned success=false.");
                    QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                              Q_ARG(QString, "Status: Reset failed."));
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Full reset exception: %s", e.what());
                QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                          Q_ARG(QString, "Status: Reset error."));
            }
        });
}

bool ResetPanel::discover_rsp_fqn(std::string &fqn_out)
{
    // Scan services for any ".../robot_state_publisher/get_parameters" of type rcl_interfaces/srv/GetParameters
    const auto services = node_->get_service_names_and_types();

    const std::string target_type = "rcl_interfaces/srv/GetParameters";
    std::regex ends_with_get_params(".*/robot_state_publisher/get_parameters$");

    for (const auto & kv : services) {
        const auto & svc_name = kv.first;
        const auto & types    = kv.second;
        if (!std::regex_match(svc_name, ends_with_get_params)) continue;

        // Confirm the type matches
        for (const auto & t : types) {
            if (t == target_type) {
                // service name looks like "/<ns>/robot_state_publisher/get_parameters"
                // derive node FQN by removing "/get_parameters"
                fqn_out = svc_name.substr(0, svc_name.size() - std::string("/get_parameters").size());
                return true;
            }
        }
    }
    return false;
}

bool ResetPanel::fetchRobotDescriptionParam(std::string &xml_out)
{
    // Try to discover fully-qualified node name of robot_state_publisher
    std::string rsp_fqn;
    if (!discover_rsp_fqn(rsp_fqn)) {
        // Fallback: try common names
        for (const auto & name : {std::string("/robot_state_publisher"), std::string("robot_state_publisher")}) {
            auto client = std::make_shared<rclcpp::AsyncParametersClient>(node_, name);
            bool ready = false;
            for (int i = 0; i < 25; ++i) { // up to ~5s
                if (client->wait_for_service(200ms)) { ready = true; break; }
                if (!rclcpp::ok()) return false;
            }
            if (!ready) continue;

            auto future = client->get_parameters({robot_description_param_});
            if (future.wait_for(2s) != std::future_status::ready) continue;

            try {
                auto params = future.get();
                if (!params.empty() && params.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
                    xml_out = params.front().as_string();
                    if (!xml_out.empty()) return true;
                }
            } catch (...) {}
        }
        return false;
    }

    // Query the discovered FQN
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(node_, rsp_fqn);
    bool ready = false;
    for (int i = 0; i < 25; ++i) { // up to ~5s
        if (client->wait_for_service(200ms)) { ready = true; break; }
        if (!rclcpp::ok()) return false;
    }
    if (!ready) return false;

    auto future = client->get_parameters({robot_description_param_});
    if (future.wait_for(2s) != std::future_status::ready) return false;

    try {
        auto params = future.get();
        if (!params.empty() && params.front().get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
            xml_out = params.front().as_string();
            return !xml_out.empty();
        }
    } catch (...) {}
    return false;
}

bool ResetPanel::fetchRobotDescriptionTopic(std::string &xml_out, std::chrono::milliseconds timeout)
{
    // Use a temporary standalone node so we don't touch RViz's executor
    rclcpp::NodeOptions opts;
    opts.use_intra_process_comms(false);
    auto temp_node = std::make_shared<rclcpp::Node>("robot_description_sniffer", opts);

    std::promise<std::string> promise;
    auto future = promise.get_future();

    rclcpp::QoS qos(1);
    qos.transient_local().reliable();

    auto sub = temp_node->create_subscription<std_msgs::msg::String>(
        robot_description_topic_, qos,
        [&promise](const std_msgs::msg::String::SharedPtr msg){
            try { promise.set_value(msg->data); } catch (...) {}
        });

    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(temp_node);
    auto status = exec.spin_until_future_complete(future, timeout);
    exec.remove_node(temp_node);

    if (status == rclcpp::FutureReturnCode::SUCCESS) {
        xml_out = future.get();
        return !xml_out.empty();
    }
    RCLCPP_WARN(node_->get_logger(), "No data received on topic '%s' within %ld ms.",
                robot_description_topic_.c_str(),
                std::chrono::duration_cast<std::chrono::milliseconds>(timeout).count());
    return false;
}

void ResetPanel::spawnRobot()
{
    if (!spawn_client_) {
        RCLCPP_ERROR(node_->get_logger(), "Spawn client not initialized.");
        return;
    }

    auto req = std::make_shared<ros_gz_interfaces::srv::SpawnEntity::Request>();
    auto & ef = req->entity_factory;

    ef.name = robot_name_;
    ef.allow_renaming = false;

    // Preferred: parameter → then topic → then file
    std::string urdf_xml;
    if (fetchRobotDescriptionParam(urdf_xml) || fetchRobotDescriptionTopic(urdf_xml)) {
        ef.sdf = urdf_xml;                 // URDF string accepted; sdformat converts internally
        RCLCPP_INFO(node_->get_logger(), "Spawning '%s' from robot_description (param/topic).", robot_name_.c_str());
    } else if (!model_file_path_.empty()) {
        ef.sdf_filename = model_file_path_; // fallback to file path (.urdf or .sdf)
        RCLCPP_INFO(node_->get_logger(), "Spawning '%s' from file: %s",
                    robot_name_.c_str(), model_file_path_.c_str());
    } else {
        RCLCPP_ERROR(node_->get_logger(), "No robot_description (param/topic) and no model_file_path_ set.");
        QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                  Q_ARG(QString, "Status: Spawn config missing."));
        return;
    }

    // Pose at spawn (matches your launch defaults)
    ef.pose.position.x = spawn_x_;
    ef.pose.position.y = spawn_y_;
    ef.pose.position.z = spawn_z_;
    // Yaw around Z (convert to quaternion)
    const double cy = std::cos(spawn_yaw_ * 0.5);
    const double sy = std::sin(spawn_yaw_ * 0.5);
    ef.pose.orientation.w = cy;
    ef.pose.orientation.x = 0.0;
    ef.pose.orientation.y = 0.0;
    ef.pose.orientation.z = sy;

    auto fut = spawn_client_->async_send_request(
        req,
        [this](rclcpp::Client<ros_gz_interfaces::srv::SpawnEntity>::SharedFuture future)
        {
            try {
                auto resp = future.get();
                if (resp->success) {
                    RCLCPP_INFO(node_->get_logger(), "Robot '%s' spawned successfully.", robot_name_.c_str());
                    
                    // Signal that reset is complete
                    if (reset_state_publisher_) {
                        std_msgs::msg::Bool reset_state_msg;
                        reset_state_msg.data = false;  // false = reset complete, ready for inference
                        reset_state_publisher_->publish(reset_state_msg);
                    }
                    
                    QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                              Q_ARG(QString, "Status: Respawn complete!"));
                    QTimer::singleShot(3000, [this]() {
                        QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                                  Q_ARG(QString, "Status: Ready"));
                    });
                } else {
                    RCLCPP_WARN(node_->get_logger(), "Spawn failed (success=false). "
                                                     "Check bridge /world/%s/create and your model source.",
                                world_name_.c_str());
                    
                    // Also signal reset complete even on failure to avoid hanging
                    if (reset_state_publisher_) {
                        std_msgs::msg::Bool reset_state_msg;
                        reset_state_msg.data = false;  // false = reset process finished (even if failed)
                        reset_state_publisher_->publish(reset_state_msg);
                    }
                    
                    QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                              Q_ARG(QString, "Status: Spawn failed."));
                    QMetaObject::invokeMethod(this, [this]() {
                        QMessageBox::warning(this, "Spawn Robot",
                            "Spawn returned success=false. Verify robot_description or model_file_path_.");
                    }, Qt::QueuedConnection);
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Spawn exception: %s", e.what());
                QMetaObject::invokeMethod(status_label_, "setText", Qt::QueuedConnection,
                                          Q_ARG(QString, "Status: Spawn error."));
                QMetaObject::invokeMethod(this, [this, e]() {
                    QMessageBox::critical(this, "Spawn Error",
                        QString("Service call failed: %1").arg(e.what()));
                }, Qt::QueuedConnection);
            }
        });
}



} // namespace aow_wlr_rviz2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aow_wlr_rviz2::ResetPanel, rviz_common::Panel)
