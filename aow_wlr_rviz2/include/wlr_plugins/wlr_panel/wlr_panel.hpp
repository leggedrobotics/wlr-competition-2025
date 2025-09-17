#ifndef SMB_BATT_MOT_PANEL_HPP__
#define SMB_BATT_MOT_PANEL_HPP__

#include <rviz_common/panel.hpp>
#include <rviz_common/properties/string_property.hpp>

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QTimer>
#include <QPaintEvent>
#include <QPainter>
#include <QBrush>
#include <QPen>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>
#include <chrono>

namespace aow_wlr_rviz2
{

// Custom widget for drawing the distance graph
class DistanceGraphWidget : public QWidget
{
    Q_OBJECT
    
public:
    DistanceGraphWidget(QWidget* parent = nullptr);
    void addDataPoint(double value, std::chrono::steady_clock::time_point timestamp);
    void setTimeWindow(double seconds) { time_window_seconds_ = seconds; }
    
protected:
    void paintEvent(QPaintEvent* event) override;
    
private:
    struct DataPoint {
        double value;
        std::chrono::steady_clock::time_point timestamp;
    };
    
    std::deque<DataPoint> data_points_;
    double time_window_seconds_{30.0}; // Increased from 10 to 30 seconds for better score tracking
    double max_value_{1.0};
    double min_value_{0.0}; // Track minimum value for proper scaling with negative scores
    void cleanOldData();
};

class AowTrackingGraph : public rviz_common::Panel
{
    Q_OBJECT

    public:
        AowTrackingGraph(QWidget *parent = nullptr);
        ~AowTrackingGraph();

    private Q_SLOTS:
        void updateTopic();

    private:
        void integratedTimeCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
        void scoreCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
        void offPathTimeCallback(const std_msgs::msg::Float64::ConstSharedPtr msg);
        void onInitialize() override;
        void subscribe();
        void unsubscribe();
        void updateDisplayValues();

        // Data storage for time-based metrics
        std::deque<std::pair<double, std::chrono::steady_clock::time_point>> time_history_; // Integrated time history
        double current_integrated_time_{0.0}; // Current integrated time
        double current_score_{0.0}; // Current score (can be infinite if eliminated)
        double current_off_path_time_{0.0}; // Current off-path time
        
                // UI elements
        QLabel* title_label_;
        QLabel* integrated_time_label_;
        QLabel* off_path_time_label_;
        DistanceGraphWidget* graph_widget_;

        // ROS components
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr integrated_time_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr score_sub_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr off_path_time_sub_;
        rclcpp::Node::SharedPtr node_;
        rviz_common::properties::StringProperty* integrated_time_topic_property_{nullptr};
        rviz_common::properties::StringProperty* score_topic_property_{nullptr};
        rviz_common::properties::StringProperty* off_path_time_topic_property_{nullptr};
        
        // Time window for display (30 seconds for better score tracking)
        static constexpr double TIME_WINDOW_SECONDS = 30.0;
    };

} // namespace aow_wlr_rviz2
#endif //MB_BATT_MOT_PANEL_HPP 