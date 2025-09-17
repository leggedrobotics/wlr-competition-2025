#include "wlr_plugins/wlr_panel/wlr_panel.hpp"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPainter>
#include <QPen>
#include <QBrush>
#include <QTimer>
#include <iostream>
#include <rviz_common/display_context.hpp>
#include <std_msgs/msg/float64.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <algorithm>

namespace aow_wlr_rviz2 {

// DistanceGraphWidget implementation
DistanceGraphWidget::DistanceGraphWidget(QWidget* parent) : QWidget(parent)
{
    setMinimumSize(300, 150);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void DistanceGraphWidget::addDataPoint(double value, std::chrono::steady_clock::time_point timestamp)
{
    data_points_.push_back({value, timestamp});
    
    // Update min/max values for scaling
    if (data_points_.size() == 1) {
        // First data point - initialize both min and max
        max_value_ = value;
        min_value_ = value;
    } else {
        max_value_ = std::max(max_value_, value);
        min_value_ = std::min(min_value_, value);
    }
    
    // Clean old data
    cleanOldData();
    
    // Trigger repaint
    update();
}

void DistanceGraphWidget::cleanOldData()
{
    auto now = std::chrono::steady_clock::now();
    auto cutoff_time = now - std::chrono::duration<double>(time_window_seconds_);
    
    while (!data_points_.empty() && data_points_.front().timestamp < cutoff_time) {
        data_points_.pop_front();
    }
    
    // Recalculate min/max values from remaining data
    if (!data_points_.empty()) {
        auto [min_it, max_it] = std::minmax_element(data_points_.begin(), data_points_.end(),
                                                   [](const DataPoint& a, const DataPoint& b) {
                                                       return a.value < b.value;
                                                   });
        min_value_ = min_it->value;
        max_value_ = max_it->value;
        
        // Ensure we have some range for visualization
        if (max_value_ == min_value_) {
            max_value_ += 0.5;
            min_value_ -= 0.5;
        }
    } else {
        max_value_ = 1.0; // Default scale
        min_value_ = 0.0;
    }
}

void DistanceGraphWidget::paintEvent(QPaintEvent* /* event */)
{
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    
    // Clear background
    painter.fillRect(rect(), QBrush(QColor(240, 240, 240)));
    
    // Draw border
    painter.setPen(QPen(QColor(100, 100, 100), 1));
    painter.drawRect(rect().adjusted(0, 0, -1, -1));
    
    if (data_points_.empty()) {
        painter.setPen(QPen(QColor(150, 150, 150)));
        painter.drawText(rect(), Qt::AlignCenter, "No Data");
        return;
    }
    
    // Clean old data before drawing
    const_cast<DistanceGraphWidget*>(this)->cleanOldData();
    
    if (data_points_.empty()) {
        painter.setPen(QPen(QColor(150, 150, 150)));
        painter.drawText(rect(), Qt::AlignCenter, "No Recent Data");
        return;
    }
    
    // Set up drawing area
    QRect drawRect = rect().adjusted(10, 10, -10, -10);
    auto now = std::chrono::steady_clock::now();
    auto start_time = now - std::chrono::duration<double>(time_window_seconds_);
    
    // Draw grid lines
    painter.setPen(QPen(QColor(200, 200, 200), 1));
    for (int i = 1; i < 5; ++i) {
        int y = drawRect.top() + (drawRect.height() * i) / 5;
        painter.drawLine(drawRect.left(), y, drawRect.right(), y);
    }
    
    // Draw zero line if zero is within the visible range
    if (min_value_ <= 0.0 && max_value_ >= 0.0) {
        painter.setPen(QPen(QColor(100, 100, 100), 2));
        double range = max_value_ - min_value_;
        int zero_y = drawRect.bottom() - static_cast<int>(((0.0 - min_value_) / range) * drawRect.height());
        painter.drawLine(drawRect.left(), zero_y, drawRect.right(), zero_y);
    }
    
    // Draw data line
    painter.setPen(QPen(QColor(0, 100, 200), 2));
    
    for (size_t i = 1; i < data_points_.size(); ++i) {
        // Calculate positions
        auto time1 = std::chrono::duration<double>(data_points_[i-1].timestamp - start_time).count();
        auto time2 = std::chrono::duration<double>(data_points_[i].timestamp - start_time).count();
        
        int x1 = drawRect.left() + static_cast<int>((time1 / time_window_seconds_) * drawRect.width());
        int x2 = drawRect.left() + static_cast<int>((time2 / time_window_seconds_) * drawRect.width());
        
        double range = max_value_ - min_value_;
        int y1 = drawRect.bottom() - static_cast<int>(((data_points_[i-1].value - min_value_) / range) * drawRect.height());
        int y2 = drawRect.bottom() - static_cast<int>(((data_points_[i].value - min_value_) / range) * drawRect.height());
        
        painter.drawLine(x1, y1, x2, y2);
    }
    
    // Draw current value indicator
    if (!data_points_.empty()) {
        painter.setPen(QPen(QColor(200, 0, 0), 3));
        auto last_time = std::chrono::duration<double>(data_points_.back().timestamp - start_time).count();
        int x = drawRect.left() + static_cast<int>((last_time / time_window_seconds_) * drawRect.width());
        double range = max_value_ - min_value_;
        int y = drawRect.bottom() - static_cast<int>(((data_points_.back().value - min_value_) / range) * drawRect.height());
        painter.drawEllipse(x - 3, y - 3, 6, 6);
    }
    
    // Draw scale labels (value axis only)
    painter.setPen(QPen(QColor(50, 50, 50)));
    painter.drawText(drawRect.left() - 35, drawRect.top() + 5, QString::number(max_value_, 'f', 1));
    painter.drawText(drawRect.left() - 35, drawRect.bottom() + 5, QString::number(min_value_, 'f', 1));
}

// AowTrackingGraph implementation
AowTrackingGraph::AowTrackingGraph(QWidget *parent) : Panel(parent) 
{
    QVBoxLayout* layout = new QVBoxLayout;
    
    // Title
    title_label_ = new QLabel("Time Tracking");
    title_label_->setStyleSheet("font-weight: bold; font-size: 14px;");
    layout->addWidget(title_label_);
    
    // Integrated time display
    integrated_time_label_ = new QLabel("Integrated Time: No Data");
    layout->addWidget(integrated_time_label_);
    
    // Off-path time display
    off_path_time_label_ = new QLabel("Off-path Time: No Data");
    layout->addWidget(off_path_time_label_);
    
    // Graph widget
    graph_widget_ = new DistanceGraphWidget();
    layout->addWidget(graph_widget_, 1); // Give it most of the space
    
    setLayout(layout);
}

AowTrackingGraph::~AowTrackingGraph()
{
    unsubscribe();
}

void AowTrackingGraph::onInitialize()
{
    // Get the ROS node from RViz context
    auto node_abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
    if (!node_abstraction) {
        RCLCPP_ERROR(rclcpp::get_logger("AowTrackingGraph"), "Failed to get ROS node abstraction");
        return;
    }
    node_ = node_abstraction->get_raw_node();
    
    // Create topic properties
    integrated_time_topic_property_ = new rviz_common::properties::StringProperty(
        "Integrated Time Topic", "/integrated_time",
        "std_msgs/Float64 topic for integrated time",
        nullptr);
    
    score_topic_property_ = new rviz_common::properties::StringProperty(
        "Score Topic", "/score",
        "std_msgs/Float64 topic for score",
        nullptr);
    
    off_path_time_topic_property_ = new rviz_common::properties::StringProperty(
        "Off-path Time Topic", "/off_path_time",
        "std_msgs/Float64 topic for off-path time",
        nullptr);

    // Connect the properties' changed signals to our updateTopic slot
    connect(integrated_time_topic_property_, &rviz_common::properties::Property::changed,
            this, &AowTrackingGraph::updateTopic);
    connect(score_topic_property_, &rviz_common::properties::Property::changed,
            this, &AowTrackingGraph::updateTopic);
    connect(off_path_time_topic_property_, &rviz_common::properties::Property::changed,
            this, &AowTrackingGraph::updateTopic);
    
    // Subscribe to the topic
    updateTopic();
    
    // Log for debugging
    RCLCPP_INFO(node_->get_logger(), "Time tracking panel initialized and subscribed to: %s, %s, and %s", 
                integrated_time_topic_property_->getStdString().c_str(),
                score_topic_property_->getStdString().c_str(),
                off_path_time_topic_property_->getStdString().c_str());
}

void AowTrackingGraph::updateTopic()
{
    unsubscribe();
    subscribe();
}

void AowTrackingGraph::subscribe()
{
    if (!integrated_time_topic_property_ || !score_topic_property_ || !off_path_time_topic_property_ || !node_) {
        RCLCPP_ERROR(rclcpp::get_logger("AowTrackingGraph"), "Failed to subscribe: missing properties or node");
        return;
    }
    
    const std::string integrated_time_topic = integrated_time_topic_property_->getStdString();
    const std::string score_topic = score_topic_property_->getStdString();
    const std::string off_path_time_topic = off_path_time_topic_property_->getStdString();
    
    if (integrated_time_topic.empty() || score_topic.empty() || off_path_time_topic.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "Empty topic names");
        return;
    }
    
    try {
        integrated_time_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            integrated_time_topic, 10, 
            std::bind(&AowTrackingGraph::integratedTimeCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to integrated time: %s", integrated_time_topic.c_str());
        
        score_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            score_topic, 10, 
            std::bind(&AowTrackingGraph::scoreCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to score: %s", score_topic.c_str());
        
        off_path_time_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            off_path_time_topic, 10, 
            std::bind(&AowTrackingGraph::offPathTimeCallback, this, std::placeholders::_1));
        RCLCPP_INFO(node_->get_logger(), "Subscribed to off-path time: %s", off_path_time_topic.c_str());
        

    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Error subscribing to topics: %s", e.what());
    }
}

void AowTrackingGraph::unsubscribe()
{
    if (integrated_time_sub_) {
        integrated_time_sub_.reset();
    }
    if (off_path_time_sub_) {
        off_path_time_sub_.reset();
    }
    if (score_sub_) {
        score_sub_.reset();
    }
}

void AowTrackingGraph::integratedTimeCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    auto now = std::chrono::steady_clock::now();
    current_integrated_time_ = msg->data;
    
    // Add to history
    time_history_.push_back({current_integrated_time_, now});
    
    // Clean old data (keep 30 seconds of history)
    auto cutoff_time = now - std::chrono::duration<double>(TIME_WINDOW_SECONDS);
    while (!time_history_.empty() && time_history_.front().second < cutoff_time) {
        time_history_.pop_front();
    }
    
    // Update graph with integrated time
    graph_widget_->addDataPoint(current_integrated_time_, now);
    
    // Update display
    updateDisplayValues();
}

void AowTrackingGraph::scoreCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    current_score_ = msg->data;
    
    // Update display
    updateDisplayValues();
}

void AowTrackingGraph::offPathTimeCallback(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
    current_off_path_time_ = msg->data;
    
    // Update display
    updateDisplayValues();
}

void AowTrackingGraph::updateDisplayValues()
{
    // Show the ACTUAL SCORE from /score topic - this is what matters!
    if (std::isinf(current_score_)) {
        integrated_time_label_->setText("🚫 SCORE: ELIMINATED (∞)");
        integrated_time_label_->setStyleSheet("color: red; font-weight: bold; background-color: yellow;");
    } else {
        integrated_time_label_->setText(
            QString("SCORE: %1s").arg(QString::number(current_score_, 'f', 2)));
        integrated_time_label_->setStyleSheet("color: blue; font-weight: bold;");
    }
    
    // Show off-path time with x/5.0 format and elimination status
    if (current_off_path_time_ > 5.0) {
        // Robot eliminated - show in red
        off_path_time_label_->setText(
            QString("🚫 ELIMINATED! Off-path: %1s (>5.0s)").arg(QString::number(current_off_path_time_, 'f', 1)));
        off_path_time_label_->setStyleSheet("color: red; font-weight: bold; background-color: yellow;");
    } else if (current_off_path_time_ > 0.1) {
        // Currently accumulating off-path time - show countdown
        off_path_time_label_->setText(
            QString("⚠️ Off-path: %1/5.0s").arg(QString::number(current_off_path_time_, 'f', 1)));
        
        // Dynamic color based on how close to elimination
        if (current_off_path_time_ > 4.0) {
            off_path_time_label_->setStyleSheet("color: red; font-weight: bold; background-color: yellow;");
        } else if (current_off_path_time_ > 3.0) {
            off_path_time_label_->setStyleSheet("color: red; font-weight: bold;");
        } else {
            off_path_time_label_->setStyleSheet("color: orange; font-weight: bold;");
        }
    } else {
        // On path
        off_path_time_label_->setText("✅ On Path (0.0/5.0s)");
        off_path_time_label_->setStyleSheet("color: green; font-weight: bold;");
    }
}

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(aow_wlr_rviz2::AowTrackingGraph, rviz_common::Panel)

// Include the MOC file for the custom widget
#include "wlr_panel.moc"