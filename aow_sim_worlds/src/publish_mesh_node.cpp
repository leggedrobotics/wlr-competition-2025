#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class MeshPublisher : public rclcpp::Node
{
public:
  MeshPublisher()
  : Node("mesh_publisher")
  {
    publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("world_mesh", 10);

    // Marker metadata
    marker_.header.frame_id = "odometry";
    marker_.ns           = "world_mesh";
    marker_.id           = 0;
    marker_.type         = visualization_msgs::msg::Marker::MESH_RESOURCE;
    marker_.action       = visualization_msgs::msg::Marker::ADD;
    marker_.mesh_resource =
      "package://aow_sim_worlds/models/COLOSSEUM/meshes/Colosseum_final.stl";

    // Pose
    marker_.pose.position.x = 12.0;
    marker_.pose.position.y = 5.0;
    marker_.pose.position.z = 0.0;
    marker_.pose.orientation.x = 0.0;
    marker_.pose.orientation.y = 0.0;
    marker_.pose.orientation.z = 0.0;
    marker_.pose.orientation.w = 1.0;

    // Scale
    marker_.scale.x = 1.0;
    marker_.scale.y = 1.0;
    marker_.scale.z = 1.0;

    // Color (opaque white)
    marker_.color.r = 1.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 1.0f;
    marker_.color.a = 1.0f;

    // Lifetime: forever
    marker_.lifetime = rclcpp::Duration::from_seconds(0);

    // Timer to publish periodically
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MeshPublisher::timer_callback, this)
    );
  }

private:
  void timer_callback()
  {
    marker_.header.stamp = this->now();
    publisher_->publish(marker_);
  }

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
  visualization_msgs::msg::Marker marker_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MeshPublisher>());
  rclcpp::shutdown();
  return 0;
}