#ifndef TRAJECTORY_UTILS_HPP
#define TRAJECTORY_UTILS_HPP

#include <vector>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

struct TrajectoryPose {
    double x, y, z;
    double yaw;
};

struct PTSParamsSimple {
    double d0 = 0.05;       // Reference distance (5cm)
    double w_d = 1.0;       // Weight for distance penalty
    double tau = 20.0;      // Reference time (20 seconds)
    double kappa = 2.0;     // Completion penalty steepness
    double alpha = 1.0;     // Accuracy component weight
    double beta = 1.0;      // Speed component weight
};

struct PTSResult {
    double pts;             // Final PTS score
    double A;               // Accuracy component
    double T_term;          // Speed component
    double F_term;          // Completion component
    bool finished;          // Did robot finish the path?
    double s_final;         // Final distance traveled along path
    double T;               // Total time taken
    double Rd2;             // Normalized squared distance
    double L;               // Total path length
};

class TrajectoryUtils {
public:
    static std::vector<TrajectoryPose> loadTrajectory(
        const std::string& trajectory_file,
        rclcpp::Logger logger
    );
    
    static nav_msgs::msg::Path createTrajectoryVisualization(
        const std::vector<TrajectoryPose>& trajectory_poses,
        rclcpp::Clock::SharedPtr clock
    );
    
    static visualization_msgs::msg::Marker createColorCodedPathMarker(
        double robot_x, 
        double robot_y, 
        double distance_to_closest_waypoint,
        int marker_id,
        rclcpp::Clock::SharedPtr clock
    );
    
    static PTSResult compute_pts_simple(
        const std::vector<TrajectoryPose>& path_xy,
        const std::vector<double>& t,
        const std::vector<double>& x,
        const std::vector<double>& y,
        bool trajectory_completed = false,
        const PTSParamsSimple& params = PTSParamsSimple()
    );
    
    // Calculate distance from robot position to closest waypoint on trajectory
    static double calculateDistanceToClosestWaypoint(
        const std::vector<TrajectoryPose>& trajectory,
        double robot_x,
        double robot_y,
        int start_index = 0,
        int search_range = -1  // -1 means search entire trajectory
    );
    
    // Find the closest trajectory index and update the last known index (with forward-only constraint)
    static int findAndUpdateClosestTrajectoryIndex(
        const std::vector<TrajectoryPose>& trajectory,
        double robot_x,
        double robot_y,
        int& last_closest_index,
        int search_range = 100  // Default search range for optimization
    );
};

#endif // TRAJECTORY_UTILS_HPP