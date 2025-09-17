#include "aow_controllers/utils/trajectory_utils.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include <fstream>
#include <sstream>
#include <filesystem>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/color_rgba.hpp>

std::vector<TrajectoryPose> TrajectoryUtils::loadTrajectory(
    const std::string& trajectory_file,
    rclcpp::Logger logger)
{
    std::vector<TrajectoryPose> trajectory_poses;
    
    try
    {
        std::string package_share_dir = ament_index_cpp::get_package_share_directory("aow_controllers");
        std::string csv_path = (std::filesystem::path(package_share_dir) / trajectory_file).string();
        
        if (!std::filesystem::exists(csv_path))
        {
            RCLCPP_ERROR(logger, "Trajectory CSV file not found: %s", csv_path.c_str());
            RCLCPP_INFO(logger, "Available trajectory parameter: trajectory_name");
            RCLCPP_INFO(logger, "Current trajectory_name: '%s'", trajectory_file.c_str());
            RCLCPP_INFO(logger, "CSV directory: %s", trajectory_file.c_str());
            return trajectory_poses;
        }
        
        std::ifstream file(csv_path);
        if (!file.is_open())
        {
            RCLCPP_ERROR(logger, "Failed to open CSV trajectory file: %s", csv_path.c_str());
            return trajectory_poses;
        }
        
        std::string line;
        trajectory_poses.clear();
        
        std::getline(file, line);
        
        while (std::getline(file, line))
        {
            std::istringstream iss(line);
            double x, y, z, yaw;
            char comma;
            
            if (iss >> x >> comma >> y >> comma >> z >> comma >> yaw)
            {
                trajectory_poses.push_back(TrajectoryPose{x, y, z, yaw});
            }
        }
        
        file.close();
        
        RCLCPP_INFO(logger, "Successfully loaded CSV trajectory: %s", trajectory_file.c_str());
        RCLCPP_INFO(logger, "Loaded %zu trajectory points", trajectory_poses.size());
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger, "Failed to load trajectory: %s", e.what());
        trajectory_poses.clear();
    }
    
    return trajectory_poses;
}

nav_msgs::msg::Path TrajectoryUtils::createTrajectoryVisualization(
    const std::vector<TrajectoryPose>& trajectory_poses,
    rclcpp::Clock::SharedPtr clock)
{
    nav_msgs::msg::Path path_msg;
    path_msg.header.frame_id = "odometry";
    path_msg.header.stamp = clock->now();
    
    if (!trajectory_poses.empty()) {
        for (const auto& pose : trajectory_poses) {
            geometry_msgs::msg::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = "odometry";
            pose_stamped.header.stamp = clock->now();
            
            pose_stamped.pose.position.x = pose.x;
            pose_stamped.pose.position.y = pose.y;
            pose_stamped.pose.position.z = pose.z;
            
            pose_stamped.pose.orientation.w = cos(pose.yaw / 2.0);
            pose_stamped.pose.orientation.x = 0.0;
            pose_stamped.pose.orientation.y = 0.0;
            pose_stamped.pose.orientation.z = sin(pose.yaw / 2.0);
            
            path_msg.poses.push_back(pose_stamped);
        }
    }
    
    return path_msg;
}

visualization_msgs::msg::Marker TrajectoryUtils::createColorCodedPathMarker(
    double robot_x, 
    double robot_y, 
    double distance_to_closest_waypoint,
    int marker_id,
    rclcpp::Clock::SharedPtr clock)
{
    // Calculate tracking error color (green to red based on distance to trajectory)
    double max_distance = 0.3; // 0.3 meter for full red
    double t = std::min(distance_to_closest_waypoint / max_distance, 1.0);
    t = t * t; // quadratic scaling
    std_msgs::msg::ColorRGBA color;
    color.r = std::min(2.0 * t, 1.0);
    color.g = std::min(2.0 * (1.0 - t), 1.0);
    color.b = 0.0;
    color.a = 1.0;
    
    // Create a sphere marker for the current robot position with color-coded tracking error
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "odometry";
    marker.header.stamp = clock->now();
    marker.ns = "robot_path_colored";
    marker.id = marker_id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    
    // Position at current robot location
    marker.pose.position.x = robot_x;
    marker.pose.position.y = robot_y;
    marker.pose.position.z = 0.1; // Slightly above ground for visibility
    marker.pose.orientation.w = 1.0;
    
    // Size of sphere (small breadcrumb)
    marker.scale.x = 0.15;
    marker.scale.y = 0.15;
    marker.scale.z = 0.15;
    
    // Color based on tracking error
    marker.color = color;
    
    // Lifetime (persist for 30 seconds)
    marker.lifetime = rclcpp::Duration::from_seconds(30.0);
    
    return marker;
}

PTSResult TrajectoryUtils::compute_pts_simple(
    const std::vector<TrajectoryPose>& path_xy,
    const std::vector<double>& t,
    const std::vector<double>& x,
    const std::vector<double>& y,
    bool trajectory_completed,
    const PTSParamsSimple& params)
{
    PTSResult result{};
    
    // Input validation
    if (path_xy.size() < 2) {
        throw std::invalid_argument("path_xy must have at least 2 points");
    }
    if (t.size() != x.size() || t.size() != y.size()) {
        throw std::invalid_argument("t, x, and y must have the same length");
    }
    if (t.empty()) {
        throw std::invalid_argument("Input arrays cannot be empty");
    }
    
    // Check that t is strictly increasing
    for (size_t i = 1; i < t.size(); ++i) {
        if (t[i] <= t[i-1]) {
            throw std::invalid_argument("t must be strictly increasing");
        }
    }
    
    const size_t N = path_xy.size();
    const size_t M = t.size();
    
    // Compute segment vectors and lengths
    std::vector<double> seg_vecs_x(N-1), seg_vecs_y(N-1);
    std::vector<double> seg_lens(N-1);
    std::vector<double> tangents_x(N-1), tangents_y(N-1);
    
    for (size_t i = 0; i < N-1; ++i) {
        seg_vecs_x[i] = path_xy[i+1].x - path_xy[i].x;
        seg_vecs_y[i] = path_xy[i+1].y - path_xy[i].y;
        seg_lens[i] = std::sqrt(seg_vecs_x[i]*seg_vecs_x[i] + seg_vecs_y[i]*seg_vecs_y[i]);
        
        if (seg_lens[i] <= 0) {
            throw std::invalid_argument("Path segments must be non-zero length");
        }
        
        tangents_x[i] = seg_vecs_x[i] / seg_lens[i];
        tangents_y[i] = seg_vecs_y[i] / seg_lens[i];
    }
    
    // Compute cumulative lengths
    std::vector<double> cumlen(N);
    cumlen[0] = 0.0;
    for (size_t i = 1; i < N; ++i) {
        cumlen[i] = cumlen[i-1] + seg_lens[i-1];
    }
    double L = cumlen[N-1];
    result.L = L;
    
    // Project points onto path and compute signed distances
    std::vector<double> s_proj(M);
    std::vector<double> d_signed(M);
    
    for (size_t i = 0; i < M; ++i) {
        double px = x[i];
        double py = y[i];
        
        double min_d2 = std::numeric_limits<double>::max();
        size_t best_j = 0;
        double best_u = 0.0;
        
        // Find closest point on path
        for (size_t j = 0; j < N-1; ++j) {
            // Vector from path point to robot
            double ap_x = px - path_xy[j].x;
            double ap_y = py - path_xy[j].y;
            
            // Project onto segment
            double v_x = seg_vecs_x[j];
            double v_y = seg_vecs_y[j];
            double v2 = seg_lens[j] * seg_lens[j];
            
            double u = std::max(0.0, std::min(1.0, (ap_x * v_x + ap_y * v_y) / v2));
            
            // Closest point on segment
            double qx = path_xy[j].x + u * v_x;
            double qy = path_xy[j].y + u * v_y;
            
            // Distance squared
            double diff_x = px - qx;
            double diff_y = py - qy;
            double d2 = diff_x * diff_x + diff_y * diff_y;
            
            if (d2 < min_d2) {
                min_d2 = d2;
                best_j = j;
                best_u = u;
            }
        }
        
        // Compute s_proj for best segment and ensure it doesn't exceed path length
        double raw_s_proj = cumlen[best_j] + best_u * seg_lens[best_j];
        s_proj[i] = std::min(raw_s_proj, L);  // Clamp to path length
        
        // Debug: Check for floating-point precision issues
        if (raw_s_proj > L) {
            double excess = raw_s_proj - L;
            if (excess > 1e-10) {  // Only warn if excess is significant (not just floating-point error)
                fprintf(stderr, "WARNING: Raw s_proj = %.10f > L = %.10f (excess = %.2e, best_j=%zu, best_u=%.6f)\n", 
                        raw_s_proj, L, excess, best_j, best_u);
            }
        }
        
        // Compute signed distance
        double qx = path_xy[best_j].x + best_u * seg_vecs_x[best_j];
        double qy = path_xy[best_j].y + best_u * seg_vecs_y[best_j];
        double diff_x = px - qx;
        double diff_y = py - qy;
        
        // Cross product for sign (positive = right side of path)
        double cross = tangents_x[best_j] * diff_y - tangents_y[best_j] * diff_x;
        double distance = std::sqrt(diff_x * diff_x + diff_y * diff_y);
        d_signed[i] = (cross >= 0) ? distance : -distance;
    }
    
    // Compute monotonic progress
    std::vector<double> s_mono(M);
    s_mono[0] = s_proj[0];
    for (size_t i = 1; i < M; ++i) {
        s_mono[i] = std::max(s_mono[i-1], s_proj[i]);
    }
    
    // Use explicit completion status to determine s_final
    double s_final;
    if (trajectory_completed) {
        s_final = L;  // If explicitly completed, treat as full path traversal
    } else {
        s_final = std::min(L, s_mono[M-1]);  // Otherwise use projection-based calculation
    }
    result.s_final = s_final;
    result.finished = trajectory_completed;
    
    // Compute finish time
    double T;
    if (result.finished) {
        // Find interpolated finish time
        size_t idx = 0;
        for (size_t i = 0; i < M; ++i) {
            if (s_mono[i] >= L) {
                idx = i;
                break;
            }
        }
        
        double t_finish;
        if (idx == 0) {
            t_finish = t[0];
        } else if (s_mono[idx] == L || idx >= M) {
            t_finish = t[std::min(idx, M-1)];
        } else {
            double s0 = s_mono[idx-1];
            double s1 = s_mono[idx];
            double t0 = t[idx-1];
            double t1 = t[idx];
            double ratio = (L - s0) / std::max(1e-12, (s1 - s0));
            t_finish = t0 + ratio * (t1 - t0);
        }
        T = std::max(1e-9, t_finish - t[0]);
    } else {
        T = std::max(1e-9, t[M-1] - t[0]);
    }
    result.T = T;
    
    // Compute components with safety bounds
    
    // Accuracy component
    double sum_d2 = 0.0;
    for (size_t i = 0; i < M; ++i) {
        double normalized_d = d_signed[i] / params.d0;
        sum_d2 += normalized_d * normalized_d;
    }
    double Rd2 = sum_d2 / M;
    result.Rd2 = Rd2;
    
    // Clamp accuracy component to prevent unrealistic values
    double A_raw = std::exp(-params.w_d * Rd2);
    double A = std::min(1.0, A_raw);  // Should never exceed 1.0 theoretically, but add safety
    if (A_raw > 1.0) {
        // This should never happen - indicates a bug in the calculation
        fprintf(stderr, "WARNING: Accuracy component A_raw = %.6f > 1.0 (Rd2 = %.6f)\n", A_raw, Rd2);
    }
    result.A = A;
    
    // Speed component - add safety bounds
    double T_term_raw = 1.0 / (1.0 + (T / params.tau));
    double T_term = std::min(1.0, T_term_raw);  // Should never exceed 1.0 theoretically
    if (T_term_raw > 1.0) {
        // This should never happen - indicates a bug in the calculation
        fprintf(stderr, "WARNING: Speed component T_term_raw = %.6f > 1.0 (T = %.6f)\n", T_term_raw, T);
    }
    result.T_term = T_term;
    
    // Completion component - ensure s_final/L ratio is bounded
    double completion_ratio = std::min(1.0, s_final / L);  // Cap at 1.0 to prevent overshoot
    if (s_final / L > 1.0) {
        fprintf(stderr, "WARNING: Completion ratio s_final/L = %.6f > 1.0 (s_final = %.3f, L = %.3f)\n", 
                s_final / L, s_final, L);
    }
    double F_term_raw = std::exp(-params.kappa * (1.0 - completion_ratio));
    double F_term = std::min(1.0, F_term_raw);  // Should never exceed 1.0 theoretically
    if (F_term_raw > 1.0) {
        fprintf(stderr, "WARNING: Completion component F_term_raw = %.6f > 1.0 (completion_ratio = %.6f)\n", 
                F_term_raw, completion_ratio);
    }
    result.F_term = F_term;
    
    // Final PTS score with safety bounds
    double pts_raw = 100.0 * std::pow(A, params.alpha) * std::pow(T_term, params.beta) * F_term;
    double pts = std::min(100.0, pts_raw);  // Hard cap at theoretical maximum
    if (pts_raw > 100.0) {
        fprintf(stderr, "WARNING: PTS score pts_raw = %.3f > 100.0 (A=%.3f, T_term=%.3f, F_term=%.3f)\n", 
                pts_raw, A, T_term, F_term);
    }
    result.pts = pts;
    
    return result;
}

double TrajectoryUtils::calculateDistanceToClosestWaypoint(
    const std::vector<TrajectoryPose>& trajectory,
    double robot_x,
    double robot_y,
    int start_index,
    int search_range)
{
    if (trajectory.empty()) {
        return std::numeric_limits<double>::max();
    }
    
    const int N = static_cast<int>(trajectory.size());
    
    // Determine search bounds (forward search only)
    int search_start = std::max(0, start_index);
    int search_end;
    
    if (search_range > 0) {
        search_end = std::min(start_index + search_range, N - 1);
    } else {
        // Search entire trajectory if no range specified
        search_end = N - 1;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    
    for (int i = search_start; i <= search_end; ++i) {
        const double dx = trajectory[i].x - robot_x;
        const double dy = trajectory[i].y - robot_y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
        }
    }
    
    return min_distance;
}

int TrajectoryUtils::findAndUpdateClosestTrajectoryIndex(
    const std::vector<TrajectoryPose>& trajectory,
    double robot_x,
    double robot_y,
    int& last_closest_index,
    int search_range)
{
    if (trajectory.empty()) {
        return 0;
    }
    
    const int N = static_cast<int>(trajectory.size());
    
    // Start search from last known closest index for optimization
    int closest_idx = std::max(0, last_closest_index);
    double min_distance = std::numeric_limits<double>::max();
    
    // Define search window around last closest index
    const int search_start = std::max(0, last_closest_index);
    const int search_end = std::min(last_closest_index + search_range, N - 1);
    
    // Find closest waypoint within search window
    for (int i = search_start; i <= search_end; ++i) {
        const double dx = trajectory[i].x - robot_x;
        const double dy = trajectory[i].y - robot_y;
        const double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    // Ensure forward-only progress (robot can't go backwards along trajectory)
    closest_idx = std::max(closest_idx, last_closest_index);
    
    // Update the last known closest index
    last_closest_index = closest_idx;
    
    return closest_idx;
}