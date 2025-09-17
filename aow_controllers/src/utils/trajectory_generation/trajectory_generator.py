# Copyright (c) 2022-2025, The Isaac Lab Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

import matplotlib.pyplot as plt
import numpy as np
import os
from datetime import datetime


def compute_cumulative_distances(path):
    """Compute the cumulative distance along a 2D path."""
    distances = np.sqrt(np.sum(np.diff(path, axis=0) ** 2, axis=1))
    cumulative_distances = np.insert(np.cumsum(distances), 0, 0)
    return cumulative_distances


def compute_tangent_angles(path):
    """Compute the tangent angle (yaw) along the path."""
    dx = np.gradient(path[:, 0])
    dy = np.gradient(path[:, 1])
    tangent_angles = np.arctan2(dy, dx)
    return tangent_angles


def resample_path(path, resolution):
    """Resample the path to have approximately the given resolution."""
    cumulative_distances = compute_cumulative_distances(path)
    total_length = cumulative_distances[-1]
    num_samples = int(total_length / resolution) + 1
    sampled_distances = np.linspace(0, total_length, num_samples)
    resampled_path = np.zeros((num_samples, 2))

    for i in range(2):  # Interpolate both x and y coordinates
        resampled_path[:, i] = np.interp(sampled_distances, cumulative_distances, path[:, i])

    return resampled_path


def generate_rsl_path(resolution=0.01, scale=1.0, num_rounds=1):
    # R - components
    R_vertical = np.array([[0, 0], [0, 2]])  # Vertical line of R
    R_horizontal = np.array([[0, 2], [1.25, 2]])  # Top horizontal line of R
    R_arc = np.array([[1.25 + 0.5 * np.sin(t), 1.5 + 0.5 * np.cos(t)] for t in np.linspace(0, np.pi, 100)])  # Arc for R
    R_short_horizontal = np.array([[1.25, 1], [0.75, 1]])  # Short horizontal line after the arc
    R_diagonal = np.array([[0.75, 1], [1.75, 0]])  # Diagonal leg of R

    # S - components
    S_horizontal_1 = np.array([[0.0, 0], [2.0, 0]])  # Bottom horizontal line of S
    S_arc_1 = np.array(
        [[2.0 + 0.5 * np.sin(t), 0.5 + 0.5 * np.cos(t)] for t in np.linspace(np.pi, 0, 100)]
    )  # Arc for S
    S_horizontal_2 = np.array([[1.5, 1], [1.0, 1]])  # Middle horizontal line of S
    S_arc_2 = np.array(
        [[1.0 - 0.5 * np.sin(t), 1.5 + 0.5 * np.cos(t)] for t in np.linspace(np.pi, 0, 100)]
    )  # Arc for S
    S_horizontal_3 = np.array([[1.0, 2], [3.0, 2]])  # Top horizontal line of S

    # L - components
    L_vertical = np.array([[0, 2], [0, 0]])  # Vertical line of L
    L_horizontal = np.array([[0, 0], [1.5, 0]])  # Horizontal line of L

    # Concatenate the paths to create a continuous shape
    R_path = np.concatenate([R_vertical, R_horizontal, R_arc, R_short_horizontal, R_diagonal])
    S_path = np.concatenate([S_horizontal_1, S_arc_1, S_horizontal_2, S_arc_2, S_horizontal_3])
    L_path = np.concatenate([L_vertical, L_horizontal])
    back_to_R_path = np.array([[6.25, 0], [6.25, -0.05], [0, -0.05]])  # Connect L to R

    # Shift the S and L to connect them smoothly to R
    S_path[:, 0] += 1.75  # Shift S to the right
    L_path[:, 0] += 4.75  # Shift L to the right

    # Combine all paths
    full_path = np.concatenate([R_path, S_path, L_path, back_to_R_path])

    # Scale the path
    full_path *= scale

    # Resample the path to have the desired resolution
    resampled_path = resample_path(full_path, resolution)

    # Compute tangent angles and cumulative distances
    tangent_angles = compute_tangent_angles(resampled_path)  # Shape: (num_points,)
    cumulative_distances = compute_cumulative_distances(resampled_path)  # Shape: (num_points,)

    # Combine into a (num_points, 4) array
    extended_path = np.hstack((resampled_path, tangent_angles[:, np.newaxis], cumulative_distances[:, np.newaxis]))

    extended_path_ori = np.copy(extended_path)
    # Repeat the path for the specified number of rounds
    for _ in range(num_rounds - 1):
        now_distance = extended_path[-1, 3]
        i_round = np.copy(extended_path_ori)
        # the cumulative distance is shifted by the total length of the path
        i_round[:, 3] += now_distance
        extended_path = np.concatenate((extended_path, i_round))
    return extended_path


def save_trajectory_as_csv(trajectory_data, name="trajectory", duration=10.0, frequency=50.0, output_path=None):
    """
    Save trajectory data as simple CSV file for easy parsing in C++.
    Format: x,y,z,yaw,timestamp
    
    Args:
        trajectory_data: numpy array with shape (n, 4) containing [x, y, yaw, cumulative_distance]
        name: name for the trajectory file
        duration: total duration of trajectory in seconds
        frequency: control frequency in Hz
        output_path: directory to save the file (default: same directory as script)
    
    Returns:
        str: filepath of the saved CSV file
    """
    if output_path is None:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_path = os.path.join(script_dir, "example_trajectory_csv")
    
    # Create output directory if it doesn't exist
    os.makedirs(output_path, exist_ok=True)
    
    # Extract components
    num_points = trajectory_data.shape[0]
    x_positions = trajectory_data[:, 0]
    y_positions = trajectory_data[:, 1]
    yaw_angles = trajectory_data[:, 2]
    
    # Generate z positions (assume flat ground)
    z_positions = np.full(num_points, 0.5)  # 0.5m height for robot base
    
    # Generate timestamps
    timestamps = np.linspace(0, duration, num_points)
    
    # Save CSV file
    filename = f"{name}.csv"
    filepath = os.path.join(output_path, filename)
    
    # Create CSV data
    csv_data = np.column_stack([x_positions, y_positions, z_positions, yaw_angles, timestamps])
    
    # Save with header
    header = "x,y,z,yaw,timestamp"
    np.savetxt(filepath, csv_data, delimiter=',', header=header, comments='', fmt='%.6f')
    
    print(f"CSV trajectory saved to: {filepath}")
    print(f"Points: {num_points}, Duration: {duration:.2f}s")
    
    return filepath


if __name__ == "__main__":
    # Generate the RSL path
    resolution = 0.01  # meters between points
    scale = 5.0        # scale factor
    num_rounds = 1     # how many times to repeat the path
    
    # Generate path
    path = generate_rsl_path(resolution=resolution, scale=scale, num_rounds=num_rounds)
    
    # Calculate trajectory parameters
    total_distance = path[-1, 3]  # cumulative distance
    desired_speed = 1.0  # m/s average speed
    duration = total_distance / desired_speed
    frequency = 1.0 / resolution  # approximate frequency based on resolution and speed
    
    print(f"Generated RSL trajectory:")
    print(f"   Resolution: {resolution}m")
    print(f"   Scale: {scale}x")
    print(f"   Rounds: {num_rounds}")
    print(f"   Total distance: {total_distance:.2f}m")
    print(f"   Duration: {duration:.2f}s")
    print(f"   Frequency: {frequency:.1f}Hz")
    
    # Save as CSV (simple and easy to parse in C++)
    trajectory_name = f"rsl_path_scale{scale}_rounds{num_rounds}"
    csv_filepath = save_trajectory_as_csv(
        trajectory_data=path,
        name=trajectory_name,
        duration=duration,
        frequency=frequency
    )
    
    # Plot the trajectory
    plt.figure(figsize=(12, 8))
    
    # Plot path
    plt.subplot(2, 2, 1)
    plt.plot(path[:, 0], path[:, 1], "b-", linewidth=2)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.title("RSL Path (X-Y)")
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.grid(True)
    
    # Plot yaw angle over time
    plt.subplot(2, 2, 2)
    plt.plot(path[:, 3], np.rad2deg(path[:, 2]), "r-", linewidth=2)
    plt.title("Yaw Angle vs Distance")
    plt.xlabel("Cumulative Distance [m]")
    plt.ylabel("Yaw [deg]")
    plt.grid(True)
    
    # Plot speed profile (approximate)
    plt.subplot(2, 2, 3)
    speeds = np.gradient(path[:, 3]) / resolution * desired_speed
    plt.plot(path[:, 3], speeds, "g-", linewidth=2)
    plt.title("Speed Profile")
    plt.xlabel("Cumulative Distance [m]")
    plt.ylabel("Speed [m/s]")
    plt.grid(True)
    
    # Plot trajectory info
    plt.subplot(2, 2, 4)
    plt.text(0.1, 0.8, f"Trajectory: {trajectory_name}", fontsize=12, fontweight='bold')
    plt.text(0.1, 0.7, f"Points: {path.shape[0]}", fontsize=10)
    plt.text(0.1, 0.6, f"Distance: {total_distance:.2f} m", fontsize=10)
    plt.text(0.1, 0.5, f"Duration: {duration:.2f} s", fontsize=10)
    plt.text(0.1, 0.4, f"Avg Speed: {desired_speed:.2f} m/s", fontsize=10)
    plt.text(0.1, 0.3, f"File: {os.path.basename(csv_filepath)}", fontsize=10)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.axis('off')
    plt.title("Trajectory Info")
    
    plt.tight_layout()
    plt.show()
    
    print(f"Path shape: {path.shape}")
    print(f"CSV file created: {csv_filepath}")
    print(f"Simple CSV format - easy to parse in C++!")