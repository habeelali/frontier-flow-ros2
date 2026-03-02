# FrontierFlow (frontier-flow-ros2)

[![ROS 2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue)](https://docs.ros.org/en/humble/index.html)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

**FrontierFlow** is a dynamic, high-performance frontier-based exploration package for ROS 2. Built for efficiency and adaptability, it bridges the gap between SLAM (Cartographer/Slam-Toolbox) and Navigation (Nav2) to enable autonomous mapping of unknown environments.

## ✨ Key Features

*   **Intelligent Scoring**: Prioritizes frontiers based on a weighted combination of cluster size, distance, and a **heading bonus** (preferring areas the robot is already facing).
*   **BFS Clustering**: Efficient 8-connected BFS clustering of frontier cells to identify meaningful exploration targets.
*   **Dynamic Replanning**: Automatically switches goals if a significantly better frontier is discovered, reducing backtracking.
*   **Nav2 Integrated**: Native interaction with the `NavigateToPose` action server.
*   **Manual Blacklisting**: Supports RViz-based "clicked points" to manually seal off areas you want the robot to avoid.
*   **Cartographer Optimized**: Handles `TRANSIENT_LOCAL` map durability for seamless integration with Cartographer's latched maps.

## 🚀 Getting Started

### Prerequisites

*   ROS 2 (Humble or newer)
*   [Nav2 (Navigation 2)](https://navigation.ros.org/)
*   [Cartographer](https://github.com/ros-cartographer/cartographer_ros) or [Slam Toolbox](https://github.com/SteveMacenski/slam_toolbox)

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/yourusername/frontier-flow-ros2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select frontier_explorer
source install/setup.bash
```

### Usage

1.  Launch your robot's SLAM and Navigation stack.
2.  Run the FrontierFlow node:

```bash
ros2 run frontier_explorer frontier_explorer_node
```

Alternatively, use a parameter file:

```bash
ros2 run frontier_explorer frontier_explorer_node --ros-args --params-file src/frontier_explorer/config/frontier_explorer_params.yaml
```

## ⚙️ Configuration

| Parameter | Default | Description |
| :--- | :--- | :--- |
| `map_topic` | `/map` | Topic for the occupancy grid map. |
| `refresh_period_sec` | `1.0` | Frequency (Hz) of frontier detection and goal evaluation. |
| `min_cluster_cells` | `25` | Minimum number of cells for a cluster to be considered a goal. |
| `frontier_size_weight` | `0.0` | Weight for cluster size in scoring (higher = prefer larger areas). |
| `heading_bonus` | `1.5` | Multiplier for frontiers within ±90° of the robot's current heading. |
| `preempt_score_ratio` | `2.0` | Minimum score ratio required to switch goals mid-navigation. |
| `goal_blacklist_sec` | `60.0` | Duration to ignore a failed or reached goal location. |

## 🛠️ Visualization

FrontierFlow publishes high-quality markers for RViz monitoring:

*   **Blue Spheres**: Detected frontier cluster centroids (scaled by size).
*   **Green Cube**: Current active exploration goal.
*   **Red Spheres (Manual)**: Visualized no-go zones added via the `/clicked_point` tool.

## 📄 License

This project is licensed under the Apache License 2.0 - see the `LICENSE` file for details.

---
*Built with ❤️ for the ROS 2 community.*
