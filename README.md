<p style="display: inline">
  <!-- Programming Language -->
  <img src="https://img.shields.io/badge/-C++-00599C.svg?logo=c%2B%2B&style=for-the-badge">
  <!-- ROS 2 -->
  <img src="https://img.shields.io/badge/-ROS%202-22314E.svg?logo=ros&style=for-the-badge&logoColor=white">
</p>

## Functional Overview
This software implements a ROS 2 node for collision avoidance control of robots or autonomous vehicles using the Dynamic Window Approach. It calculates and outputs velocity and angular velocity commands to efficiently navigate the robot.

![dwa_planner](https://github.com/user-attachments/assets/b1033045-13b0-4d1d-ad8f-8d4ee20f1b80)

## Requirements
### System Requirements
- OS : Ubuntu 22.04  
- ROS2 : Humble

### System Dependencies
- [arcanain_simulator](https://github.com/Arcanain/arcanain_simulator) 

## How To Use
### Execution Steps
```bash
cd ~/ros2_ws
source ~/ros2_ws/install/setup.bash
ros2 launch dwa_planner dwa_planner.py
```

### Folder Structure
```
dwa_planner/
├── include/                               - Directory for header files
│   └── pure_pursuit_planner/              - Header files for the pure_pursuit_planner package
│       └── pure_pursuit_planner_component.hpp - Header file for the Pure Pursuit planner component
├── launch/                                - Directory for ROS 2 launch files
│   └── dwa_planner.py            - Launch script for the pure_pursuit_planner node
├── rviz/                                  - Directory for RViz configuration files
│   └── dwa_planner.rviz          - RViz configuration file for the Pure Pursuit planner
├── src/                                   - Directory for source files
│   ├── dwa_noe.cpp - Implementation of the Pure Pursuit planner component
│   └── dwa_planner_node.cpp      - Node implementation for the Pure Pursuit planner
├── CMakeLists.txt                         - CMake build configuration file
└── package.xml                            - ROS 2 package metadata file
```

## Interface Table

### Input

| Variable Name      | Type            | Description                         |
|-------------------------|-------------------|---------------------------------------|
| `odom`                  | `nav_msgs::msg::Odometry` | Odometry information of the robot |
| `tgt_path`              | `nav_msgs::msg::Path` | Target trajectory of the robot |

### Output

| Variable Name      | Type            | Description                         |
|-------------------------|-------------------|---------------------------------------|
| `cmd_vel`               | `geometry_msgs::msg::Twist` | Velocity and angular velocity commands for the robot |

### Internal Values

| Variable Name      | Type            | Description                         |
|-------------------------|-------------------|---------------------------------------|
| `x`, `y`, `yaw`         | `double`          | Current position and orientation of the robot |
| `v`, `w`                | `double`          | Velocity and angular velocity of the robot |
| `cx`, `cy`,`cyaw`, `ck` | `std::vector<double>` | List of x and y coordinates of the path |
| `target_ind`            | `int`             | Current target index |
| `target_vel`            | `double`          | Target velocity |
| `goal_threshold`        | `double`          | Threshold for goal judgment |
| `k`, `Lfc`, `Kp`, `dt`  | `double`          | Pure Pursuit parameters |
| `oldNearestPointIndex`  | `int`             | Index of the nearest point in the previous iteration |
| `current_vel`           | `double`          | Current velocity of the robot |
| `minCurvature`,`maxCurvature`         | `double`          | Minimum and maximum curvature values |
| `minVelocity`,`maxVelocity`           | `double`          | Minimum and maximum velocity values |

## Software architecture

### Class Diagram

```mermaid
classDiagram
    class PurePursuitNode {
        +PurePursuitNode()
        -void updateControl()
        -std::pair<double, double> purePursuitControl(int&)
        -std::pair<int, double> searchTargetIndex()
        -double calcDistance(double, double) const
        -void odometry_callback(nav_msgs::msg::Odometry::SharedPtr)
        -void path_callback(nav_msgs::msg::Path::SharedPtr)
        -void publishCmd(double, double)
        -rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub
        -rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub
        -rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub
        -rclcpp::TimerBase::SharedPtr timer
        -std::vector<double> cx
        -std::vector<double> cy
        -std::vector<double> cyaw
        -std::vector<double> ck
        -double x, y, yaw, v, w
        -int target_ind
        -int oldNearestPointIndex
        -double target_vel
        -double current_vel
        -bool path_subscribe_flag
        -double goal_threshold
        -const double k
        -const double Lfc
        -const double Kp
        -const double dt
        -double minCurvature
        -double maxCurvature
        -double minVelocity
        -double maxVelocity
    }
```

### Flowchart

```mermaid
flowchart TD
    A[Start] --> B[Initialize ROS 2 Node: pure_pursuit_planner]
    B --> C[Create Publishers and Subscribers]
    C --> D[Enter Timer Callback Loop]
    D --> E[Update Control]
    E --> F[Pure Pursuit Control]
    F --> G[Publish Command Velocity]
    G --> H[Check if Goal is Reached]
    H -->|No| I[Continue Path Tracking]
    H -->|Yes| J[Stop the Robot]
    I --> D
    J --> K[End]

    subgraph PurePursuitNode
        L[Constructor: Initialize Node, Topics, and Parameters]
        M[odometry_callback: Update Robot State]
        N[path_callback: Receive and Process Path]
        O[updateControl: Timer Callback for Control Update]
        P[purePursuitControl: Calculate Steering and Velocity]
        Q[publishCmd: Publish Velocity Command to cmd_vel]
    end

    L --> M
    L --> N
    M --> O
    N -.-> O
    O --> P
    P --> Q

```

## System architecture

```mermaid
classDiagram
    class PathPublisher {
        +PathPublisher()
        +void loadPathData(string&)
        +void publishPath()
        -rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_
        -rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr visualize_path_pub_
        -rclcpp::TimerBase::SharedPtr timer_
        -nav_msgs::msg::Path path_
        -nav_msgs::msg::Path visualize_path_
    }
    
    class PurePursuitNode {
        +PurePursuitNode()
        -void updateControl()
        -std::pair<double, double> purePursuitControl(int&)
        -std::pair<int, double> searchTargetIndex()
        -double calcDistance(double, double) const
        -void odometry_callback(nav_msgs::msg::Odometry::SharedPtr)
        -void path_callback(nav_msgs::msg::Path::SharedPtr)
        -void publishCmd(double, double)
        -rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub
        -rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub
        -rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub
        -rclcpp::TimerBase::SharedPtr timer
        -std::vector<double> cx
        -std::vector<double> cy
        -std::vector<double> cyaw
        -std::vector<double> ck
        -double x, y, yaw, v, w
        -int target_ind
        -int oldNearestPointIndex
        -double target_vel
        -double current_vel
        -bool path_subscribe_flag
        -double goal_threshold
        -const double k
        -const double Lfc
        -const double Kp
        -const double dt
        -double minCurvature
        -double maxCurvature
        -double minVelocity
        -double maxVelocity
    }

    PathPublisher --|> PurePursuitNode: path_

    class OdometryPublisher {
        -rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub
        -rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub
        -rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr localmap_pub
        -rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr laser_range_pub
        -rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber
        -std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster
        -std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_
        -rclcpp::TimerBase::SharedPtr timer_
        -nav_msgs::msg::Path path
        -double x, y, th, vx, vth
        -rclcpp::Time current_time, last_time
        +OdometryPublisher()
        -void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr)
        -void timer_callback()
        -void send_static_transform()
    }

    PurePursuitNode --|> OdometryPublisher: v, w
    OdometryPublisher --|> PurePursuitNode: x, y, th
```
