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
    class DWA {
        +DynamicWindowApproach(x, model, goal, evalParam, ob, R, robotR) : vector<double>
        -CalcDynamicWindow(x, model) : array<double, 4>
        -GenerateTrajectory(x, vt, ot, evaldt) : array<double, 5>
        -CalcHeadingEval(x, goal) : double
        -CalcDistEval(x, ob, R, robotR) : double
        -NormalizeEval(evalDB) : void
        -SelectBestControl(evalDB, evalParam) : vector<double>
    }
    class DWAPlannerNode {
        +DWAPlannerNode()
        +~DWAPlannerNode()
        -timerCallback() : void
        -odomCallback(msg) : void
        -local_obstacle_callback(msg) : void
        -target_callback(msg) : void
        -send_static_transform() : void
        -odom_sub_ : Subscription<nav_msgs::msg::Odometry>
        -local_obstacle_sub_ : Subscription<visualization_msgs::msg::MarkerArray>
        -target_sub_ : Subscription<geometry_msgs::msg::PoseStamped>
        -cmd_vel_pub_ : Publisher<geometry_msgs::msg::Twist>
        -timer_ : TimerBase
        -static_broadcaster_ : StaticTransformBroadcaster
        -x_ : array<double, 5>
        -goal_ : array<double, 2>
        -obstacle_ : vector<array<double, 2>>
        -kinematic_ : array<double, 6>
        -eval_param_ : array<double, 4>
        -robot_radius_ : double
        -obstacle_radius_ : double
        -received_obstacles_ : bool
        -received_goal_ : bool
        -received_odom_ : bool
    }
    class rclcppNode {
        <<library>>
    }

    DWAPlannerNode o-- DWA : Uses
    DWAPlannerNode <|-- rclcppNode : Extends
```

### Flowchart

```mermaid
flowchart TD
    subgraph DWAPlannerNode
        A[Start] --> B[Initialize ROS 2 Node: dwa_planner]
        B --> C[Create Publishers and Subscribers]
        K[odomCallback: Update Robot State]
        L[local_obstacle_callback: Update Obstacles]
        M[target_callback: Update Target Goal]
        N[timerCallback: Loop Update Control]
        N --> E[Check if Data is Received]
        E -->|No| F[Wait for Required Topics]
        E -->|Yes| G[Compute Control Using DWA]
        V[publishCmd: Publish Velocity Command]
        I[End Timer Callback]
        F --> N
    end

    subgraph DWA
        O[DynamicWindowApproach: DWA Computation]
        P[CalcDynamicWindow: Calculate Velocity and Angular Range]
        Q[GenerateTrajectory: Generate Possible Trajectories]
        R[CalcHeadingEval: Evaluate Heading]
        S[CalcDistEval: Evaluate Distance to Obstacles]
        T[NormalizeEval: Normalize Evaluations]
        U[SelectBestControl: Choose Best Control]
    end

    C --> K
    C --> L
    C --> M
    K --> N
    L --> N
    M --> N
    G --> O
    O --> P
    P --> Q
    Q --> R
    R --> S
    S --> T
    T --> U
    U --> V
    V --> I
```
