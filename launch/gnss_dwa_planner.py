import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_name = 'dwa_planner'
    simulator_package = 'arcanain_simulator'
    rviz_file_name = "dwa_planner.rviz"
    odrive_package = 'odrive_ros2_control'
    judge_package = 'rtk_judge'
    lidar_dir = get_package_share_directory('sllidar_ros2')
    filter_dir = get_package_share_directory('lidar_based_obstacle_detection')

    # sllidar_node + base_link->laser の static TF
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'sllidar_s2_launch.py')
        )
    )

    # /scan -> /filtered_scan（lidar_based_obstacle_detection_launch.py は rviz2 も起動するので Node を直接書く）
    obstacle_detection_node = Node(
        package='lidar_based_obstacle_detection',
        executable='lidar_based_obstacle_detection_node',
        name='lidar_based_obstacle_detection',
        parameters=[os.path.join(filter_dir, 'config', 'params.yaml')],
        output='screen',
    )


    # waypoint_publisher の既定値と同じ CSV
    default_path_csv = os.path.expanduser(
            '~/ros2_ws/src/path_smoother/path/tsukuba_1004.csv'
    )
    path_csv_arg = DeclareLaunchArgument(
            'path_csv',
            default_value=default_path_csv,
            description='Path to the CSV file for waypoint_publisher (tgt_path / waypoint)',
        )

    dummy_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'map', 'dummy_link']
    )

    odrive_ros2_control_node = Node(
        package=odrive_package,
        executable='control_odrive_and_odom_pub_gps',
        output="screen",
    )

    gnss_node = Node(
        package='gnss_preprocessing',
        executable='gnss_preprocessing',
        output='screen'
    )

    rtk_judge_node = Node(
        package=judge_package,
        executable='judge_rtk_status_origin',
        output="screen",
    )

    dwa_params = PathJoinSubstitution(
        [FindPackageShare(package_name), 'config', 'params.yaml']
    )

    file_path = os.path.expanduser('~/ros2_ws/src/arcanain_simulator/urdf/mobile_robot.urdf.xml')

    with open(file_path, 'r') as file:
        robot_description = file.read()

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", rviz_file_name]
    )

    

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    robot_description_rviz_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    joint_state_publisher_rviz_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description}]
    )

    dwa_planner_node = Node(
        package=package_name,
        executable='dwa_planner',
        output="screen",
        parameters=[dwa_params],
    )

    odometry_pub_node = Node(
        package=simulator_package,
        executable='odrive_gps_switch_pub',
        output="screen",
    )

    obstacle_pub_node = Node(
        package=simulator_package,
        executable='obstacle_pub',
        output="screen",
    )

    waypoint_pub_node = Node(
        package=simulator_package,
        executable='waypoint_pub',
        output="screen",
    )

    lidar_waypoint_pub_node = Node(
        package='path_smoother',
        executable='waypoint_publisher',
        output="screen",
        parameters=[{'path_file': LaunchConfiguration('path_csv')}],
    )

    nodes = [
        path_csv_arg,
        dummy_node,
        lidar_launch,
        obstacle_detection_node,
        rviz_node,
        robot_description_rviz_node,
        joint_state_publisher_rviz_node,
        gnss_node,
        odrive_ros2_control_node,
        rtk_judge_node,
        dwa_planner_node,
        odometry_pub_node,
        lidar_waypoint_pub_node,
    ]

    return LaunchDescription(nodes)