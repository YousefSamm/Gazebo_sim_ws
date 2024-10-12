import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    package_description = 'robot_description'
    package_directory = get_package_share_directory(package_description)

    urdf_file = 'robot.xacro'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded!")

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        emulate_tty=True,
        output="screen",
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}] 
    )

    # Declare launch arguments for the first robot
    declare_spawn_x = DeclareLaunchArgument("x1", default_value="0.0", description="Model 1 Spawn X Axis Value")
    declare_spawn_y = DeclareLaunchArgument("y1", default_value="0.0", description="Model 1 Spawn Y Axis Value")
    declare_spawn_z = DeclareLaunchArgument("z1", default_value="0.5", description="Model 1 Spawn Z Axis Value")

    # Declare launch arguments for the second robot
  
    # First robot spawn node
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn",
        arguments=[
            "-name", "my_robot",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x1"), 
            "-y", LaunchConfiguration("y1"),
            "-z", LaunchConfiguration("z1"),
        ],
        output="screen",
    )

    # Second robot spawn node
    gz_spawn_entity_2 = Node(
        package="ros_gz_sim",
        executable="create",
        name="my_robot_spawn_2",
        arguments=[
            "-name", "my_robot_2",
            "-allow_renaming", "true",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x2"),  
            "-y", LaunchConfiguration("y2"),
            "-z", LaunchConfiguration("z2"),
        ],
        output="screen",
    )
    # ROS-Gazebo Bridge #
    ign_bridge = Node(
    package="ros_gz_bridge",
    executable="parameter_bridge",
    name="ign_bridge",
    arguments=[
        "/clock" + "@rosgraph_msgs/msg/Clock" + "[ignition.msgs.Clock",
        "/cmd_vel" + "@geometry_msgs/msg/Twist" + "@ignition.msgs.Twist",
        "/tf" + "@tf2_msgs/msg/TFMessage" + "[ignition.msgs.Pose_V",
        "/odom" + "@nav_msgs/msg/Odometry" + "[ignition.msgs.Odometry",
        "/laser/scan" + "@sensor_msgs/msg/LaserScan" + "[ignition.msgs.LaserScan",
        "/imu" + "@sensor_msgs/msg/Imu" + "[ignition.msgs.IMU",
    ],
    remappings=[
        # there are no remappings for this robot description
    ],
    output="screen",
)


    return LaunchDescription([
        robot_state_publisher_node,
        declare_spawn_x,
        declare_spawn_y,
        declare_spawn_z,
        gz_spawn_entity,
        ign_bridge
    ])
