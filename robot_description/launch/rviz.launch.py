import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)

    urdf_file = 'robot.urdf'
    robot_desc_path = os.path.join(package_directory, "urdf", urdf_file)
    print("URDF Loaded !")

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name="robot_state_publisher_node",
        output="screen",
        emulate_tty=True,
        parameters=[{'use_sim_time': True,'robot_description': Command(['xacro ',robot_desc_path])}]
    )
    rviz_config_file="config.rviz"
    rviz_config_path=os.path.join(package_directory, "rviz", rviz_config_file)
    print('Rviz config loaded!')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'use_sim_time': True}],
        arguments=['d', rviz_config_path]
    )
    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    
    
    ]
    )