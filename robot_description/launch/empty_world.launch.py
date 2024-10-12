import os
from ament_index_python.packages import (get_package_prefix, get_package_share_directory)
from launch import LaunchDescription
from launch_ros.actions import SetParameter
from launch.substitutions import (PathJoinSubstitution, LaunchConfiguration)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription)
def generate_launch_description():

    package_description = "robot_description"
    package_directory = get_package_share_directory(package_description)


    install_dir_path = (get_package_prefix(package_description) + "/share")
    robot_meshes_path = os.path.join(package_directory, "meshes")
    gazebo_resource_paths = [install_dir_path, robot_meshes_path]
    if "IGN_GAZEBO_RESOURCE_PATH" in os.environ:
        for resource_path in gazebo_resource_paths:
            if resource_path not in os.environ["IGN_GAZEBO_RESOURCE_PATH"]:
                os.environ["IGN_GAZEBO_RESOURCE_PATH"] += (':' + resource_path)
    else:
        os.environ["IGN_GAZEBO_RESOURCE_PATH"] = (':'.join(gazebo_resource_paths))
    world_file = "empty.sdf"
    world_config = LaunchConfiguration("world")
    declare_world_arg = DeclareLaunchArgument("world", default_value=["-r ", world_file], description="SDF World File")

    gzsim_pkg = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gzsim_pkg, "launch", "gz_sim.launch.py"])),
            launch_arguments={"gz_args": world_config}.items(),    
    )

    return LaunchDescription([
        declare_world_arg,
        SetParameter(name="use_sim_time", value=True),    
        gz_sim,
    ])

    

