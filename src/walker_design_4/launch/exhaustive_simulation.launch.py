# NOTE: colcon build --packages-select walker_design_4 && source ~/.bashrc
# NOTE: ros2 launch walker_design_4 exhaustive_simulation.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description(): 
    # Set the path to this package.
    # Change this when used as a template
    pkg_share = FindPackageShare(package='walker_design_4').find('walker_design_4')

    # launch arguments
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    world = LaunchConfiguration('world')
    use_simulator = LaunchConfiguration('use_simulator')

    # Set the path to the world file
    world_file_name = 'ramp.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    world_arg = DeclareLaunchArgument(
        name='world',
        default_value=world_path,
        description='Full path to the world model file to load'
    )

    use_simulator_arg = DeclareLaunchArgument(
        name='use_simulator',
        default_value='True',
        description='Whether to start the simulator'
    )

    start_gazebo_server_cmd = ExecuteProcess(
        condition=IfCondition(use_simulator),
        cmd=[
            'gazebo', '--verbose',
            '-e', 'bullet',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '-s', 'libgazebo_ros_force_system.so',
            '--pause',
            world,
        ],
        # cwd=[launch_dir],
        output='screen',
    )

    exhaustive_simulation = Node(
        package='walker_design_4',
        executable='exhaustive_simulation'
    )

    exhaustive_simulation_joint_angles = Node(
        package='walker_design_4',
        executable='exhaustive_simulation_joint_angles'
    )

    exhaustive_simulation_masses = Node(
        package='walker_design_4',
        executable='exhaustive_simulation_masses'
    )

    return LaunchDescription([
        headless_arg, 
        sim_time_arg,
        world_arg, 
        use_simulator_arg,
        
        start_gazebo_server_cmd,
        # exhaustive_simulation,
        exhaustive_simulation_joint_angles,
        # exhaustive_simulation_masses
        #kneelock
    ])