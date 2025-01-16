# NOTE: colcon build --packages-select walker_design_3 && source ~/.bashrc
# NOTE: ros2 launch walker_design_3 gazebo.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():

    # Set the path to the Gazebo ROS package
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros').find('gazebo_ros')   
    
    # Set the path to this package.
    # Change this when used as a template
    pkg_share = FindPackageShare(package='walker_design_3').find('walker_design_3')

    # NOTE: 
    headless = LaunchConfiguration('headless')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    world = LaunchConfiguration('world')
    use_simulator = LaunchConfiguration('use_simulator')
    factory = LaunchConfiguration('factory')

    # change this when used as a template
    path_to_urdf = os.path.join(get_package_share_directory('walker_design_3'), 'urdf', 'walker_design_3.urdf') 
    # Set the path to the world file
    world_file_name = 'ramp.world'
    world_path = os.path.join(pkg_share, 'worlds', world_file_name)

    headless_arg = DeclareLaunchArgument(
        name='headless',
        default_value='False',
        description='Whether to execute gzclient'
    )

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(path_to_urdf),
        description="Absolute path to the urdf file",
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

    factory_arg = DeclareLaunchArgument(
        name='factory',
        default_value='true',
        description='Whether to start Gazebo with GazeboRosFactory'
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

    model_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'robot_description': model_description}]
    )

    # gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so myworld.world
    # NOTE: to start gazebo with the ramp already loaded:
    # gazebo -s libgazebo_ros_init.so -s libgazebo_ros_factory.so worlds/ramp.world
    # NOTE: only executing the command above in a separate terminal works, adding the command to the launch file does not
    spawn_urdf_in_gazebo = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=["-topic", "/robot_description", 
                   "-entity", "walker_design_3",
                   "-z", "0.18",
                   "-y", "0.1",
                   "-R", "0.035"] # change this when used as a template
    )

    # load joint_state_broadcaster controller in Gazebo
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', "--controller-manager","/controller_manager"]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )

    return LaunchDescription([
        headless_arg, 
        sim_time_arg,
        model_arg,
        world_arg, 
        use_simulator_arg,
        factory_arg,
        
        robot_state_publisher,
        start_gazebo_server_cmd,
        spawn_urdf_in_gazebo,
        #rviz,
        #joint_state_publisher,
        #joint_state_publisher_gui
        # load_joint_state_broadcaster
    ])