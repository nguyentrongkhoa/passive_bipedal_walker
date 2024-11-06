import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    path_to_urdf = os.path.join(get_package_share_directory('walker_design_1'), 'urdf', 'walker_design_1.urdf') # change this

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(path_to_urdf),
        description="Absolute path to the urdf file",
    )

    sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    gazebo_launch_path = os.path.join(
        get_package_share_directory('gazebo_ros'),
        'launch',
        'gazebo.launch.py'
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

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[path_to_urdf]
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
        arguments=["-topic", "/robot_description", "-entity", "walker_design_1"]
    )

    # TODO: research about the JointControllers plugin in gazebo 

    return LaunchDescription([
        sim_time_arg,
        model_arg,
        # model_description, 
        robot_state_publisher,
        joint_state_publisher,
        spawn_urdf_in_gazebo
    ])