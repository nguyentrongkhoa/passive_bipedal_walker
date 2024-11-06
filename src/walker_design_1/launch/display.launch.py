from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    path_to_urdf = os.path.join(get_package_share_directory('walker_design_1'), 'urdf', 'walker_design_1.urdf') # change this
    rviz_config_path = os.path.join(get_package_share_directory('walker_design_1'), 'config', 'rviz_config.rviz')

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(path_to_urdf),
        description="Absolute path to the urdf file",
    )

    model_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': model_description}],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[path_to_urdf]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    ) 

    launch_rviz_action = ExecuteProcess(
        cmd=[f'ros2 run rviz2 rviz2 -d {rviz_config_path}'],
        output='log'
    )

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        model_arg,
        # model_description, 
        robot_state_publisher,
        joint_state_publisher,
        rviz

    ])