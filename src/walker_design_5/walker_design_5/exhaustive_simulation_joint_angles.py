# NOTE: interesting gazebo topics to subscribe to:
# 1: /gazebo/default/joint/gazebo/default/pose/info?p=/time
# 2: /gazebo/default/world_control/reset
# 3: /gazebo/default/world_stats
# 4: /gazebo/default/model/info ... pose/position

# NOTE: the joint_state_broadcaster publishes from the available state_interface to 2 topics:
# 1. /joint_states: sensor_msgs/msg/JointState
# 2. /dynamic_joint_states: control_msgs/msg/DynamicJointState
# NOTE: this is particularly helpful to check if a leg is in an extended position or not

# NOTE: /clock: rosgraph_msgs/msg/Clock

import os
import rclpy
import numpy as np
import csv

from rclpy.qos import *
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from gazebo_msgs.msg import LinkStates
from std_srvs.srv import Empty
from numpy import linspace, sin, deg2rad, pi
from itertools import product

from jinja2 import Environment, FileSystemLoader
from subprocess import run, DEVNULL

import xml.etree.ElementTree as ET
import xml_functions

''' 
This node will loop over all possible joint angle combinations as initial condition for the left leg (swing leg).
The two jinja parameters are left_knee_angle and left_hip_angle and range from 0 to 90 degrees respectively

'''
class ExhaustiveSimulationMasses(Node):
    def __init__(self, ramp_angle=2, new_simulation=True, walker_counter=0, csv_filename='mass_simulation_result.csv'):
        """_summary_

        Parameters
        ----------
        ramp_angle : in degrees
            Should be from 1 to 10, by default 2
        new_simulation : bool, optional
            If this is true, the csv file will be completely overwritten at initialization. 
            Otherwise, the file will be appended to with new rows. This can be set to False,
            when for example Gazebo encounters an error and not all configurations have been tested.
            In that case, walker_counter should be set to the last line number in the csv file.
        walker_counter : int, optional
            Always equals 0 for a new simulation. 
        """        

        super().__init__('exhaustive_simulation_masses')
        self.RAMP_ANGLE_IN_DEG = ramp_angle 
        self.RAMP_ANGLE_IN_RAD = deg2rad(self.RAMP_ANGLE_IN_DEG)
        self.SPAWN_Z_OFFSET = 5 * sin(self.RAMP_ANGLE_IN_RAD) + 0.45 # NOTE: 5m is the length of the ramp
        self.SPAWN_Y_OFFSET = 0.3 # so that the walker does not fall backwards from the top of the ramp 
        # TODO: experiment with this
        self.SPAWN_R = -self.RAMP_ANGLE_IN_RAD # so that the stance leg is perpendicular to the ramp after spawning
        self.walker_counter = walker_counter

        # NOTE: when ALL links move slower than this, the walker is deemed stationary and will be deleted from the world
        self.velocity_threshold = 0.1 # m/s  # TODO: experiment with this

        # variables in the simulation
        self.ALL_M_UPPER = linspace(0.2, 0.5, 5)
        self.ALL_M_LOWER = linspace(1, 3, 20)
        self.ALL_M_COMBINATION = list(product(self.ALL_M_LOWER, self.ALL_M_UPPER))

        self.ALL_RIGHT_HIP_JOINT_ANGLE = linspace(2.5, 2.7, 20)

        # this original xml tree will serve as the basis for parametrization of the urdf file (mass, angles, ...)
        self.PATH_TO_URDF = os.path.join(get_package_share_directory('walker_design_5'), 'urdf')
        self.URDF_TREE = ET.parse(f'{self.PATH_TO_URDF}/walker_design_5.urdf')
        # assign other masses at the beginning 
        self.URDF_TREE = xml_functions.modify_mass_urdf(
            self.URDF_TREE,
            {'left_leg_link': 10,
             'right_leg_link': 10,
             'hip_link': 0.02}
        )
        
        # Subscribe to /joint_states topic
        self.link_states_subscriber = self.create_subscription(
            LinkStates,
            '/world/link_states',
            self.link_states_callback,
            10
        )
        
        # Create a service clients for spawning and deleting entities
        self.delete_entity_client = self.create_client(DeleteEntity, '/delete_entity')
        self.pause_physics_client = self.create_client(Empty, '/pause_physics')
        
        # Wait for the service to be available
        while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity service to become available...')

        # spawn the first configuration already at node initilization
        current_config = self.ALL_RIGHT_HIP_JOINT_ANGLE[self.walker_counter]
        current_urdf_tree = xml_functions.modify_joint_angles_urdf(
            self.URDF_TREE,
            {'right_hip_joint': current_config}
        )

        # create and write current_urdf to a new file in the same share directory as the template
        # so that the urdf can be spawned
        current_urdf_tree.write(f'{self.PATH_TO_URDF}/walker')
        # run spawn_entity.py node from the terminal
        # NOTE: this first spawn will not unpause the physics as opposed to subsequent ones.
        # this is so that one can observe if the starting position is correct
        run(['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-file', f'{self.PATH_TO_URDF}/walker',
            '-entity', 'walker',
            '-z', str(self.SPAWN_Z_OFFSET), # spawn the walker at the top of the ramp
            '-y', str(self.SPAWN_Y_OFFSET),
            '-R', str(self.SPAWN_R)],
            stdout=DEVNULL) # suppress output
        
        # write csv file headers 
        self.CSV_PATH = csv_filename 
        if new_simulation:
            with open(self.CSV_PATH, 'w') as file:
                writer =csv.writer(file)
                writer.writerow(['right_hip_joint_angle', 'distance'])

    # each time this function is called, a decision will be made whether the current entity is stationary or not
    # and therefore whether it should be deleted
    # NOTE: only when ALL link velocities are less than a predefined threshold, the current entity will be removed
    def link_states_callback(self, msg):

        should_delete_entity = True

        # Loop through each link's pose and twist
        for link_name, twist in zip(msg.name, msg.twist):
            linear_velocity = twist.linear
            magnitude = (linear_velocity.x ** 2 + linear_velocity.y ** 2 + linear_velocity.z ** 2) ** 0.5
            # NOTE: if evn only 1 link is still moving, the entity should not be deleted
            if magnitude > self.velocity_threshold:
                should_delete_entity = False
     
        if should_delete_entity:
            # calculate distance traveled by the walker
            # for this, the position of the dummy_link  will be used
            for link_name, pose in zip(msg.name, msg.pose):
                if link_name == 'walker::hip_link':
                    # this path vector points from the top of the ramp towards the position
                    # where the walker stops moving
                    path_vector = np.array([
                        pose.position.x, 
                        pose.position.y, 
                        pose.position.z - self.SPAWN_Z_OFFSET
                    ])
                    d = np.linalg.norm(path_vector)
                    
                    # append to csv file
                    with open(self.CSV_PATH, 'a') as file:
                        writer = csv.writer(file)
                        angles = self.ALL_RIGHT_HIP_JOINT_ANGLE[self.walker_counter]
                        data_to_write = [angles, d]
                        writer.writerow(data_to_write)
                        self.get_logger().info('Done appending to csv file')

            # call the /delete_entity service
            request = DeleteEntity.Request()
            request.name = 'walker' 
            self.delete_entity_client.call_async(request)

            # pause physics before the next robot is spawned
            # NOTE: if physics is not paused, sometimes the robot will be deleted almost
            # right after being spawned
            self.pause_physics_client.call_async(Empty.Request())

            # create new robot with new configuration
            self.walker_counter += 1
            current_config = self.ALL_RIGHT_HIP_JOINT_ANGLE[self.walker_counter]
            current_urdf_tree = xml_functions.modify_joint_angles_urdf(
                self.URDF_TREE,
                {'right_hip_joint': current_config}
            )
            # the file will be overwritten with current_urdf_tree
            current_urdf_tree.write(f'{self.PATH_TO_URDF}/walker')
            # run spawn_entity.py node from the terminal
            run(['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-file', f'{self.PATH_TO_URDF}/walker',
                '-entity', 'walker',
                '-z', str(self.SPAWN_Z_OFFSET), 
                '-y', str(self.SPAWN_Y_OFFSET),
                '-R', str(self.SPAWN_R),
                '-unpause'],
                stdout=DEVNULL)
            
def main(args=None):
    rclpy.init(args=args)
    exhaustive_simulation_joint_angles = ExhaustiveSimulationMasses(
        # TODO: change this for other ramp angles
        # TODO: when ramp_angle is changed, ramp.world should also be updated with the correct ramp mesh file
        ramp_angle=5, 
        new_simulation=True,
        walker_counter=0,
        csv_filename='joint_angles_simulation_result_10kg.csv'
    ) 
    try:
        rclpy.spin(exhaustive_simulation_joint_angles)
    except KeyboardInterrupt:
        exhaustive_simulation_joint_angles.get_logger().info('Node interrupted by user.')
    finally:
        exhaustive_simulation_joint_angles.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

