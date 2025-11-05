'''

Describer:  
			
			This scrip LOAD and START a basic joint_trajectory_controller
			The info and configuration of the controller can be found in the config folder:
			
			/src/tm5_900/config/simple_controller.yaml

			Note: I remove rviz here in order to using an other rviz config later on my own environment 
                  Also, gazebo and the empty world are launched in my own environment later 

			--> I will invoke this launch file later in my environment launch file <--
'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue



def generate_launch_description():


	robot_model = 'tm5_900_robot'


	xacro_file = os.path.join(
    get_package_share_directory('tm5_900'),
    'description',
    'desc_tm900',
    'urdf',
    robot_model + '.urdf.xacro'
)

	
	# Robot State Publisher 
	robot_state_publisher = Node(package    ='robot_state_publisher',
								 executable ='robot_state_publisher',
								 name       ='robot_state_publisher',
								 output     ='both',
								 parameters =[{
									'robot_description': ParameterValue(
										Command(['xacro ', xacro_file]),
										value_type=str
									)
								}] )



	# Spawn the robot in Gazebo
	spawn_entity_robot = Node(package     ='gazebo_ros', 
							  executable  ='spawn_entity.py', 
							  arguments   = ['-entity', 'tm5_900', '-topic', 'robot_description'],
							  output      ='screen')
	



	# Gazebo   
	#world_file_name = 'my_empty_world.world'
	#world = os.path.join(get_package_share_directory('tm5_900'), 'worlds', world_file_name)
	#gazebo_node = ExecuteProcess(cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], output='screen')


	


	# load and START the controllers in launch file
	
	load_joint_state_broadcaster = ExecuteProcess(
										cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
										output='screen')

	
	load_joint_trajectory_controller = ExecuteProcess( 
										cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'], 
										output='screen')



	return LaunchDescription([robot_state_publisher, spawn_entity_robot, load_joint_state_broadcaster, load_joint_trajectory_controller  ])