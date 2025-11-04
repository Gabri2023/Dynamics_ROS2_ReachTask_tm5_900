'''

Author: David Valencia
Date: 07/ 04 /2022

Modification: 07/ 04 /2022

Describer: 

		Main environment v2.0
		
		This script is the main environment of my project. Here is the body of the code
		state state, action samples, data from sensor, reset request all are generated/read here.

		I use an action client to move the robot arm while using a client service to move the target point (green sphere).

		Also, the state space is created with the end-effector position, the sphere (target) position and the joint state.
		if the robot reaches the goal or the number of steps in each episode finishes the environment will reset i.e robot
		will move to target position and the target point will move to a new random location.

		To summarize, this script does:

			1) Return the state space
			2) Return the reward (distance between target and end-effector)
			3) Generate and return random action
			4) Reset the environment (robot to home position and target to new position

		However in order to be me organized I just create the class here without node and to run
		

		To run this environmet --> I create a second script called: run_environmet.py or collection_data.py  

		Executable name in the setup file: no needed here cause i will run the second script
'''

import os
import sys
import time
import rclpy
import random
import numpy as np
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState

import tf2_ros 
from tf2_ros import TransformException

from rclpy.action        import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from rclpy.duration import Duration
from gazebo_msgs.msg import ContactsState




class MyRLEnvironmentNode(Node):

	def __init__ (self):

		super().__init__('node_main_rl_environment')
		self.robot_x = 0.0
		self.robot_y = 0.0
		self.robot_z = 0.0

		self.joint_1_pos = 0.0
		self.joint_2_pos = 0.0
		self.joint_3_pos = 0.0
		self.joint_4_pos = 0.0
		self.joint_5_pos = 0.0
		self.joint_6_pos = 0.0

		self.prev_joint_1_pos = 0.0
		self.prev_joint_2_pos = 0.0
		self.prev_joint_3_pos = 0.0
		self.prev_joint_4_pos = 0.0
		self.prev_joint_5_pos = 0.0
		self.prev_joint_6_pos = 0.0

		self.pos_sphere_x = 0.0
		self.pos_sphere_y = 0.0
		self.pos_sphere_z = 0.0

		self.collision = False
		self.node_ready = False  

		print ("initializing.....")
		
		# end-effector transformation
		self.tf_buffer   = tf2_ros.Buffer()
		self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #----------- serve per il bumper_sensor sulla flange_link

		self.bumper_links = ["my_doosan_robot::wrist_1_link::wrist_1_link_collision",
					   "my_doosan_robot::wrist_2_link::wrist_2_link_collision", 
					   "my_doosan_robot::wrist_3_link::wrist_3_link_collision", 
					   "my_doosan_robot::flange_link::flange_link_collision", 
					   "my_doosan_robot::arm_1_link::arm_1_link_collision",
					   "my_doosan_robot::arm_2_link::arm_2_link_collision",
					   "my_doosan_robot::shoulder_1_link::shoulder_1_link_collision",
					   ]
		self.contact_state = None
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_flange_link',
				self.contact_state_callback,
				10
			)
		'''
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_base_link',
				self.contact_state_callback,
				10
			)
			'''
		
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_wrist_1_link',
				self.contact_state_callback,
				10
			)
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_wrist_2_link',
				self.contact_state_callback,
				10
			)
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_wrist_3_link',
				self.contact_state_callback,
				10
			)
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_arm_1_link',
				self.contact_state_callback,
				10
			)
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_arm_2_link',
				self.contact_state_callback,
				10
			)
		self.contact_sensor_subscription = self.create_subscription(
				ContactsState,
				'/contact_sensor/bumper_shoulder_1_link',
				self.contact_state_callback,
				10
			)


		# --------------------------Client for reset the sphere position --------------------------#
		self.client_reset_sphere = self.create_client(SetEntityState,'/gazebo/set_entity_state')
		while not self.client_reset_sphere.wait_for_service(timeout_sec=0.2):
			self.get_logger().info('sphere reset-service not available, waiting...')
		self.request_sphere_reset = SetEntityState.Request()


		# ------------------------- Action-client to change joints position -----------------------#
		self.trajectory_action_client = ActionClient (self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')


		# --------------------------Subcribers topics --------------------------------------------#

		# Subcribe topic with the joints states
		self.joint_state_subscription = message_filters.Subscriber(self, JointState, '/joint_states')
		
		# Subcribe topic with the sphere position
		self.target_point_subscription = message_filters.Subscriber(self, ModelStates, '/gazebo/model_states')

		# Create the message filter (if a msg is detected for each subcriber, do the callback)
		#self.ts = message_filters.TimeSynchronizer([self.joint_state_subscription, self.target_point_subscription], queue_size=30)
		self.ts = message_filters.ApproximateTimeSynchronizer([self.joint_state_subscription, self.target_point_subscription], queue_size=10, slop=0.1, allow_headerless=True)
		self.ts.registerCallback(self.initial_callback)



	def contact_state_callback(self, msg):
		self.contact_state = msg

    # Se ci sono stati contatti, guarda ogni state
		if self.contact_state is not None:
			for state in msg.states:
				name1 = state.collision1_name
				name2 = state.collision2_name

				# Se uno dei due nomi contiene uno dei tuoi link bumper
				for bumper in self.bumper_links:
					if bumper in name1 or bumper in name2:
						self.collision = True
						return



	def initial_callback(self, joint_state_msg, target_point_msg):

		self.prev_joint_1_pos = self.joint_1_pos
		self.prev_joint_2_pos = self.joint_2_pos
		self.prev_joint_3_pos = self.joint_3_pos
		self.prev_joint_4_pos = self.joint_4_pos
		self.prev_joint_5_pos = self.joint_5_pos
		self.prev_joint_6_pos = self.joint_6_pos

		# Position of each joint:
		self.joint_1_pos = joint_state_msg.position[0]
		self.joint_2_pos = joint_state_msg.position[1]
		self.joint_3_pos = joint_state_msg.position[2]
		self.joint_4_pos = joint_state_msg.position[3]
		self.joint_5_pos = joint_state_msg.position[4]
		self.joint_6_pos = joint_state_msg.position[5]



		# Velocity of each joint:
		self.joint_1_vel =  joint_state_msg.velocity[0]
		self.joint_2_vel =  joint_state_msg.velocity[1]
		self.joint_3_vel =  joint_state_msg.velocity[2]
		self.joint_4_vel =  joint_state_msg.velocity[3]
		self.joint_5_vel =  joint_state_msg.velocity[4]
		self.joint_6_vel =  joint_state_msg.velocity[5]

		# Determine the sphere position in Gazebo wrt world frame
		sphere_index = target_point_msg.name.index('my_sphere') # Get the corret index for the sphere
		self.pos_sphere_x = target_point_msg.pose[sphere_index].position.x 
		self.pos_sphere_y = target_point_msg.pose[sphere_index].position.y 
		self.pos_sphere_z = target_point_msg.pose[sphere_index].position.z 
		self.node_ready = True

		# Determine the pose(position and location) of the end-effector w.r.t. world frame
		self.robot_x, self.robot_y, self.robot_z = self.get_end_effector_transformation()
		print(self.robot_x, self.robot_y, self.robot_z)



	def get_end_effector_transformation(self):
		try:
			now = rclpy.time.Time()
			self.reference_frame = 'world'
			self.child_frame     = 'flange_link'
			trans = self.tf_buffer.lookup_transform(self.reference_frame, self.child_frame, now)
		except TransformException as ex:
			self.get_logger().debug(f'Could not transform {self.reference_frame} to {self.child_frame}: {ex}')
			# restituisci valori validi di fallback invece di None
			return 0.0, 0.0, 0.0
		else:
			ef_robot_x = trans.transform.translation.x
			ef_robot_y = trans.transform.translation.y
			ef_robot_z = trans.transform.translation.z
			return round(ef_robot_x,3), round(ef_robot_y,3), round(ef_robot_z,3)


	
	def reset_environment_request(self, timeout=5.0):

		# Every time this function is called a request to the Reset the Environment is sent 
		# i.e. Move the robot to home position and change the
		# sphere location and waits until get response/confirmation

		# -------------------- reset sphere position------------------#

		# For now the sphere's position will be inside a 1x1x1 workspace in front of the robot 
		sphere_position_x = random.uniform( -0.5, 0.5)
		sphere_position_y = random.uniform( -0.5, -0.8)
		sphere_position_z = random.uniform( 0.5, 1.0)

		self.request_sphere_reset.state.name = 'my_sphere'
		self.request_sphere_reset.state.reference_frame = 'world'
		self.request_sphere_reset.state.pose.position.x = sphere_position_x
		self.request_sphere_reset.state.pose.position.y = sphere_position_y
		self.request_sphere_reset.state.pose.position.z = sphere_position_z
		
		self.future_sphere_reset = self.client_reset_sphere.call_async(self.request_sphere_reset)

		self.get_logger().info('Reseting sphere to new position...')

		rclpy.spin_until_future_complete(self, self.future_sphere_reset)

		sphere_service_response = self.future_sphere_reset.result()
		
		if sphere_service_response.success:
			self.get_logger().info("Sphere Moved to a New Positon Success")
		else:
			self.get_logger().info("Sphere Reset Request failed")

		#---------------------reset robot position-------------------#
		
	
		home_point_msg = JointTrajectoryPoint()
		home_point_msg.positions     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		home_point_msg.velocities    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		home_point_msg.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		home_point_msg.time_from_start = Duration(seconds=2.0).to_msg()
	
		
		joint_names = ['elbow_joint','shoulder_1_joint','shoulder_2_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        
		home_goal_msg = FollowJointTrajectory.Goal()
		home_goal_msg.goal_time_tolerance    = Duration(seconds=0.3).to_msg()
		home_goal_msg.trajectory.joint_names = joint_names
		home_goal_msg.trajectory.points      = [home_point_msg]
		

		if not self.trajectory_action_client.wait_for_server(timeout_sec=10.0):
			self.get_logger().error('Trajectory action server not available!')
			t0 = time.time()
			while time.time() - t0 < timeout:
				if self.trajectory_action_client.wait_for_server(timeout_sec=0.5):
					break
				rclpy.spin_once(self, timeout_sec=0.1)
			else:
				self.get_logger().error('Trajectory action server still not available after timeout')
				return False
		
		send_home_goal_future = self.trajectory_action_client.send_goal_async(home_goal_msg) # Sending home-position request
		
		rclpy.spin_until_future_complete(self, send_home_goal_future) # Wait for goal status
		goal_reset_handle = send_home_goal_future.result()
		
		if not goal_reset_handle.accepted:
			self.get_logger().info(' Home-Goal rejected ')
			return
		
		self.get_logger().info('Moving robot to home position...')
		
		get_reset_result = goal_reset_handle.get_result_async()
		rclpy.spin_until_future_complete(self, get_reset_result)  # Wait for response
		pausa_secondi = 0.1  # Inizia con 50ms e vedi se è sufficiente
		time.sleep(pausa_secondi)

		if get_reset_result.result().result.error_code == 0:
			self.get_logger().info('Robot in Home position without problems')

		else:
			self.get_logger().info('There was a problem with the action')

		


	def action_step_service(self, action_values):
		
		# Every time this function is called, it passes the action vector (desire position of each joint) 
		# to the action-client to execute the trajectory
		
		points = []

		point_msg = JointTrajectoryPoint()
		point_msg.positions     = list(map(float, action_values*[1.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
		print('#######',point_msg.positions, '#######')
		point_msg.velocities    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		point_msg.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
		point_msg.time_from_start = Duration(seconds=4.0).to_msg() # be careful about this time 
		points.append(point_msg) 

		joint_names = ['elbow_joint','shoulder_1_joint','shoulder_2_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        
		goal_msg    = FollowJointTrajectory.Goal()
		goal_msg.goal_time_tolerance = Duration(seconds=0.3).to_msg() # goal_time_tolerance allows some freedom in time, so that the trajectory goal can still
															        # succeed even if the joints reach the goal some time after the precise end time of the trajectory.
															
		goal_msg.trajectory.joint_names = joint_names
		goal_msg.trajectory.points      = points

		self.get_logger().info('Waiting for action server to move the robot...')
		self.trajectory_action_client.wait_for_server() # waits for the action server to be available

		self.get_logger().info('Sending goal-action request...')
		self.send_goal_future = self.trajectory_action_client.send_goal_async(goal_msg) 

		self.get_logger().info('Checking if the goal is accepted...')
		rclpy.spin_until_future_complete(self, self.send_goal_future ) # Wait for goal status

		goal_handle = self.send_goal_future.result()

		if not goal_handle.accepted:
			self.get_logger().info(' Action-Goal rejected ')
			return
		self.get_logger().info('Action-Goal accepted')

		self.get_logger().info('Checking the response from action-service...')
		self.get_result = goal_handle.get_result_async()
		rclpy.spin_until_future_complete(self, self.get_result ) # Wait for response

		if self.get_result.result().result.error_code == 0:
			self.get_logger().info('Action Completed without problem')
		else:
			self.get_logger().info('There was a problem with the action')


	


	def calculate_reward_funct(self):

		# I aim with this function to get the reward value. For now, the reward is based on the distance
		# i.e. Calculate the euclidean distance between the link6 (end effector) and sphere (target point)
		# and each timestep the robot receives -1 but if it reaches the goal (distance < 0.05) receives +10
		
		try:
			robot_end_position    = np.array((self.robot_x, self.robot_y, self.robot_z))
			target_point_position = np.array((self.pos_sphere_x, self.pos_sphere_y, self.pos_sphere_z))
			
		except: 
			self.get_logger().info('could not calculate the distance yet, trying again...')
			return -0.2, True

		reward_d = 0
		k = 0.5

		if self.robot_z <= -0.1:  #per evitare che il robot rimanda bloccato sotto terra
			reward_d=-100
			done = True
			return reward_d, done
		else:
			distance = np.linalg.norm(robot_end_position - target_point_position)
			print ("###----DISTANZA---", distance)

			if distance <= 0.05:
				self.get_logger().info('Goal Reached')
				done = True
				reward_d = 10
			else:
				done = False
				reward_d = -0.2
				reward_d -= k * distance

			return reward_d, done
		
	def calculate_reward_funct_2(self):

		# I aim with this function to get the reward value. For now, the reward is based on the distance
		# i.e. Calculate the euclidean distance between the link6 (end effector) and sphere (target point)
		# and each timestep the robot receives -1 but if it reaches the goal (distance < 0.05) receives +10
		
		joint_pos = np.array((self.joint_1_pos, self.joint_2_pos, self.joint_3_pos,self.joint_4_pos,self.joint_5_pos,self.joint_6_pos ))
		prev_joint_pos = np.array((self.prev_joint_1_pos, self.prev_joint_2_pos, self.prev_joint_3_pos, self.prev_joint_4_pos, self.prev_joint_5_pos, self.prev_joint_6_pos))

		try:
			robot_end_position    = np.array((self.robot_x, self.robot_y, self.robot_z))
			target_point_position = np.array((self.pos_sphere_x, self.pos_sphere_y, self.pos_sphere_z))
			
		except: 
			self.get_logger().info('could not calculate the distance yet, trying again...')
			return -10, True

		reward_d = 0
		reward_scale=10

		if self.robot_z <= -0.1:  #per evitare che il robot rimanda bloccato sotto terra
			reward_d=-1100
			print("TESTA SOTTO!!")
			done = True
			return reward_d, done
		else:
			distance = np.linalg.norm(robot_end_position - target_point_position)
			print ("###----DISTANZA---", distance)

			if distance <= 0.15:
				self.get_logger().info('Goal Reached')
				done = True
				reward_d = reward_scale * 1
			else:
				done = False
				reward_d = reward_scale * -1

		# 1. Crea il vettore delle *variazioni* di posizione (delta angoli)
		joint_deltas = np.array([
			self.joint_1_pos - self.prev_joint_1_pos,
			self.joint_2_pos - self.prev_joint_2_pos,
			self.joint_3_pos - self.prev_joint_3_pos,
			self.joint_4_pos - self.prev_joint_4_pos,
			self.joint_5_pos - self.prev_joint_5_pos,
			self.joint_6_pos - self.prev_joint_6_pos
		])

		# 2. Calcola la norma L2 (Euclidea)
		# Questo valore è 0 se il robot è fermo e cresce al crescere della variazione
		jerk_norm = np.linalg.norm(joint_pos - prev_joint_pos)
		print('jernorm', jerk_norm)
		print('distanze', joint_pos, prev_joint_pos, joint_pos - prev_joint_pos)

		# 3. Calcola la penalità (è negativa)
		K_JERK = 0.5
		jerk_penalty = -K_JERK * jerk_norm

		
		# 4. Aggiungi la penalità al reward totale
		reward_d = reward_d + jerk_penalty

		return reward_d, done


	def state_space_funct(self):
    # costruisce lo stato sempre, usando valori di default se necessario
		try:
			state = [
				float(self.robot_x), float(self.robot_y), float(self.robot_z),
				float(self.joint_1_pos), float(self.joint_2_pos), float(self.joint_3_pos),
				float(self.joint_4_pos), float(self.joint_5_pos), float(self.joint_6_pos),
				float(self.pos_sphere_x), float(self.pos_sphere_y), float(self.pos_sphere_z)
			]
		except Exception as e:
			self.get_logger().warning('state_space_funct: node not ready or invalid values, returning zeros. ' + str(e))
			state = [0.0]*12
		return np.array(state, dtype=np.float32)


