'''
Author: David Valencia
Date: 07/ 04 /2022

Modification: 07/ 04 /2022
Refactor (Delta Control): 29/ 10 /2025

Describer: 

        Main environment v2.0
        Refactored to use delta/incremental actions for stable RL training.
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

        self.state_updated_flag = False

        
        self.action_step_size = 0.5 

        # Limiti dei giunti (presi dal tuo URDF)
        # NOTA: I polsi nel tuo URDF erano bloccati a 0.0. Li ho sbloccati 
        # a +/- 3.14 (180 gradi) per un controllo reale. 
        # AGGIUSTA QUESTI VALORI se i tuoi limiti reali sono diversi.
        # Ordine: [shoulder_1, shoulder_2, elbow, wrist_1, wrist_2, wrist_3]
        pi = 3.14159
        self.joint_limits_min = np.array([
            -180*pi/180,  # shoulder_1_lower_limit
             -90*pi/180,  # shoulder_2_lower_limit
            -155*pi/180,  # elbow_lower_limit
            -90*pi/180,        # wrist_1 (sbloccato)
            -90*pi/180,        # wrist_2 (sbloccato)
            -90*pi/180         # wrist_3 (sbloccato)
        ], dtype=np.float32)
        
        self.joint_limits_max = np.array([
             180*pi/180,  # shoulder_1_upper_limit
              90*pi/180,  # shoulder_2_upper_limit
             155*pi/180,  # elbow_upper_limit
             90*pi/180,        # wrist_1 (sbloccato)
             90*pi/180,        # wrist_2 (sbloccato)
             90*pi/180         # wrist_3 (sbloccato)
        ], dtype=np.float32)

        # ---> FINE MODIFICA

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        # Posizioni attuali
        self.joint_1_pos = 0.0
        self.joint_2_pos = 0.0
        self.joint_3_pos = 0.0
        self.joint_4_pos = 0.0
        self.joint_5_pos = 0.0
        self.joint_6_pos = 0.0

        # Posizioni step precedente (per penalità jerk)
        self.prev_joint_1_pos = 0.0
        self.prev_joint_2_pos = 0.0
        self.prev_joint_3_pos = 0.0
        self.prev_joint_4_pos = 0.0
        self.prev_joint_5_pos = 0.0
        self.prev_joint_6_pos = 0.0

        # Velocità attuali (se servono)
        self.joint_1_vel = 0.0
        self.joint_2_vel = 0.0
        self.joint_3_vel = 0.0
        self.joint_4_vel = 0.0
        self.joint_5_vel = 0.0
        self.joint_6_vel = 0.0

        self.pos_sphere_x = 0.0
        self.pos_sphere_y = 0.0
        self.pos_sphere_z = 0.0

        self.collision = False
        self.node_ready = False  

        print ("initializing.....")
        
        # end-effector transformation
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #----------- Bumper Sensors
        self.bumper_links = [
                       "my_doosan_robot::wrist_1_link::wrist_1_link_collision",
                       "my_doosan_robot::wrist_2_link::wrist_2_link_collision", 
                       "my_doosan_robot::wrist_3_link::wrist_3_link_collision", 
                       "my_doosan_robot::flange_link::flange_link_collision", 
                       "my_doosan_robot::arm_1_link::arm_1_link_collision",
                       "my_doosan_robot::arm_2_link::arm_2_link_collision",
                       "my_doosan_robot::shoulder_1_link::shoulder_1_link_collision",
                       # Aggiungi qui altri link se necessario
                       ]
        
        # ---> INIZIO MODIFICA: Correzione bug iscrizioni multiple
        # Salviamo le iscrizioni in una lista per evitare che vengano sovrascritte
        self.bumper_subscriptions = [] 
        self.contact_state = None
        
        # Lista dei topic dei bumper a cui iscriversi
        bumper_topics = [
            '/contact_sensor/bumper_flange_link',
            '/contact_sensor/bumper_wrist_1_link',
            '/contact_sensor/bumper_wrist_2_link',
            '/contact_sensor/bumper_wrist_3_link',
            '/contact_sensor/bumper_arm_1_link',
            '/contact_sensor/bumper_arm_2_link',
            '/contact_sensor/bumper_shoulder_1_link',
            # Aggiungi qui altri topic se necessario
        ]
        
        for topic in bumper_topics:
            subscription = self.create_subscription(
                ContactsState,
                topic,
                self.contact_state_callback,
                10
            )
            self.bumper_subscriptions.append(subscription)
        # ---> FINE MODIFICA


        # --------------------------Client for reset the sphere position --------------------------#
        self.client_reset_sphere = self.create_client(SetEntityState,'/gazebo/set_entity_state')
        while not self.client_reset_sphere.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('sphere reset-service not available, waiting...')
        self.request_sphere_reset = SetEntityState.Request()


        # ------------------------- Action-client to change joints position -----------------------#
        self.trajectory_action_client = ActionClient (self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')


        # --------------------------Subcribers topics --------------------------------------------#
        self.joint_state_subscription = message_filters.Subscriber(self, JointState, '/joint_states')
        self.target_point_subscription = message_filters.Subscriber(self, ModelStates, '/gazebo/model_states')

        self.ts = message_filters.ApproximateTimeSynchronizer([self.joint_state_subscription, self.target_point_subscription], queue_size=10, slop=0.1, allow_headerless=True)
        self.ts.registerCallback(self.initial_callback)



    def contact_state_callback(self, msg):
        self.contact_state = msg
        
        # Se stiamo già gestendo una collisione, non fare nulla
        if self.collision:
            return

        if self.contact_state is not None:
            for state in msg.states:
                name1 = state.collision1_name
                name2 = state.collision2_name

                # Se uno dei due nomi contiene uno dei tuoi link bumper
                for bumper in self.bumper_links:
                    if bumper in name1 or bumper in name2:
                        self.collision = True
                        self.get_logger().warn(f'COLLISIONE RILEVATA TRA: {name1} e {name2}')
                        return # Esci appena trovi la prima collisione



    def initial_callback(self, joint_state_msg, target_point_msg):

        # 1. Salva la posizione corrente come "precedente" (per penalità jerk)
        #    Questo deve essere fatto PRIMA di aggiornare i valori.
        self.prev_joint_1_pos = self.joint_1_pos
        self.prev_joint_2_pos = self.joint_2_pos
        self.prev_joint_3_pos = self.joint_3_pos
        self.prev_joint_4_pos = self.joint_4_pos
        self.prev_joint_5_pos = self.joint_5_pos
        self.prev_joint_6_pos = self.joint_6_pos

        # ---> INIZIO MODIFICA: Correzione ordine lettura giunti
        # Assumiamo che l'ordine in /joint_states sia:
        # [shoulder_1, shoulder_2, elbow, wrist_1, wrist_2, wrist_3]
        # Se l'ordine è diverso, devi stamparlo con `ros2 topic echo /joint_states`
        # e adattare gli indici [N] qui sotto.
        try:
            # 1. Trova l'indice di ogni giunto in base al NOME
            idx_s1 = joint_state_msg.name.index('shoulder_1_joint') # idx_s1 sarà 1
            idx_s2 = joint_state_msg.name.index('shoulder_2_joint') # idx_s2 sarà 2
            idx_e  = joint_state_msg.name.index('elbow_joint')      # idx_e sarà 0
            idx_w1 = joint_state_msg.name.index('wrist_1_joint')
            idx_w2 = joint_state_msg.name.index('wrist_2_joint')
            idx_w3 = joint_state_msg.name.index('wrist_3_joint')
            
            # 2. Aggiorna le posizioni "interne" in modo LOGICO
            # (La nostra variabile logica "joint_1" prende il valore di "shoulder_1")
            self.joint_1_pos = joint_state_msg.position[idx_s1] # Prende il valore all'indice 1
            self.joint_2_pos = joint_state_msg.position[idx_s2] # Prende il valore all'indice 2
            self.joint_3_pos = joint_state_msg.position[idx_e]  # Prende il valore all'indice 0
            self.joint_4_pos = joint_state_msg.position[idx_w1]
            self.joint_5_pos = joint_state_msg.position[idx_w2]
            self.joint_6_pos = joint_state_msg.position[idx_w3]

            # 3. Aggiorna le velocità in modo LOGICO
            self.joint_1_vel = joint_state_msg.velocity[idx_s1]
            self.joint_2_vel = joint_state_msg.velocity[idx_s2]
            self.joint_3_vel = joint_state_msg.velocity[idx_e]
            self.joint_4_vel = joint_state_msg.velocity[idx_w1]
            self.joint_5_vel = joint_state_msg.velocity[idx_w2]
            self.joint_6_vel = joint_state_msg.velocity[idx_w3]
            
        except ValueError as e:
            self.get_logger().error(f"Errore: Giunto non trovato in /joint_states. Messaggio: {e}")
            self.get_logger().error(f"Giunti ricevuti: {joint_state_msg.name}")
            return
        # ---> FINE MODIFICA

        # Determine the sphere position in Gazebo wrt world frame
        try:
            sphere_index = target_point_msg.name.index('my_sphere')
            self.pos_sphere_x = target_point_msg.pose[sphere_index].position.x 
            self.pos_sphere_y = target_point_msg.pose[sphere_index].position.y 
            self.pos_sphere_z = target_point_msg.pose[sphere_index].position.z 
        except ValueError:
            self.get_logger().warn("Modello 'my_sphere' non trovato in /gazebo/model_states")
            # Non aggiornare le posizioni della sfera, usa le vecchie
            pass
            
        self.node_ready = True

        # Determine the pose(position and location) of the end-effector w.r.t. world frame
        self.robot_x, self.robot_y, self.robot_z = self.get_end_effector_transformation()
        
        self.state_updated_flag = True
        
        # self.get_logger().info(f"Pos EF: [{self.robot_x}, {self.robot_y}, {self.robot_z}]")



    def get_end_effector_transformation(self):
        try:
            now = rclpy.time.Time()
            self.reference_frame = 'world'
            self.child_frame     = 'flange_link'
            trans = self.tf_buffer.lookup_transform(self.reference_frame, self.child_frame, now, timeout=Duration(seconds=0.05))
        except TransformException as ex:
            self.get_logger().debug(f'Could not transform {self.reference_frame} to {self.child_frame}: {ex}')
            # restituisci valori validi di fallback (le posizioni precedenti)
            return self.robot_x, self.robot_y, self.robot_z
        else:
            ef_robot_x = trans.transform.translation.x
            ef_robot_y = trans.transform.translation.y
            ef_robot_z = trans.transform.translation.z
            return round(ef_robot_x,3), round(ef_robot_y,3), round(ef_robot_z,3)


    
    def reset_environment_request(self, timeout=5.0):

        # -------------------- reset sphere position------------------#
        sphere_position_x = random.uniform( -0.5, 0.5)
        sphere_position_y = random.uniform( 0.65, -0.65)
        sphere_position_z = random.uniform( 0.4, 0.75)

        self.request_sphere_reset.state.name = 'my_sphere'
        self.request_sphere_reset.state.reference_frame = 'world'
        self.request_sphere_reset.state.pose.position.x = sphere_position_x
        self.request_sphere_reset.state.pose.position.y = sphere_position_y
        self.request_sphere_reset.state.pose.position.z = sphere_position_z
        
        self.future_sphere_reset = self.client_reset_sphere.call_async(self.request_sphere_reset)
        self.get_logger().info('Reseting sphere to new position...')
        rclpy.spin_until_future_complete(self, self.future_sphere_reset)
        
        try:
            sphere_service_response = self.future_sphere_reset.result()
            if sphere_service_response and sphere_service_response.success:
                self.get_logger().info("Sphere Moved to a New Positon Success")
            else:
                self.get_logger().warn("Sphere Reset Request failed or no response")
        except Exception as e:
            self.get_logger().error(f"Eccezione durante il reset della sfera: {e}")

        #---------------------reset robot position-------------------#
        home_point_msg = JointTrajectoryPoint()
        home_point_msg.positions     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home_point_msg.velocities    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home_point_msg.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home_point_msg.time_from_start = Duration(seconds=0.5).to_msg() # Tempo più lungo per un reset sicuro
    
        
        # ---> INIZIO MODIFICA: Correzione ordine giunti
        joint_names = ['shoulder_1_joint','shoulder_2_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        # ---> FINE MODIFICA
        
        home_goal_msg = FollowJointTrajectory.Goal()
        home_goal_msg.goal_time_tolerance    = Duration(seconds=0.002).to_msg()
        home_goal_msg.trajectory.joint_names = joint_names
        home_goal_msg.trajectory.points      = [home_point_msg]
        
        if not self.trajectory_action_client.wait_for_server(timeout_sec=5.0): # Timeout ridotto
            self.get_logger().error('Trajectory action server not available!')
            return False # Indica fallimento
        
        send_home_goal_future = self.trajectory_action_client.send_goal_async(home_goal_msg)
        rclpy.spin_until_future_complete(self, send_home_goal_future)
        goal_reset_handle = send_home_goal_future.result()
        
        if not goal_reset_handle or not goal_reset_handle.accepted:
            self.get_logger().info(' Home-Goal rejected ')
            return False
        
        self.get_logger().info('Moving robot to home position...')
        get_reset_result = goal_reset_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_reset_result)
        
        # Pausa per stabilizzazione
        time.sleep(0.1) 

        try:
            result = get_reset_result.result()
            if result.result.error_code == 0:
                self.get_logger().info('Robot in Home position without problems')
                
                # ---> INIZIO MODIFICA: Reset posizioni precedenti per penalità jerk
                self.prev_joint_1_pos = 0.0
                self.prev_joint_2_pos = 0.0
                self.prev_joint_3_pos = 0.0
                self.prev_joint_4_pos = 0.0
                self.prev_joint_5_pos = 0.0
                self.prev_joint_6_pos = 0.0
                # ---> FINE MODIFICA
                return True # Successo
            else:
                self.get_logger().warn(f'Reset robot fallito con codice errore: {result.result.error_code}')
                return False
        except Exception as e:
            self.get_logger().error(f"Eccezione durante il reset del robot: {e}")
            return False


    def action_step_service(self, action_values):
        
        # 'action_values' è un array numpy da 6 elementi (es. da -1 a 1)

        self.state_updated_flag = False
        
        points = []
        point_msg = JointTrajectoryPoint()

        # ---> INIZIO MODIFICA: Logica di controllo Delta (Incrementale)
        
        # 1. Prendi la posizione corrente (in ordine corretto)
        current_pos = np.array([
            self.joint_1_pos, self.joint_2_pos, self.joint_3_pos,
            self.joint_4_pos, self.joint_5_pos, self.joint_6_pos
        ])

        # 2. Calcola il target sommando il delta (azione * step_size)
        delta_action = action_values * self.action_step_size
        target_positions = current_pos + delta_action

        # 3. Applica i limiti (Clipping) - FONDAMENTALE per stabilità
        target_positions = np.clip(
            target_positions, 
            self.joint_limits_min, 
            self.joint_limits_max
        )

        # 4. Assegna le posizioni target calcolate
        point_msg.positions = list(map(float, target_positions))
        
        # ---> FINE MODIFICA

        # self.get_logger().info(f'TARGET POS: {np.round(point_msg.positions, 2)}')
        
        point_msg.velocities    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point_msg.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # ---> INIZIO MODIFICA: Riduci il tempo di step (FONDAMENTALE)
        # 4.0 secondi è troppo. Usiamo un tempo breve per step reattivi.
        point_msg.time_from_start = Duration(seconds=0.2).to_msg()
        # ---> FINE MODIFICA
        
        points.append(point_msg) 

        # ---> INIZIO MODIFICA: Correzione ordine giunti
        joint_names = ['shoulder_1_joint','shoulder_2_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        # ---> FINE MODIFICA
        
        goal_msg    = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(seconds=0.1).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points      = points

        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server non disponibile per lo step!')
            return # Salta questo step
            
        self.send_goal_future = self.trajectory_action_client.send_goal_async(goal_msg) 
        rclpy.spin_until_future_complete(self, self.send_goal_future )
        goal_handle = self.send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().warn(' Action-Goal (step) rejected ')
            return

        self.get_result = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self.get_result )

        try:
            result = self.get_result.result()
            if result.result.error_code != 0:
                self.get_logger().warn(f'Azione (step) fallita con codice errore: {result.result.error_code}')
        except Exception as e:
             self.get_logger().error(f'Eccezione durante lo step: {e}')
        
        
    def calculate_reward_funct_2(self):
        # ... (Questa funzione è invariata e ora dovrebbe funzionare) ...
        
        #joint_pos = np.array((self.joint_1_pos, self.joint_2_pos, self.joint_3_pos,self.joint_4_pos,self.joint_5_pos,self.joint_6_pos ))
        #prev_joint_pos = np.array((self.prev_joint_1_pos, self.prev_joint_2_pos, self.prev_joint_3_pos, self.prev_joint_4_pos, self.prev_joint_5_pos, self.prev_joint_6_pos))

        try:
            robot_end_position    = np.array((self.robot_x, self.robot_y, self.robot_z))
            target_point_position = np.array((self.pos_sphere_x, self.pos_sphere_y, self.pos_sphere_z))
            
        except Exception: 
            self.get_logger().warn('Dati di posizione non validi per il reward, ritorno -10')
            return -10, True

        reward_d = 0
        reward_scale=10

        if self.robot_z <= -0.01:  #per evitare che il robot rimanda bloccato sotto terra
            reward_d=-1100
            self.get_logger().warn("TESTA SOTTO!!")
            done = True
            return reward_d, done
        else:
            distance = np.linalg.norm(robot_end_position - target_point_position)
            print ("###----DISTANZA---", distance)

            if distance <= 0.12:
                self.get_logger().info('### Goal Reached ###')
                done = True
                reward_d = reward_scale * 1
            else:
                done = False
                reward_d = reward_scale * -1

        # Calcolo penalità Jerk
        #jerk_norm = np.linalg.norm(joint_pos - prev_joint_pos)
        # print('jernorm', jerk_norm)
        # print('distanze', joint_pos, prev_joint_pos, joint_pos - prev_joint_pos)

        #K_JERK = 0.5
        #jerk_penalty = -K_JERK * jerk_norm
        
        #total_reward = reward_d + jerk_penalty

        return reward_d, done


    def state_space_funct(self):
    # ... (Questa funzione è invariata) ...
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