"""
Riassunto del Codice: Nodo ROS 2 per Ambiente di Reinforcement Learning (RL)

Questo codice definisce un nodo ROS 2 (MyRLEnvironmentNode) essenziale per l'interazione
tra un algoritmo di Reinforcement Learning (RL) e un simulatore robotico (Gazebo)
che utilizza un braccio robotico (TM5-900).

Funzionalità Principali:
1.  Monitoraggio Stato: Sottoscrive i topic di ROS 2 (JointState, ModelStates, Contatti) per ottenere
    la posizione dei giunti, la posizione dell'End-Effector e la posizione dell'oggetto target ('my_sphere').
2.  Azioni di Step: Implementa la logica per eseguire un'azione (cambio di posizione dei giunti)
    utilizzando l'Action Client 'FollowJointTrajectory', applicando i limiti fisici del robot.
3.  Reset: Fornisce una funzione per resettare l'ambiente (spostando la sfera target in una
    nuova posizione casuale e riportando il robot nella posizione 'Home').
4.  Reward: Calcola la funzione di ricompensa (reward) basata sulla distanza
    tra l'End-Effector e la sfera, includendo penalità per collisioni o uscite dai limiti di lavoro.

A differenza del file "main_rl_environment.py", questo permette di inserire la posizione della sfera da teminale, 
attraverso il comando (in un altro terminale): 

    ros2 topic pub --once /target_position geometry_msgs/msg/Point "{x: ... , y: ... , z: ...}" 

Inserire i valori al posto dei puntini. Rispettare i vincoli dello spazio raggiungibile:

            x = [-0.5, 0.5]
            y = [-0.65, 0.65]
            z = [0.4, 0.75]

            
Il codice elaborerà un valore randomico della sfera finchè non si pubbilcherà un punto
manualmente sul topic /target_position (usando il comando alla riga 21). Il punto manuale
sarà immesso nel buffer dei punti da raggiungere e sarà spawnato al prossimo episodio.

Nota:
    nel buffer viene letto soltanto l'ultimo punto pubblicato, quindi per una sequenza di valori
    sarà letto soltanto l'ultimo disponibile.


"""

# Importa moduli standard Python.
import time
# Importa la libreria client ROS 2 Python.
import rclpy
# Usato per generare posizioni casuali per il reset della sfera.
import random
# Usato per calcoli numerici e manipolazione di array (es. limiti dei giunti, distanza).
import numpy as np
# Usato per la sincronizzazione temporale dei messaggi di più topic.
import message_filters
# Classe base per i nodi ROS 2.
from rclpy.node import Node
# Messaggio per lo stato dei giunti (posizione, velocità, ecc.).
from sensor_msgs.msg import JointState
# Messaggio per la posizione dei modelli in Gazebo (usato per la sfera target).
from gazebo_msgs.msg import ModelStates
# Servizio ROS 2 per reimpostare la posizione di un'entità in Gazebo (usato per la sfera).
from gazebo_msgs.srv import SetEntityState

# Usato per la gestione delle trasformazioni (End-Effector vs World).
import tf2_ros 
# Eccezione sollevata se una trasformazione fallisce.
from tf2_ros import TransformException

# Usato per interagire con l'Action Server del controller di traiettoria.
from rclpy.action        import ActionClient
# Punto di una traiettoria per i giunti.
from trajectory_msgs.msg import JointTrajectoryPoint
# Action ROS 2 standard per il controllo della traiettoria dei giunti.
from control_msgs.action import FollowJointTrajectory

# Usato per definire durate temporali (timeout, time_from_start).
from rclpy.duration import Duration
# Messaggio per rilevare lo stato dei contatti (collisioni).
from gazebo_msgs.msg import ContactsState

from geometry_msgs.msg import Point



# Definizione del nodo ROS 2 che funge da interfaccia tra RL e simulazione.
class MyRLEnvironmentNode(Node):

    # Costruttore della classe, inizializza tutte le variabili di stato, i client, i subscriber e i listener.
    def __init__ (self):

        # Inizializza il nodo ROS 2 con il nome specificato.
        super().__init__('node_main_rl_environment')

        # Flag per segnalare all'agente RL esterno che lo stato è stato aggiornato.
        self.state_updated_flag = False

        # Fattore di scala per l'azione ricevuta dall'agente RL (delta di posizione massima).
        self.action_step_size = 0.5 

        self.manual_target = None
        

        self.target_sub = self.create_subscription(
    Point,
    '/target_position',
    self.target_callback,
    10
)

        
        # Ordine: [shoulder_1, shoulder_2, elbow, wrist_1, wrist_2, wrist_3]
        pi = 3.14159
        # Definizione dei limiti minimi di movimento per ciascuno dei 6 giunti in radianti.
        self.joint_limits_min = np.array([
            -180*pi/180,  # shoulder_1_lower_limit
             -90*pi/180,  # shoulder_2_lower_limit
            -155*pi/180,  # elbow_lower_limit
            -90*pi/180,        # wrist_1_lower_limit 
            -90*pi/180,        # wrist_2_lower_limit
            -90*pi/180         # wrist_3_lower_limit
        ], dtype=np.float32)
        
        # Definizione dei limiti massimi di movimento per ciascuno dei 6 giunti in radianti.
        self.joint_limits_max = np.array([
             180*pi/180,  # shoulder_1_upper_limit
              90*pi/180,  # shoulder_2_upper_limit
             155*pi/180,  # elbow_upper_limit
             90*pi/180,        # wrist_1_upper_limit
             90*pi/180,        # wrist_2_upper_limit
             90*pi/180         # wrist_3_upper_limit
        ], dtype=np.float32)

        # Fine blocco di modifica.

        # Coordinate dell'End-Effector nel frame 'world'.
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0

        # Variabili per memorizzare la posizione attuale di ciascun giunto.
        # Posizioni attuali
        self.joint_1_pos = 0.0
        self.joint_2_pos = 0.0
        self.joint_3_pos = 0.0
        self.joint_4_pos = 0.0
        self.joint_5_pos = 0.0
        self.joint_6_pos = 0.0

        # Variabili per memorizzare la posizione dei giunti allo step precedente (per penalità jerk/movimento).
        # Posizioni step precedente (per penalità jerk)
        self.prev_joint_1_pos = 0.0
        self.prev_joint_2_pos = 0.0
        self.prev_joint_3_pos = 0.0
        self.prev_joint_4_pos = 0.0
        self.prev_joint_5_pos = 0.0
        self.prev_joint_6_pos = 0.0

        # Variabili per memorizzare la velocità attuale di ciascun giunto.
        # Velocità attuali (se servono)
        self.joint_1_vel = 0.0
        self.joint_2_vel = 0.0
        self.joint_3_vel = 0.0
        self.joint_4_vel = 0.0
        self.joint_5_vel = 0.0
        self.joint_6_vel = 0.0

        # Coordinate della sfera target 'my_sphere' nel frame 'world'.
        self.pos_sphere_x = 0.0
        self.pos_sphere_y = 0.0
        self.pos_sphere_z = 0.0

        # Flag che indica se è stata rilevata una collisione.
        self.collision = False
        self.touch= False
        # Flag che indica se il nodo ha ricevuto i dati iniziali ed è pronto per l'RL.
        self.node_ready = False  

        print ("initializing.....")
        
        # Inizializzazione del listener per le trasformazioni TF (usato per ottenere la posizione dell'end-effector).
        # end-effector transformation
        # Buffer per memorizzare le trasformazioni TF.
        self.tf_buffer   = tf2_ros.Buffer()
        # Listener per le trasformazioni (per ottenere la posa dell'End-Effector).
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Lista dei nomi dei link del robot da monitorare per le collisioni.
        #----------- Bumper Sensors
        self.bumper_links = [
                       "tm5_900::wrist_1_link::wrist_1_link_collision",
                       "tm5_900::wrist_2_link::wrist_2_link_collision", 
                       "tm5_900::wrist_3_link::wrist_3_link_collision", 
                       "tm5_900::flange_link::flange_link_collision", 
                       "tm5_900::arm_1_link::arm_1_link_collision",
                       "tm5_900::arm_2_link::arm_2_link_collision",
                       "tm5_900::shoulder_1_link::shoulder_1_link_collision",
                       "my_obstacle::cylinder_link_1::cylinder_collision_1",
                       "my_obstacle::cylinder_link_2::cylinder_collision_2",
                       "my_obstacle::cylinder_link_3::cylinder_collision_3"
                      
                       ]
        
        # Inizio blocco di modifica: Correzione bug iscrizioni multiple
        # Salviamo le iscrizioni in una lista per evitare che vengano sovrascritte
        self.bumper_subscriptions = [] 
        # Variabile per memorizzare l'ultimo stato di contatto ricevuto.
        self.contact_state = None
        
        # Lista dei topic dei sensori di contatto (bumper) a cui iscriversi.
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
        
        # Creazione delle sottoscrizioni per ogni topic di contatto.
        for topic in bumper_topics:
            subscription = self.create_subscription(
                ContactsState,
                topic,
                self.contact_state_callback,
                10
            )
            self.bumper_subscriptions.append(subscription)
       

        # Client per il servizio di reset della posizione della sfera in Gazebo.
        # --------------------------Client for reset the sphere position --------------------------#
        self.client_reset_sphere = self.create_client(SetEntityState,'/gazebo/set_entity_state')
        # Attesa che il servizio sia disponibile.
        while not self.client_reset_sphere.wait_for_service(timeout_sec=0.2):
            self.get_logger().info('sphere reset-service not available, waiting...')
        self.request_sphere_reset = SetEntityState.Request()


        # Action Client per inviare comandi di traiettoria al robot.
        # ------------------------- Action-client to change joints position -----------------------#
        self.trajectory_action_client = ActionClient (self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')


        # Subscriber per lo stato dei giunti e della sfera.
        # --------------------------Subcribers topics --------------------------------------------#
        self.joint_state_subscription = message_filters.Subscriber(self, JointState, '/joint_states')
        self.target_point_subscription = message_filters.Subscriber(self, ModelStates, '/gazebo/model_states')

        # Sincronizzatore temporale approssimativo per combinare i messaggi.
        self.ts = message_filters.ApproximateTimeSynchronizer([self.joint_state_subscription, self.target_point_subscription], queue_size=10, slop=0.1, allow_headerless=True)
        # Registra la callback che viene chiamata quando i messaggi sincronizzati arrivano.
        self.ts.registerCallback(self.initial_callback)


    # Callback che elabora i messaggi ContactsState per rilevare collisioni.
    def contact_state_callback(self, msg):
        self.contact_state = msg
        
        # Previene l'elaborazione se una collisione è già stata registrata (per evitare log multipli).
        # Se stiamo già gestendo una collisione, non fare nulla
        if self.collision:
            return

        if self.contact_state is not None:
            for state in msg.states:
                name1 = state.collision1_name
                name2 = state.collision2_name
                

                # Itera sui link monitorati per verificare se una collisione li coinvolge.
                # Se uno dei due nomi contiene uno dei tuoi link bumper
                for bumper in self.bumper_links:
                    if bumper in name1 or bumper in name2:
                        self.collision = True
                        self.get_logger().warn(f'COLLISIONE RILEVATA TRA: {name1} e {name2}')
                        if bumper in [
                            'my_obstacle::cylinder_link_1::cylinder_collision_1',
                            'my_obstacle::cylinder_link_2::cylinder_collision_2',
                            'my_obstacle::cylinder_link_3::cylinder_collision_3'
                        ]:
                            self.touch = True
                            self.get_logger().warn(f'### COLLISIONE CON OSTACOLO --> RESET HOME POSITION ###')
                return # Esci appena trovi la prima collisione


    # Callback che aggiorna le variabili di stato interno del robot e del target.
    def initial_callback(self, joint_state_msg, target_point_msg):

        # Salva la posizione corrente come "precedente" prima di aggiornarla con il nuovo stato.
        # 1. Salva la posizione corrente come "precedente" (per penalità jerk)
        #    Questo deve essere fatto PRIMA di aggiornare i valori.
        self.prev_joint_1_pos = self.joint_1_pos
        self.prev_joint_2_pos = self.joint_2_pos
        self.prev_joint_3_pos = self.joint_3_pos
        self.prev_joint_4_pos = self.joint_4_pos
        self.prev_joint_5_pos = self.joint_5_pos
        self.prev_joint_6_pos = self.joint_6_pos

        # Inizio blocco di modifica: Correzione ordine lettura giunti
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
        # Fine blocco di modifica.

        # Trova l'indice della sfera target nel messaggio ModelStates.
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

        # Aggiorna la posizione dell'end-effector (End-Effector) tramite TF2.
        # Determine the pose(position and location) of the end-effector w.r.t. world frame
        self.robot_x, self.robot_y, self.robot_z = self.get_end_effector_transformation()
        
        self.state_updated_flag = True
        
        # self.get_logger().info(f"Pos EF: [{self.robot_x}, {self.robot_y}, {self.robot_z}]")


    # Funzione che utilizza TF2 per ottenere la posa dell'End-Effector ('flange_link') rispetto al frame 'world'.
    def get_end_effector_transformation(self):
        try:
            now = rclpy.time.Time()
            self.reference_frame = 'world'
            self.child_frame     = 'flange_link'
            # Esegue la ricerca della trasformazione con un breve timeout.
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


    def random_point_with_exclusions(self):
        """
        Genera un punto (x, y, z) nel parallelepipedo:
            x ∈ [-0.5, 0.5]
            y ∈ [-0.65, 0.65]
            z ∈ [0.4, 0.75]

        Escludendo le seguenti regioni circolari:
            1) centro (0, 0), r = 0.1 (Asse Z del robot)
            2) centro (-0.25, 0.55), r = 0.06 (Ostacolo 1)
            3) centro (0.6, 0.35), r = 0.06 (Ostacolo 2)
            4) centro (-0.25, -0.433), r = 0.06 (Ostacolo 3)
        """

        forbidden_circles = [
            ((0.0,      0.0),      0.1),
            ((-0.25,    0.55),     0.06), #Commentare se non si usano ostacoli
            ((0.6,      0.35),     0.06), #Commentare se non si usano ostacoli
            ((-0.25,   -0.433),    0.06), #Commentare se non si usano ostacoli
        ]

    

        while True:
            # Generazione del punto casuale
            x = random.uniform(-0.5, 0.5)
            y = random.uniform(-0.65, 0.65)
            z = random.uniform(0.4, 0.75)

            # Controllo se cade dentro una delle regioni proibite
            inside_forbidden = False
            for (cx, cy), r in forbidden_circles:
                if (x - cx)**2 + (y - cy)**2 <= r**2:
                    inside_forbidden = True
                    break

            # Se non è in nessuna regione vietata, lo accettiamo
            if not inside_forbidden:
                return x, y, z
            
    def target_callback(self, msg):
        self.manual_target = [msg.x, msg.y, msg.z]
        


    
    # Gestisce la logica di reset dell'ambiente all'inizio di un nuovo episodio.
    def reset_environment_request(self, timeout=5.0):

        # -------------------- reset sphere position------------------#
        
        if self.manual_target is not None :
            sphere_position_x, sphere_position_y, sphere_position_z = self.manual_target
            self.manual_target = None
        else:
           sphere_position_x, sphere_position_y, sphere_position_z = self.random_point_with_exclusions()

        #sphere_position_x = random.uniform( -0.5, 0.5)  ##vecchia funzione che non tiene conto delle regioni proibite
        #sphere_position_y = random.uniform( 0.65, -0.65)
        #sphere_position_z = random.uniform( 0.4, 0.75)

        self.request_sphere_reset.state.name = 'my_sphere'
        self.request_sphere_reset.state.reference_frame = 'world'
        self.request_sphere_reset.state.pose.position.x = sphere_position_x
        self.request_sphere_reset.state.pose.position.y = sphere_position_y
        self.request_sphere_reset.state.pose.position.z = sphere_position_z
        
        # Invia la richiesta asincrona per spostare la sfera.
        self.future_sphere_reset = self.client_reset_sphere.call_async(self.request_sphere_reset)
        self.get_logger().info('Reseting sphere to new position...')
        # Attende il completamento del servizio di reset della sfera.
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
        # Definisce il punto 'Home' (posizioni a zero) per il reset del robot.
        home_point_msg = JointTrajectoryPoint()
        home_point_msg.positions     = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home_point_msg.velocities    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        home_point_msg.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # Tempo più lungo per un reset sicuro
        home_point_msg.time_from_start = Duration(seconds=0.5).to_msg() 
    
        
        # Inizio blocco di modifica: Correzione ordine giunti
        joint_names = ['shoulder_1_joint','shoulder_2_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        # Fine blocco di modifica.
        
        home_goal_msg = FollowJointTrajectory.Goal()
        home_goal_msg.goal_time_tolerance    = Duration(seconds=0.002).to_msg()
        home_goal_msg.trajectory.joint_names = joint_names
        home_goal_msg.trajectory.points      = [home_point_msg]
        
        if not self.trajectory_action_client.wait_for_server(timeout_sec=5.0): # Timeout ridotto
            self.get_logger().error('Trajectory action server not available!')
            return False # Indica fallimento
        
        # Invia l'obiettivo di traiettoria per riportare il robot a casa.
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
                
                # Inizio blocco di modifica: Reset posizioni precedenti per penalità jerk
                # Azzera le posizioni precedenti dopo il reset per il calcolo corretto del jerk nello step successivo.
                self.prev_joint_1_pos = 0.0
                self.prev_joint_2_pos = 0.0
                self.prev_joint_3_pos = 0.0
                self.prev_joint_4_pos = 0.0
                self.prev_joint_5_pos = 0.0
                self.prev_joint_6_pos = 0.0
                # Fine blocco di modifica.
                return True # Successo
            else:
                self.get_logger().warn(f'Reset robot fallito con codice errore: {result.result.error_code}')
                return False
        except Exception as e:
            self.get_logger().error(f"Eccezione durante il reset del robot: {e}")
            return False


    # Gestisce l'esecuzione di un'azione (delta di posizione) ricevuta dall'agente RL.
    def action_step_service(self, action_values):
        
        # 'action_values' è un array numpy da 6 elementi (es. da -1 a 1)
        # La funzione non restituisce nulla, ma invia un goal di traiettoria al controller.

        self.state_updated_flag = False
        
        points = []
        point_msg = JointTrajectoryPoint()

        # Inizio blocco di modifica: Logica di controllo Delta (Incrementale)
        
        # 1. Prendi la posizione corrente (in ordine corretto)
        current_pos = np.array([
            self.joint_1_pos, self.joint_2_pos, self.joint_3_pos,
            self.joint_4_pos, self.joint_5_pos, self.joint_6_pos
        ])

        # 2. Calcola il target sommando il delta (azione * step_size)
        # Calcola il delta di posizione finale applicando il fattore di scala all'azione ricevuta.
        delta_action = action_values * self.action_step_size
        # Calcola la nuova posizione target sommando il delta alla posizione corrente.
        target_positions = current_pos + delta_action

        # 3. Applica i limiti (Clipping) - FONDAMENTALE per stabilità
        # Limita i target di posizione all'interno dei limiti fisici del giunto (clipping).
        target_positions = np.clip(
            target_positions, 
            self.joint_limits_min, 
            self.joint_limits_max
        )

        # 4. Assegna le posizioni target calcolate
        point_msg.positions = list(map(float, target_positions))
        
        # Fine blocco di modifica.

        # self.get_logger().info(f'TARGET POS: {np.round(point_msg.positions, 2)}')
        
        point_msg.velocities    = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        point_msg.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Inizio blocco di modifica: Riduci il tempo di step (FONDAMENTALE)
        # Imposta un tempo di esecuzione breve per lo step.
        # 4.0 secondi è troppo. Usiamo un tempo breve per step reattivi.
        point_msg.time_from_start = Duration(seconds=0.2).to_msg()
        # Fine blocco di modifica.
        
        points.append(point_msg) 

        # Inizio blocco di modifica: Correzione ordine giunti
        joint_names = ['shoulder_1_joint','shoulder_2_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
        # Fine blocco di modifica.
        
        goal_msg    = FollowJointTrajectory.Goal()
        goal_msg.goal_time_tolerance = Duration(seconds=0.1).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points      = points

        if not self.trajectory_action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Action server non disponibile per lo step!')
            return # Salta questo step
            
        # Invia l'obiettivo di traiettoria per eseguire il passo.
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
        
        
    # Calcola la ricompensa per lo step corrente, usata dall'agente RL.
    def calculate_reward_funct_2(self):
        # ... (Questa funzione è invariata e ora dovrebbe funzionare) ...
        
       
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
            # Calcola la distanza euclidea tra l'End-Effector e la sfera target.
            distance = np.linalg.norm(robot_end_position - target_point_position)
            print ("###----DISTANZA---", distance)

            # Criterio di successo: la distanza è inferiore a una soglia.
            if distance <= 0.06:
                self.get_logger().info('### Goal Reached ###')
                done = True
                reward_d = reward_scale * 1
            else:
                done = False
                reward_d = reward_scale * -1


        return reward_d, done


    # Crea il vettore di stato (osservazione) finale per l'agente RL (12 elementi).
    def state_space_funct(self):
    # ... (Questa funzione è invariata) ...
        try:
            # Vettore di stato contenente posizione EF, posizioni giunti e posizione target.
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