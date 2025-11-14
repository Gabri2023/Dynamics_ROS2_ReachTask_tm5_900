'''
  Questo file di lancio ROS 2 (Python) è responsabile dell'avvio 
  del robot Doosan (tm5_900) e dei suoi controller.
  
  Cosa fa:
  1. Trova il file di descrizione del robot (.urdf.xacro).
  2. Esegue il processore 'xacro' per convertire il file .xacro in un URDF valido.
  3. Avvia il nodo 'robot_state_publisher' e gli passa l'URDF generato.
  4. Avvia il nodo 'spawn_entity.py' per caricare (spawnare) il robot in Gazebo,
     dicendogli di prendere la descrizione del robot dal topic 'robot_description'.
  5. Carica e avvia i controller di 'ros2_control': 
     - 'joint_state_broadcaster' (per leggere lo stato dei giunti)
     - 'joint_trajectory_controller' (per inviare comandi di movimento)
'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import Command
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


# Definisce la funzione principale che 'ros2 launch' cercherà ed eseguirà.
def generate_launch_description():

    # Definisce il nome del file del modello robotico.
    robot_model = 'tm5_900_robot'

    # Costruisce il percorso completo al file .urdf.xacro del robot.
    # 'get_package_share_directory' trova la cartella 'share' del pacchetto 'tm5_900'.
    xacro_file = os.path.join(
        get_package_share_directory('tm5_900'),
        'description',
        'desc_tm900',
        'urdf',
        robot_model + '.urdf.xacro'
    )

    
    # 1. Avvia il Robot State Publisher
    # Questo nodo legge l'URDF, elabora i giunti e pubblica le trasformazioni (TF)
    # dello stato del robot (es. dove si trova ogni link).
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',  # Mostra l'output sia su console che su file di log.
        parameters=[{
            # Definisce il parametro 'robot_description'.
            'robot_description': ParameterValue(
                # Il valore non è un file, ma l'output del comando 'xacro'.
                # 'Command' esegue 'xacro' sul file 'xacro_file' e ne cattura
                # l'output (l'URDF completo in formato stringa).
                Command(['xacro ', xacro_file]),
                value_type=str  # Specifica che l'output è una stringa.
            )
        }]
    )


    # 2. Spawna il robot in Gazebo
    # Questo nodo si occupa di inserire il modello del robot nella simulazione Gazebo.
    spawn_entity_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-entity', 'tm5_900',         # Il nome del modello all'interno di Gazebo
            '-topic', 'robot_description' # Dice allo spawner di *ascoltare* sul topic
                                          # 'robot_description' per ottenere l'URDF 
                                          # (che sta venendo pubblicato dal nodo sopra).
        ],
        output='screen'
    )
    
    
    # 3. Carica e AVVIA i controller di ros2_control
    
    # Esegue un comando shell per caricare il 'joint_state_broadcaster'.
    # Questo controller è FONDAMENTALE: legge lo stato reale dei giunti
    # da Gazebo (o dal robot reale) e lo pubblica sul topic /joint_states.
    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_state_broadcaster'],
        output='screen'
    )

    # Esegue un comando shell per caricare il 'joint_trajectory_controller'.
    # Questo controller è FONDAMENTALE: si iscrive ai comandi di traiettoria
    # (es. da MoveIt) e li invia ai giunti del robot in Gazebo (o reali).
    load_joint_trajectory_controller = ExecuteProcess( 
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'], 
        output='screen'
    )


    # 4. Restituisce la lista di tutte le azioni da avviare
    # Vengono avviati tutti e quattro i processi contemporaneamente.
    return LaunchDescription([
        robot_state_publisher,          # Avvia il publisher dell'URDF e dei TF
        spawn_entity_robot,             # Spawna il robot in Gazebo
        load_joint_state_broadcaster,   # Avvia il controller per leggere i giunti
        load_joint_trajectory_controller  # Avvia il controller per comandare i giunti
    ])