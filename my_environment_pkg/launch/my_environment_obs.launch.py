'''
  Questo file di lancio (launch file) ROS 2 orchestra l'avvio di 
  un ambiente di simulazione Gazebo completo.
  
  Cosa fa:
  1. Avvia Gazebo caricando un mondo specifico ("my_world.world").
  2. Include il file di lancio per avviare il robot Doosan (tm5_900) e i suoi controller.
  3. Include il file di lancio per spawnare la sfera (my_sphere_pkg).
  4. Include il file di lancio per spawnare gli ostacoli (my_obstacle_pkg).
  
  In sintesi, avvia Gazebo, il robot, la sfera e gli ostacoli
  contemporaneamente.
'''

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess

from launch_ros.actions import Node


# Definisce la funzione principale che 'ros2 launch' cercherà ed eseguirà.
def generate_launch_description():

    # 1. Trova i percorsi dei pacchetti necessari
    # Trova il percorso della cartella 'share' del pacchetto della sfera.
    my_sphere_files       = get_package_share_directory('my_sphere_pkg')
    # Trova il percorso della cartella 'share' del pacchetto del robot Doosan.
    my_doosan_robot_files = get_package_share_directory('tm5_900')
    # Trova il percorso della cartella 'share' del pacchetto degli ostacoli.
    my_obstacle_files = get_package_share_directory('my_obstacle_pkg')


    # 2. Prepara il lancio del Robot Doosan
    # 'IncludeLaunchDescription' è un'azione che permette di lanciare *un altro* file di lancio.
    # Stiamo includendo il file 'my_doosan_controller.launch.py' dal pacchetto 'tm5_900'.
    doosan_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(my_doosan_robot_files + '/launch/my_doosan_controller.launch.py')
    ) 
    
    # 3. Prepara il lancio della Sfera
    # Allo stesso modo, includiamo il file 'my_sphere.launch.py' dal pacchetto 'my_sphere_pkg'.
    sphere_mark  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(my_sphere_files + '/launch/my_sphere.launch.py')
    ) 
 
    # 4. Prepara il lancio degli Ostacoli
    # Includiamo il file 'my_obstacle.launch.py' dal pacchetto 'my_obstacle_pkg'.
    spawn_obstacles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(my_obstacle_files + '/launch/my_obstacle.launch.py')
    ) 
 

    # 5. Prepara l'avvio di Gazebo
    # Definisce il nome del file del mondo da caricare.
    world_file_name = 'my_world.world'
    # Trova il percorso assoluto del file .world nel pacchetto 'my_environment_pkg'.
    world = os.path.join(get_package_share_directory('my_environment_pkg'), 'worlds', world_file_name)
    
    # 'ExecuteProcess' è un'azione che esegue un comando nel terminale.
    # È il metodo standard per avviare l'eseguibile di Gazebo (non un nodo ROS).
    gazebo_node = ExecuteProcess(
        # Il comando completo da eseguire: avvia 'gazebo', in modo 'verbose', 
        # carica il file 'world' specifico, e carica il plugin 'libgazebo_ros_factory.so'
        # (necessario per permettere a ROS 2 di spawnare modelli in Gazebo).
        cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], 
        # Mostra l'output di Gazebo direttamente nel terminale.
        output='screen'
    )

    Node(
    package='ros_ign_bridge',
    executable='parameter_bridge',
    name='contacts_bridge',
    arguments=[
        '/gazebo/default/physics/contacts@ignition.msgs.Contacts@ros_ign_interfaces/msg/Contacts'
    ],
    output='screen'
)

        # Esempio: il tuo nodo listener RL
    Node(
            package='my_rl_pkg',
            executable='contact_listener',
            output='screen')
                


    # Spazio per futuri nodi (attualmente vuoto)
    # Node 


    # 6. Assembla il Launch Description
    # Inizializza l'oggetto LaunchDescription vuoto, che conterrà tutte le azioni.
    ld = LaunchDescription()

    # Aggiunge le quattro azioni (processi) che abbiamo definito alla lista di avvio.
    # ROS 2 le avvierà in parallelo.
    ld.add_action (doosan_robot)
    ld.add_action (sphere_mark)
    ld.add_action (gazebo_node)
    ld.add_action (spawn_obstacles)
    
    # Azione commentata per avviare RViz (attualmente non usata)
    #ld.add_action (rviz_node)

    # Restituisce l'oggetto LaunchDescription completo a 'ros2 launch' per l'esecuzione.
    return ld