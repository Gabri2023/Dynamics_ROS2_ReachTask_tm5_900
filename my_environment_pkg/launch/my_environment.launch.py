'''
  Questo file di lancio (launch file) ROS 2 ha il compito di 
  avviare un ambiente di simulazione completo. 
  
  Cosa fa:
  1. Avvia Gazebo caricando un mondo specifico ("my_world.world").
  2. Lancia il file di lancio del robot Doosan (che avvia il robot e i suoi controller).
  3. Lancia il file di lancio della "sfera" (che probabilmente spawna la sfera 
     in Gazebo e avvia il nodo 'marker').
  
  In sintesi, orchestra l'avvio di Gazebo, del robot e di un oggetto
  (sfera) nell'ambiente.
'''

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess


# Definisce la funzione principale che 'ros2 launch' cercherà ed eseguirà.
def generate_launch_description():

    # 1. Trova i percorsi dei pacchetti necessari
    # Trova il percorso della cartella 'share' del pacchetto della sfera.
    my_sphere_files       = get_package_share_directory('my_sphere_pkg')
    # Trova il percorso della cartella 'share' del pacchetto del robot Doosan.
    my_doosan_robot_files = get_package_share_directory('tm5_900') 

    # 2. Prepara il lancio del Robot Doosan
    # 'IncludeLaunchDescription' è un'azione che permette di lanciare *un altro* file di lancio.
    # Stiamo includendo il file 'my_doosan_controller.launch.py' dal pacchetto 'tm5_900'.
    doosan_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(my_doosan_robot_files + '/launch/my_tm5_900_controller.launch.py')
    ) 
    
    # 3. Prepara il lancio della Sfera
    # Allo stesso modo, includiamo il file 'my_sphere.launch.py' dal pacchetto 'my_sphere_pkg'.
    # Questo (probabilmente) avvia sia lo spawn della sfera in Gazebo sia il nodo del marker.
    sphere_mark  = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(my_sphere_files + '/launch/my_sphere.launch.py')
    ) 

    # 4. Prepara l'avvio di Gazebo
    # Definisce il nome del file del mondo da caricare.
    world_file_name = 'my_world.world'
    # Trova il percorso assoluto del file .world nel pacchetto 'my_environment_pkg'.
    world = os.path.join(get_package_share_directory('my_environment_pkg'), 'worlds', world_file_name)
    
    # 'ExecuteProcess' è un'azione che esegue un comando nel terminale.
    # È il metodo standard per avviare l'eseguibile di Gazebo (non un nodo ROS).
    gazebo_node = ExecuteProcess(
        # Il comando completo da eseguire nel terminale
        cmd=['gazebo', '--verbose', world,'-s', 'libgazebo_ros_factory.so'], 
        # Mostra l'output di Gazebo direttamente nel terminale.
        output='screen'
    )
    # Spiegazione 'cmd':
    # 'gazebo': Il comando per avviare Gazebo.
    # '--verbose': Mostra più output di debug.
    # 'world': Il percorso al file .world che abbiamo trovato prima.
    # '-s', 'libgazebo_ros_factory.so': Carica il plugin ROS (factory) per permettere 
    #                                  a ROS 2 di comunicare con Gazebo (es. per spawnare i modelli).


    # Spazio per futuri nodi (attualmente vuoto)
    # Node 


    # 5. Assembla il Launch Description
    # Inizializza l'oggetto LaunchDescription vuoto, che conterrà tutte le azioni.
    ld = LaunchDescription()

    # Aggiunge le tre azioni (processi) che abbiamo definito alla lista di avvio.
    # ROS 2 le avvierà in parallelo.
    ld.add_action (doosan_robot) # Avvia il robot e i controller
    ld.add_action (sphere_mark)  # Avvia lo spawn della sfera e il marker
    ld.add_action (gazebo_node)  # Avvia la simulazione Gazebo

    # Restituisce l'oggetto LaunchDescription completo a 'ros2 launch' per l'esecuzione.
    return ld