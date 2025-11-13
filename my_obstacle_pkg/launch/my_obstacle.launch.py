'''
  Questo file di lancio ROS 2 (Python) ha il compito di caricare 
  un modello 3D in Gazebo. Cerca un file SDF specifico (model.sdf)
  nel pacchetto my_obstacle_pkg e usa lo script spawn_entity.py per 
  caricarlo nella simulazione. Il modello verrà chiamato my_obstacle 
  all'interno di Gazebo e posizionato all'origine (0,0,0).

'''

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory


# Definisce la funzione principale che 'ros2 launch' cercherà ed eseguirà.
# Tutto ciò che vuoi lanciare deve essere definito e restituito da qui.
def generate_launch_description():

    # 1. Trova il percorso del pacchetto
    # Cerca il percorso assoluto della cartella 'share' del pacchetto 'my_obstacle_pkg'.
    # Esempio: /home/tuo_utente/ros2_ws/install/my_obstacle_pkg/share/my_obstacle_pkg
    pkg_dir = get_package_share_directory('my_obstacle_pkg') 

    # 2. Definisce il percorso del modello SDF
    # Definisce il percorso *relativo* del tuo file SDF all'interno della cartella 'share'
    # del pacchetto che hai trovato sopra.
    # Nota: L'utente ha inserito 'models' nel 'os.path.join' sotto, quindi qui
    # il percorso è relativo alla cartella 'models'.
    sdf_file_name = 'sdf/obstacle_goal/model.sdf'
    
    # Costruisce il percorso *assoluto* e completo al file 'model.sdf'.
    # os.path.join è intelligente: unisce i percorsi con il giusto separatore ('/').
    # Risultato: /home/.../share/my_obstacle_pkg/models/sdf/obstacle_goal/model.sdf
    sdf = os.path.join(pkg_dir, 'models', sdf_file_name)
    
    # 3. Definisce l'azione per spawnare (caricare) il modello
    # 'spawn_entity' è un'istanza della classe 'Node' che abbiamo importato.
    # Stiamo definendo un'azione per avviare un nodo specifico.
    spawn_entity = Node(
        # Specifica il pacchetto ROS 2 che contiene l'eseguibile: 'gazebo_ros'
        package='gazebo_ros', 
        
        # L'eseguibile specifico da lanciare: lo script 'spawn_entity.py'
        # Questo script è progettato per aggiungere modelli a Gazebo.
        executable='spawn_entity.py', 
        
        # Questi sono gli argomenti passati a 'spawn_entity.py'
        arguments=[
            '-entity', 'my_obstacle',  # Assegna il nome 'my_obstacle' al modello dentro Gazebo
            '-file', sdf,              # Specifica il file SDF da caricare (usa la variabile 'sdf' col percorso assoluto)
            '-x', '0',                 # La posizione X di spawn nel mondo
            '-y', '0',                 # La posizione Y di spawn nel mondo
            '-z', '0'                  # La posizione Z di spawn nel mondo
        ], 
        
        # Indirizza l'output (log, errori, print) di questo nodo
        # direttamente al terminale da cui hai lanciato il file.
        output='screen'
    )

    # 4. Restituisce la "lista di cose da fare"
    # Crea e restituisce un 'LaunchDescription' che contiene la lista di tutte
    # le azioni da eseguire. In questo caso, c'è solo un'azione: spawn_entity.
    return LaunchDescription([
        spawn_entity
    ])