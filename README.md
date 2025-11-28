# README IN FASE DI SCRITTURA

# CODICI PER AVVIO TRAIN E TEST

**PREPARAZIONE TERMINALI**

  source /opt/ros/foxy/setup.bash
  
  colcon build && install/setup.bash
  
  (da fare in due terminali diversi)


**Configurazione file per aggiornamento dinamico di home_position, aggiornamento della target_position, comandi di pause e resume**

Seguire le istruzioni presenti all'interno dei seguenti file (da fare solo una volta):

- How_to_configure_robot_home.txt
- How_to_configure_robot_target.txt
- How_to_configure_robot_pause_resume.txt

Dentro i file sono presenti anche le istruzioni per l'utilizzo dei comandi.

**AVVIO GAZEBO** (nel PRIMO terminale)

ros2 launch my_environment_pkg my_environment.launch.py (*senza ostacoli*)

ros2 launch my_environment_pkg my_environment_obs.launch.py (*con ostacoli*)

**AVVIO TRAIN/TEST** (nel SECONDO terminale)

- per il *train*:  ros2 run my_environment_pkg run_environment
  
- per il *test*:   ros2 run my_environment_pkg test_agent


nei file train_agent.py e test_agent.py bisogna inserire il path dei pesi che si desidera usare. I pesi si trovano nella cartella esterna "checkpoints/train_8", dove si trova anche il file "Monitoraggio.csv" che contiene lo storico dell'evoluzione dell'apprendimento del robot.

Per **plottare** i grafici dell'apprendimento, andare in my_environment_pkg/plot_train.py ed eseguire il codice (modificare il file del Monitoraggio.csv da garficare).
