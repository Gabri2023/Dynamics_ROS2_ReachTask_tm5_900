# README IN FASE DI SCRITTURA

# CODICI PER AVVIO TRAIN E TEST

source /opt/ros/foxy/setup.bash
colcon build && install/setup.bash
(da fare in due terminali)

ros2 launch my_environment_pkg my_environment.launch.py

- per il train (nell'altro terminale) ros2 run my_environment_pkg run_environment_2
- per il test  (nell'altro terminale) ros2 run my_environment_pkg test_agent


nei file train_agent.py e test_agent.py bisogna inserire il path dei pesi che si desidera usare. I pesi si trovano nella cartella esterna "checkpoints/train_8", dove si trova anche il file "Monitoraggio.csv" che contiene lo storico dell'evoluzione dell'apprendimento del robot.

Per plottare i grafici dell'apprendimento, andare in my_environment_pkg/plot_train.py ed eseguire il codice.