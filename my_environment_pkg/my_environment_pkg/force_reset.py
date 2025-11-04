from my_environment_pkg.run_environment_2 import MyGymEnv
import sys

env = MyGymEnv()

# Esegui reset
env.step()
print("Reset eseguito. Stato osservato:", obs)

# Chiudi ROS2 pulitamente
env.close()