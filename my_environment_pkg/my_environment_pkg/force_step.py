import rclpy
import numpy as np
from my_environment_pkg.run_environment_2 import MyGymEnv   # importa la tua classe

def main():
    env = MyGymEnv()   # inizializza
    env.reset()        # fai un reset prima dello step

    # Definisci un'azione manuale (6 giunti)
    action = np.array([0.2, -0.1, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)

    obs, reward, done, _, info = env.step(action)

    print("Step eseguito ✅")
    print("Azione inviata:", action)
    print("Nuova osservazione:", obs)
    print("Reward:", reward, "Done:", done)

    env.close()

if __name__ == "__main__":
    main()