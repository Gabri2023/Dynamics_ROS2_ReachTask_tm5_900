import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy

from my_environment_pkg.main_rl_environment_3 import MyRLEnvironmentNode 


class MyGymEnv(gym.Env):
    def __init__(self):
        super().__init__()
        
        self.collisions=0
        self.steps=1

        # Inizializza ROS2
        rclpy.init(args=None)
        self.node = MyRLEnvironmentNode()

        # Azione: 6 joint angles
        self.action_space = spaces.Box(low=np.array([-3.1, -1.55, -2.5, -0.0, -0.0,  -0.0]), #cambiati a 3900
                                       high=np.array([3.1, 1.55, 2.5, 0.0, 0.0, 0.0]),
                                       dtype=np.float32)

        # Osservazione: posizione effettore, posizioni giunti, posizione bersaglio
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)
        
    
    
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        print('RILEVATE', self.collisions, ' COLLISIONI.')

        try:
            self.node.reset_environment_request()
            while not self.node.state_updated_flag:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            obs = self.node.state_space_funct()
        except Exception as e:
            print("[ERROR] Reset fallito:", e)
            obs = None

        # fallback se obs non valido
        if obs is None or not isinstance(obs, (list, np.ndarray)) or np.any(np.isnan(obs)):
            print("[WARN] Oss. invalida in reset(), uso array di zeri.")
            obs = np.zeros(12, dtype=np.float32)


        self.prev_joint_1_pos = 0.0
        self.prev_joint_2_pos = 0.0
        self.prev_joint_3_pos = 0.0
        self.prev_joint_4_pos = 0.0
        self.prev_joint_5_pos = 0.0
        self.prev_joint_6_pos = 0.0

    
        self.node.collision = False
        return np.array(obs, dtype=np.float32), {}


    def step(self, action):
    
        self.node.action_step_service(action)
        while not self.node.state_updated_flag:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        obs = self.node.state_space_funct()

        reward, done = self.node.calculate_reward_funct_2()

        #reward+=-0.001/self.steps   da inserire alla fine per indurre l'algoritmo a minimizzare i tempi

        if self.node.collision:
            reward += -0.4 #penalità
            self.collisions +=1
            print('######----AVVENUTA COLLISIONE!!')
            #done=True

        
        #self.steps +=1  anche questo
           
        info = {}
        return np.array(obs, dtype=np.float32), reward, done, False, info

    def render(self, mode="human"):
        pass  # opzionale

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def get_collisions(self):
        print('Reading value', self.collisions)
        return self.collisions
    
    def set_collisions(self, val ):
        print('Setting value to', val)
        self.collisions = val
    
        

from my_environment_pkg.train_agent import main

def run():
    main()

if __name__ == "__main__":
    run()

