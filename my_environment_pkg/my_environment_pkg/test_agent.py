import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import gymnasium as gym
import gymnasium_robotics
import numpy as np
import torch

from .models.sac_agent import SAC
from .utils.model_saver import load_agent

from .run_environment_2 import MyGymEnv

import csv
import os



from gymnasium.envs.registration import register

# Registrazione dell'ambiente personalizzato in Gymnasium


 

'''if not os.path.exists(log_file):
    with open(log_file, mode='a', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["Train_episodes", "Reward", "Success"])'''

# Set up environment and testing parameters
def main():

    log_file = 'checkpoints/train_8/Test.csv' 

    # Environment setup
    env = gym.make('MyGymEnv')
    obs,_ = env.reset()
    state_dim = obs.shape[0]  # → 12
    print("DEBUG obs type:", type(obs))
    print("DEBUG obs value:", obs)
    action_dim = env.action_space.shape[0]  # → 6

    # Device setup
    # device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    num_ep=2650
    device = "cpu"
    model_path = f"/home/gabri/dynamics/robotic_arm_environment/checkpoints/train_8/sac_her_fetchreach_{num_ep}_train_delta_100steps_sparso_0_5.pth"

    # Initialize the SAC agent
    sac = SAC(state_dim, action_dim, device=device)

    # Load the trained model
    sac = load_agent(sac, model_path, device)
    print(f"Loaded model from {model_path}")

    # Testing parameters
    num_episodes = 25
    episode_length = 100
    Reward_global=0
    success_count=0

    for episode in range(num_episodes):
        obs, _ = env.reset()
        episode_reward = 0
        
        for t in range(episode_length):
            print('è il numero di step', t+1)
            # Prepare state
            state = obs

            # Select action
            action = sac.select_action(state)

            # Step in the environment
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # Update state and reward
            obs = next_obs
            episode_reward += reward

            if done:
                break
        
        if (np.linalg.norm(obs[9:12] - obs[0:3]) < 0.15):
             success_count +=1

        Reward_global += episode_reward
        print(f"Episode {episode}, Reward: {episode_reward}")
    
    with open(log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([num_ep,Reward_global/num_episodes, success_count, 25/success_count ])

if __name__ == "__main__":
    main()
