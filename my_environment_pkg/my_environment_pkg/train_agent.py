import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import gymnasium as gym
import gymnasium_robotics
import numpy as np
import torch
from my_environment_pkg.models.sac_agent import SAC
from my_environment_pkg.utils.model_saver import load_replay_buffer, load_agent, save_agent, save_replay_buffer
from my_environment_pkg.buffers.her_replay_buffer import HERReplayBuffer
from my_environment_pkg.run_environment_2 import MyGymEnv
import csv

from gymnasium.envs.registration import register

# Registrazione dell'ambiente personalizzato in Gymnasium
register(
    id='MyGymEnv',  # Identificativo del tuo ambiente
    entry_point='my_environment_pkg.run_environment_2:MyGymEnv',  # Indica la classe nel modulo
    max_episode_steps=100,  # Imposta il numero massimo di passi per episodio, se necessario
)




def main():
    # Set up environment
    env = gym.make('MyGymEnv')
    obs = env.reset()[0]

    # Stampa la struttura di 'obs' per verificare le dimensioni
    print("Struttura di 'obs':", obs)
    print("Dimensione di 'obs':", obs.shape)

    # Lo stato è l'intera osservazione da 12 elementi
    state_dim = obs.shape[0]  # → 12
    action_dim = env.action_space.shape[0]  # → 6

    print(f"state_dim: {state_dim}, action_dim: {action_dim}")

    log_file = 'checkpoints/train_8/Monitoraggio.csv'

    # Device setup
    # device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    device = "cpu"

    # Initialize SAC agent
    sac = SAC(state_dim, action_dim, device=device)
    save_dir = "checkpoints/train_8/"
    os.makedirs(save_dir, exist_ok=True)


    #####---------PER RIPRENDERE L'ALLENAMENTO-------
    checkpoint_episode = 15350    # numero da cui vuoi ripartir
    agent_path = os.path.join(save_dir, f"sac_her_fetchreach_{checkpoint_episode}_train_delta_100steps_sparso_0_5.pth")
    replay_buffer_path = os.path.join(save_dir, f"replay_buffer_{checkpoint_episode}train_delta_100steps_sparso_0_5.pkl")

    if os.path.exists(agent_path) and os.path.exists(replay_buffer_path):
        print(f"Caricamento modello e buffer dal checkpoint {checkpoint_episode}...")
        sac = load_agent(sac, agent_path, device=device)
        sac.replay_buffer = load_replay_buffer(sac.replay_buffer, replay_buffer_path)
        start_episode = checkpoint_episode
    else:
        print("Nessun checkpoint trovato, si parte da zero.")
        start_episode = 0

    episode_reward = 0
    success_count = 0

    # Set hyperparameters
    max_episodes = 20000 # max number of episodes to stop training
    episode_length = env._max_episode_steps # the default is 50
    batch_size = 256
    num_random_episodes = batch_size
    save = True

    for episode in range(start_episode,max_episodes):
        obs, _ = env.reset()
        trajectory = []

        for t in range(episode_length):
            print('Step ', t+1, '-- Episodio ', episode + 1)
            # Create the state input by concatenating observation and desired goal
            state = obs

            # Select action
            action = sac.select_action(state)

            # Step in the environment
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            if reward > 0:
                success_count += 1
                print('#### ------ SUCCESS ----- ####')
            print('Reward step:', reward)
        
            # Append transition to trajectory
            trajectory.append((obs, action, reward, next_obs, done))
            
            obs = next_obs
            episode_reward += reward
     

            if done:
                break

        # Store trajectory in the HER replay buffer
        sac.replay_buffer.store_trajectory(trajectory)

        # Train the SAC agent
        if len(sac.replay_buffer) > num_random_episodes:
            for _ in range(episode_length):
                sac.update(batch_size)

        print(f"Episode {episode+1}, Reward episodio cumulativo: {episode_reward}")

        

        if (episode + 1) % 25 == 0:
            #collisions = env.unwrapped.get_collisions()
            with open(log_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([episode+1,round(episode_reward / 25, 4), success_count, success_count/25])
            #env.unwrapped.set_collisions(0)
            episode_reward = 0
            success_count = 0


        # Define success (if last position is within 0.05 of the goal)
        #success = np.linalg.norm(obs[9:12] - obs[0:3]) < 0.05
        
        # save model in every 1000 episodes
        if save and (episode + 1) % 50 == 0:
            agent_path = os.path.join(save_dir, f"sac_her_fetchreach_{episode + 1}_train_delta_100steps_sparso_0_5.pth")
            replay_buffer_path = os.path.join(save_dir, f"replay_buffer_{episode + 1}train_delta_100steps_sparso_0_5.pkl")
            save_agent(sac, agent_path)
            save_replay_buffer(sac.replay_buffer, replay_buffer_path) # in case want to train further later
            print("Model and replay buffer saved at episode:", episode + 1)
        
    print("Training completed.")

if __name__ == "__main__":
    main()
