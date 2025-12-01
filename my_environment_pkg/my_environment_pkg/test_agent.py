"""
Test di un Agente SAC Addestrato (Reinforcement Learning)

Questo script testa le performance di un agente SAC addestrato su un braccio robotico.
Vengono eseguiti N episodi, calcolando ricompense, distanza dal target e logging dei giunti.
"""

import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import gymnasium as gym
import numpy as np
from .models.sac_agent import SAC
from .utils.model_saver import load_agent
from .run_environment import MyGymEnv
import csv

def main():
    # -------------------- Configurazione del file CSV --------------------
    num_test = 1
    log_file = f'checkpoints/train_8/Test_{num_test}.csv'

    # -------------------- Inizializzazione Ambiente --------------------
    env = gym.make('MyGymEnv')
    obs, _ = env.reset()
    state_dim = obs.shape[0]
    action_dim = env.action_space.shape[0]

    # -------------------- Caricamento modello SAC --------------------
    epoche = 15400
    device = "cpu"
    dir = "checkpoints/train_8/"
    model_path = os.path.join(dir, f"sac_her_fetchreach_{epoche}_train_delta_100steps_sparso_0_5.pth")

    sac = SAC(state_dim, action_dim, device=device)
    sac = load_agent(sac, model_path, device)
    print(f"Loaded model from {model_path}")

    # -------------------- Parametri di test --------------------
    num_episodes = 30
    episode_length = 100
    Reward_global = 0
    success_count = 0

    # -------------------- Creazione header CSV --------------------
    with open(log_file, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "Epoche_modello",
            "Num_episodio_attuale",
            "Num_episodi_da_fare",
            "Dist_EE_target_iniziale",
            "Dist_EE_target_attuale",
            "X_target",
            "Y_target",
            "Z_target",
            "Numero_step",
            "Ricompensa_episodio_attuale",
            "Conteggio_successi_globale",
            "Tasso_successo_globale",
            "Angolo_primo_giunto",
            "Angolo_secondo_giunto",
            "Angolo_terzo_giunto",
            "Angolo_quarto_giunto",
            "Angolo_quinto_giunto",
            "Angolo_sesto_giunto",
            "Stato_home_position",
            "Target_raggiunto"
        ])

    # -------------------- Ciclo principale di test --------------------
    for episode in range(num_episodes):
        obs, _ = env.reset()
        episode_reward = 0
        Reached = False

        for t in range(episode_length):
            print("Step: ", t+1)
            # Legge angoli dei giunti dall'osservazione
            posizioni_giunti = [float(f"{p:.4f}") for p in obs[3:9]]
            target = [float(f"{p:.4f}") for p in obs[9:12]]

            # Calcola distanze dall'end-effector al target
            if t == 0:
                dist_iniziale = np.linalg.norm(obs[9:12] - obs[0:3])
                dist_attuale = dist_iniziale
                Home_pos = True
            if t > 0:
                dist_attuale = np.linalg.norm(obs[9:12] - obs[0:3])
                Home_pos = False

            # Seleziona azione tramite agente SAC
            state = obs
            action = sac.select_action(state)

            # Esegue lo step nell'ambiente
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # Aggiorna stato e ricompensa cumulativa
            obs = next_obs
            episode_reward += reward

            # Verifica se il target è stato raggiunto
            if np.linalg.norm(obs[9:12] - obs[0:3]) < 0.06:
                Reached = True
                success_count += 1

            with open(log_file, mode='a', newline="") as f:
                writer = csv.writer(f)
                writer.writerow(([
                    epoche,
                    episode+1,
                    num_episodes,
                    float(f"{dist_iniziale:.4f}"),
                    float(f"{dist_attuale:.4f}"),
                    *target,
                    t+1,
                    float(f"{episode_reward:.4f}"),
                    success_count,
                    float(f"{success_count/num_episodes:.4f}"),
                    *posizioni_giunti,
                    Home_pos,
                    Reached
                ]))

            if done:
                break
            
        
        Reward_global += episode_reward

        print(f"Episode {episode+1}, Reward: {episode_reward}")

if __name__ == "__main__":
    main()
