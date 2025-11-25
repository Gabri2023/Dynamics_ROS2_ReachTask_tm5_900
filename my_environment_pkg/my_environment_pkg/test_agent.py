"""
Riassunto del Codice: Testing di un Agente SAC Addestrato (Reinforcement Learning)

Questo script è progettato per testare le performance di un agente di Reinforcement Learning
che utilizza l'algoritmo **Soft Actor-Critic (SAC)**, precedentemente addestrato per
controllare un braccio robotico (l'ambiente 'MyGymEnv').

Funzionalità Principali:
1.  **Configurazione:** Carica l'ambiente di simulazione personalizzato (ROS 2/Gazebo) e l'agente SAC.
2.  **Caricamento Modello:** Carica i pesi di un modello SAC addestrato da un file '.pth'.
3.  **Testing:** Esegue un numero definito di episodi di test (num_episodes), raccogliendo lo stato e selezionando
    l'azione tramite la policy appresa dall'agente.
4.  **Valutazione:** Calcola la ricompensa totale media e il tasso di successo (Goal Reached)
    basato sul criterio di distanza.
5.  **Logging:** Salva i risultati del test (numero di epoch, ricompensa media, successi) in un file CSV.
"""

import sys
import os
# Aggiunge la directory genitore al percorso di sistema per importare i moduli interni.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import gymnasium as gym
import numpy as np
import torch
# Importa l'implementazione dell'agente SAC (Soft Actor-Critic).
from .models.sac_agent import SAC
# Importa la funzione helper per caricare i pesi del modello.
from .utils.model_saver import load_agent
# Importa l'ambiente Gymnasium personalizzato.
from .run_environment import MyGymEnv
import csv
import os

def main():

    # Percorso del file CSV per il logging dei risultati del test.
    
    #log_file = 'checkpoints/train_8/Test.csv'
    log_file = 'checkpoints/train_8_and_obs/Test.csv'

    # -------------------- 1. Setup Ambiente e Variabili --------------------
    # Inizializzazione dell'ambiente Gymnasium personalizzato (ROS/Gazebo).
    env = gym.make('MyGymEnv')
    # Resetta l'ambiente e ottiene la prima osservazione.
    obs,_ = env.reset()
    # Dimensione dello stato (spazio di osservazione) [es. 12 elementi].
    state_dim = obs.shape[0]
    
    # Dimensione dell'azione (numero di giunti controllabili) [es. 6 elementi].
    action_dim = env.action_space.shape[0]

    # Configurazione del dispositivo (CPU/GPU) per l'agente.
   
    num_ep=18100 # Numero di epoch di addestramento del modello da caricare.
    device = "cpu"

    # Percorso del modello SAC addestrato.
    # Il primo è riferito alla cartella del train senza ostacoli, il secondo con ostacoli

    #model_path = f"/home/gabri/tm5_900/checkpoints/train_8/sac_her_fetchreach_{num_ep}_train_delta_100steps_sparso_0_5.pth"
    model_path = f"/home/gabri/tm5_900/checkpoints/train_8_and_obs/sac_her_fetchreach_{num_ep}_train_delta_100steps_sparso_0_5_obs.pth"

    # -------------------- 2. Caricamento Agente SAC --------------------
    # Inizializza l'agente SAC con le dimensioni corrette.
    sac = SAC(state_dim, action_dim, device=device)

    # Carica i pesi addestrati nell'istanza SAC.
    sac = load_agent(sac, model_path, device)
    print(f"Loaded model from {model_path}")

    # -------------------- 3. Parametri e Ciclo di Testing --------------------
    # Numero totale di episodi da eseguire per il test.
    num_episodes = 50
    # Lunghezza massima di ogni episodio in termini di step.
    episode_length = 100
    # Variabile per accumulare la ricompensa totale di tutti gli episodi.
    Reward_global=0
    # Contatore per i successi (raggiungimento del target).
    success_count=0

    # Ciclo di testing principale.
    for episode in range(num_episodes):
        # Resetta l'ambiente all'inizio di ogni episodio.
        obs, _ = env.reset()
        episode_reward = 0

        for t in range(episode_length):
            print('Numero di step:', t+1)
            # Prepara lo stato (l'osservazione) per l'agente.
            state = obs

            # L'agente seleziona l'azione (utilizzando la policy deterministica o stocastica, a seconda dell'implementazione SAC).
            action = sac.select_action(state)

            # Esegue lo step nell'ambiente con l'azione selezionata.
            next_obs, reward, terminated, truncated, _ = env.step(action)
            # 'done' è True se l'episodio è terminato per 'terminated' (es. successo/fallimento) o 'truncated' (es. limite di tempo).
            done = terminated or truncated

            # Aggiorna lo stato e la ricompensa cumulativa.
            obs = next_obs
            episode_reward += reward

            if done:
                break

        # Criterio di successo: verifica se la distanza tra il target (obs[9:12]) e l'end-effector (obs[0:3])
        # è inferiore alla soglia di 0.06 metri.
        if (np.linalg.norm(obs[9:12] - obs[0:3]) < 0.06):
             success_count +=1

        # Aggiorna la ricompensa globale con la ricompensa dell'episodio corrente.
        Reward_global += episode_reward
        print(f"Episode {episode}, Reward: {episode_reward}")

    # -------------------- 4. Logging dei Risultati --------------------
    # Scrive i risultati finali nel file CSV.
    with open(log_file, mode='a', newline='') as f:
            writer = csv.writer(f)
            # Scrive: [Epoch del modello, Ricompensa Media, Conteggio Successi, Tasso di Successo]
            # Nota: la terza e quarta colonna sono identiche qui (success_count / num_episodes * 100) sarebbe più tipico per il tasso.
            writer.writerow([num_ep, Reward_global/num_episodes, success_count, success_count ])

if __name__ == "__main__":
    main()