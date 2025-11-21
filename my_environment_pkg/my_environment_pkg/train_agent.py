"""
Riassunto del Codice: Ciclo di Addestramento SAC + HER

Questo script implementa il ciclo di addestramento principale per un agente di Reinforcement
Learning che utilizza l'algoritmo **Soft Actor-Critic (SAC)** con **Hindsight Experience
Replay (HER)**. Si interfaccia con l'ambiente robotico personalizzato 'MyGymEnv' (collegato
a ROS 2/Gazebo).

Il codice gestisce:
1.  La registrazione dell'ambiente personalizzato in Gymnasium.
2.  L'inizializzazione dell'agente SAC e la creazione della directory di salvataggio.
3.  La logica per riprendere l'addestramento da un checkpoint esistente (modello e buffer).
4.  Il ciclo principale di addestramento, che include la raccolta delle traiettorie, l'aggiornamento
    del buffer di replay HER e l'ottimizzazione del modello SAC.
5.  Il logging delle performance (reward cumulativo medio e successi) in un file CSV.
"""

import sys
import os
# Aggiunge la directory genitore al PYTHONPATH per risolvere gli import interni al pacchetto.
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

import gymnasium as gym
# Importa l'implementazione dell'agente SAC.
from my_environment_pkg.models.sac_agent import SAC
# Importa le funzioni di utilità per il salvataggio/caricamento.
from my_environment_pkg.utils.model_saver import load_replay_buffer, load_agent, save_agent, save_replay_buffer
import csv

from gymnasium.envs.registration import register

# Registrazione dell'ambiente personalizzato in Gymnasium.
# Questo permette di usare `gym.make('MyGymEnv')`.
register(
    id='MyGymEnv',  # Identificativo dell'ambiente
    entry_point='my_environment_pkg.run_environment:MyGymEnv',  # Percorso della classe dell'ambiente
    max_episode_steps=100,  # Numero massimo di passi per episodio (usato per il limite di tempo)
)


def main():
    # -------------------- 1. Setup Ambiente --------------------
    # Crea un'istanza dell'ambiente registrato.
    env = gym.make('MyGymEnv')
    # Resetta l'ambiente e ottiene l'osservazione iniziale.
    obs = env.reset()[0]

    # Stampa la struttura di 'obs' per debug e verifica delle dimensioni.
    print("Struttura di 'obs':", obs)
    print("Dimensione di 'obs':", obs.shape)

    # Dimensione dello stato (osservazione, es. 12 elementi).
    state_dim = obs.shape[0]
    # Dimensione dell'azione (es. 6 giunti).
    action_dim = env.action_space.shape[0]

    print(f"state_dim: {state_dim}, action_dim: {action_dim}")

    # Percorso del file CSV per il monitoraggio e logging dei risultati.
    log_file = 'checkpoints/train_8/Monitoraggio.csv'

    # -------------------- 2. Setup Agente e Checkpoint --------------------
    # Imposta il dispositivo su CPU (può essere 'cuda' se disponibile).
    device = "cpu"

    # Inizializza l'agente SAC.
    sac = SAC(state_dim, action_dim, device=device)
    # Definisce la directory di salvataggio e la crea se non esiste.
    save_dir = "checkpoints/train_8/"
    os.makedirs(save_dir, exist_ok=True)


    # Parametri del checkpoint per riprendere l'addestramento.
    checkpoint_episode = 15350    # Episodio dal quale ripartire
    agent_path = os.path.join(save_dir, f"sac_her_fetchreach_{checkpoint_episode}_train_delta_100steps_sparso_0_5.pth")
    replay_buffer_path = os.path.join(save_dir, f"replay_buffer_{checkpoint_episode}train_delta_100steps_sparso_0_5.pkl")

    # Logica per il caricamento del checkpoint.
    if os.path.exists(agent_path) and os.path.exists(replay_buffer_path):
        print(f"Caricamento modello e buffer dal checkpoint {checkpoint_episode}...")
        sac = load_agent(sac, agent_path, device=device)
        sac.replay_buffer = load_replay_buffer(sac.replay_buffer, replay_buffer_path)
        start_episode = checkpoint_episode
    else:
        print("Nessun checkpoint trovato, si parte da zero.")
        start_episode = 0

    episode_reward = 0  # Variabile per accumulare la ricompensa nell'intervallo di logging.
    success_count = 0   # Contatore per i successi nell'intervallo di logging.

    # -------------------- 3. Hyperparameters e Ciclo di Training --------------------
    max_episodes = 20000              # Numero massimo di episodi di addestramento.
    episode_length = env._max_episode_steps # Lunghezza massima dell'episodio (100 step).
    batch_size = 256                  # Dimensione del batch per l'aggiornamento del modello.
    num_random_episodes = batch_size  # Numero minimo di step nel buffer prima di iniziare l'addestramento.
    save = True                       # Flag per abilitare il salvataggio periodico.

    # Ciclo principale di addestramento.
    for episode in range(start_episode, max_episodes):
        # Resetta l'ambiente per un nuovo episodio.
        obs, _ = env.reset()
        # Lista per memorizzare la traiettoria corrente dell'episodio (necessaria per HER).
        trajectory = []

        # Ciclo per gli step all'interno dell'episodio.
        for t in range(episode_length):
            print('Step ', t+1, '-- Episodio ', episode + 1)
            # Lo stato è l'osservazione corrente.
            state = obs

            # Selezione dell'azione: l'agente campiona dalla sua policy (stocastica in SAC).
            action = sac.select_action(state)

            # Esegue lo step nell'ambiente.
            next_obs, reward, terminated, truncated, _ = env.step(action)
            done = terminated or truncated

            # Verifica e conteggio del successo (basato sulla ricompensa > 0).
            if reward > 0:
                success_count += 1
                print('#### ------ SUCCESS ----- ####')
            print('Reward step:', reward)

            # Aggiunge la transizione (stato, azione, ricompensa, stato successivo, done) alla traiettoria.
            trajectory.append((obs, action, reward, next_obs, done))

            # Aggiorna lo stato e la ricompensa cumulativa dell'episodio corrente.
            obs = next_obs
            episode_reward += reward

            if done:
                break

        # Memorizza la traiettoria completata nel Replay Buffer, applicando HER.
        sac.replay_buffer.store_trajectory(trajectory)

        # -------------------- 4. Aggiornamento dell'Agente --------------------
        # Addestra l'agente solo se il buffer ha raggiunto una dimensione minima.
        if len(sac.replay_buffer) > num_random_episodes:
            # Esegue un numero di aggiornamenti del modello pari alla lunghezza dell'episodio.
            for _ in range(episode_length):
                sac.update(batch_size)

        print(f"Episode {episode+1}, Reward episodio cumulativo: {episode_reward}")

        # -------------------- 5. Logging e Salvataggio --------------------
        # Logga i risultati ogni 25 episodi.
        if (episode + 1) % 25 == 0:

            # Scrive i dati: [Episodio, Reward Medio negli ultimi 25, Successi totali, Tasso di Successo (su 25)]
            with open(log_file, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([episode+1, round(episode_reward / 25, 4), success_count, success_count/25])
            # Reset dei contatori per il prossimo intervallo di 25 episodi.
            episode_reward = 0
            success_count = 0


        # Salva il modello e il buffer ogni 50 episodi.
        if save and (episode + 1) % 50 == 0:
            agent_path = os.path.join(save_dir, f"sac_her_fetchreach_{episode + 1}_train_delta_100steps_sparso_0_5.pth")
            replay_buffer_path = os.path.join(save_dir, f"replay_buffer_{episode + 1}train_delta_100steps_sparso_0_5.pkl")
            save_agent(sac, agent_path)
            save_replay_buffer(sac.replay_buffer, replay_buffer_path)
            print("Model and replay buffer saved at episode:", episode + 1)

    print("Training completed.")

if __name__ == "__main__":
    main()