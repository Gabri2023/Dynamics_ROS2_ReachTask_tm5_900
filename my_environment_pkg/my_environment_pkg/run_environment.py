"""
Riassunto del Codice: Interfaccia Gymnasium (OpenAI Gym) per l'Ambiente Robotico ROS 2

Questo script definisce la classe **MyGymEnv**, che funge da wrapper tra l'ambiente di
simulazione ROS 2/Gazebo (gestito da MyRLEnvironmentNode) e la struttura standard di
un ambiente di Reinforcement Learning (RL) basato su Gymnasium (precedentemente OpenAI Gym).

Funzionalità Principali:
1.  **Inizializzazione:** Configura il nodo ROS 2, definisce lo spazio delle Azioni (i delta
    di posizione dei 6 giunti) e lo spazio delle Osservazioni (lo stato completo del sistema).
2.  **Reset:** Riporta l'ambiente allo stato iniziale (robot a casa, target riposizionato)
    e restituisce la prima osservazione.
3.  **Step:** Esegue un'azione sul robot, attende l'aggiornamento dello stato in Gazebo,
    calcola la ricompensa, verifica se l'episodio è terminato (done) e restituisce lo stato successivo.
4.  **Gestione Collisioni:** Applica una penalità negativa aggiuntiva in caso di collisione.
"""

import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy

# Importa la classe del nodo ROS 2 che gestisce l'interazione diretta con Gazebo.
from my_environment_pkg.main_rl_environment import MyRLEnvironmentNode


class MyGymEnv(gym.Env):
    """
    Ambiente personalizzato di Gymnasium per l'addestramento RL
    di un braccio robotico tramite ROS 2.
    """
    def __init__(self):
        super().__init__()

        # Contatore per il numero totale di collisioni durante l'addestramento.
        self.collisions=0
        # Contatore per il numero di step eseguiti nell'episodio (attualmente non usato per il reward).
        self.steps=1

        # Inizializza il runtime ROS 2.
        rclpy.init(args=None)
        # Crea un'istanza del nodo ROS 2 di ambiente.
        self.node = MyRLEnvironmentNode()

        # Definisce il valore approssimativo di pi (utilizzato per i limiti dell'azione).
        pi=3,14

        # Spazio delle Azioni (Action Space):
        # Definisce l'intervallo di valori che l'agente può scegliere per ogni giunto ad ogni step.
        # L'array di low/high definisce i limiti del delta di posizione (in radianti) per i 6 giunti.
        # I valori usati corrispondono approssimativamente ai limiti di joint_limits_max/min.
        self.action_space = spaces.Box(low=np.array([-3.14,-1.57,-2.71,-1.57,-1.57,-1.57]),
                                       high=np.array([3.14,1.57,2.71,1.57,1.57,1.57]),
                                       dtype=np.float32)

        # Spazio delle Osservazioni (Observation Space):
        # Definisce la struttura e i limiti dello stato osservabile dall'agente.
        # shape=(12,): [3 posizioni EF, 6 posizioni Giunti, 3 posizioni Target].
        # Viene impostato su limiti infiniti poiché i valori specifici sono gestiti dal nodo ROS.
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(12,), dtype=np.float32)


    def reset(self, seed=None, options=None):
        """
        Resetta l'ambiente all'inizio di un nuovo episodio.
        """
        super().reset(seed=seed)

        # Stampa le collisioni rilevate nell'episodio precedente.
        print('RILEVATE', self.collisions, ' COLLISIONI.')

        try:
            # Chiama la funzione di reset del nodo ROS (resetta sfera e robot in Home).
            self.node.reset_environment_request()
            # Attende che il nodo ROS abbia aggiornato il primo stato dopo il reset.
            while not self.node.state_updated_flag:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            # Ottiene lo stato iniziale del sistema.
            obs = self.node.state_space_funct()
        except Exception as e:
            print("[ERROR] Reset fallito:", e)
            obs = None

        # Gestione del fallback se l'osservazione non è valida.
        if obs is None or not isinstance(obs, (list, np.ndarray)) or np.any(np.isnan(obs)):
            print("[WARN] Oss. invalida in reset(), uso array di zeri.")
            obs = np.zeros(12, dtype=np.float32)

        # Reset delle variabili di stato precedente (per il calcolo del jerk nel nodo ROS).
        # Nota: Questi dovrebbero essere azzerati nel node.reset_environment_request,
        # ma vengono azzerati esplicitamente qui per sicurezza.
        self.prev_joint_1_pos = 0.0
        # ... (azzeramento posizioni precedenti 2-6)
        self.prev_joint_6_pos = 0.0

        # Reset del flag di collisione nel nodo ROS.
        self.node.collision = False

        # Restituisce l'osservazione iniziale e un dizionario 'info' vuoto (come richiesto da Gymnasium).
        return np.array(obs, dtype=np.float32), {}


    def step(self, action):
        """
        Esegue un passo nell'ambiente, dato un vettore di azioni.
        """
        # Invia l'azione (delta di posizione) al nodo ROS che la esegue sul robot.
        self.node.action_step_service(action)

        # Attende l'aggiornamento dello stato del robot dopo l'esecuzione dell'azione.
        while not self.node.state_updated_flag:
            rclpy.spin_once(self.node, timeout_sec=0.01)

        # Ottiene la nuova osservazione (stato) dopo l'azione.
        obs = self.node.state_space_funct()

        # Calcola la ricompensa e verifica se l'episodio è terminato (done).
        reward, done = self.node.calculate_reward_funct_2()

        # Codice commentato per penalità temporale (opzionale):
        # reward+=-0.001/self.steps

        # Gestione della penalità per collisione.
        if self.node.collision:
            reward += -0.4 # Penalità aggiuntiva negativa
            self.collisions +=1
            print('######----AVVENUTA COLLISIONE!!')
            # L'episodio NON viene terminato qui, ma si potrebbe forzare 'done=True'
            # se si desidera terminare immediatamente al primo contatto.

        # Codice commentato per il conteggio degli step (opzionale):
        # self.steps +=1

        # Dizionario 'info' vuoto, come richiesto dall'interfaccia.
        info = {}

        # Restituisce lo stato successivo, la ricompensa, 'done', 'truncated' (False) e 'info'.
        return np.array(obs, dtype=np.float32), reward, done, False, info

    def render(self, mode="human"):
        """
        Funzione di rendering (opzionale), non implementata in questo caso.
        """
        pass

    def close(self):
        """
        Chiude l'ambiente, distruggendo il nodo ROS e spegnendo il runtime ROS.
        """
        self.node.destroy_node()
        rclpy.shutdown()

    # Funzioni di utilità per accedere e modificare il contatore delle collisioni.
    def get_collisions(self):
        print('Reading value', self.collisions)
        return self.collisions

    def set_collisions(self, val ):
        print('Setting value to', val)
        self.collisions = val


# Definizione della funzione principale che avvia l'addestramento.
from my_environment_pkg.train_agent import main

def run():
    """
    Funzione wrapper per avviare il processo di addestramento.
    """
    main()

if __name__ == "__main__":
    # Quando lo script viene eseguito direttamente, avvia la funzione run.
    run()