"""
Riassunto del Codice: Replay Buffer con Hindsight Experience Replay (HER)

Questo modulo definisce la classe **ReplayBuffer**, un componente fondamentale per l'addestramento off-policy in Reinforcement Learning. È specificamente progettato per implementare la strategia **Hindsight Experience Replay (HER)**.

Funzionalità Principali:
1.  **Memorizzazione:** Memorizza l'esperienza dell'agente (transizioni e traiettorie complete).
2.  **Campionamento HER:** Durante il campionamento di un batch, la classe implementa la logica HER, sostituendo l'obiettivo desiderato (`g`) di una transizione con un obiettivo raggiunto (`ag`) selezionato casualmente dal futuro dello stesso episodio.
3.  **Supporto HER:** La struttura del buffer è adattata per gestire lo stato, l'azione, la ricompensa e i segnali 'done', inclusa la gestione esplicita degli obiettivi raggiunti e desiderati.
"""


import torch
import torch.nn as nn
import torch.nn.functional as F

class QNetwork(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(QNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim + action_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        self.fc3 = nn.Linear(256, 1)
        
    def forward(self, state, action):
        x = torch.cat([state, action], dim=1)
        x = F.relu(self.fc1(x))
        x = F.relu(self.fc2(x))
        return self.fc3(x)
