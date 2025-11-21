"""
Riassunto del Codice: Policy Network per Agente Soft Actor-Critic (SAC)

Questo modulo definisce la classe **PolicyNetwork**, che funge da attore (Actor)
all'interno dell'algoritmo SAC. Implementa una policy stocastica gaussiana, dove il network
apprende i parametri statistici (media e deviazione standard) della distribuzione
delle azioni.

Caratteristiche Principali:
1.  **Struttura:** Network feedforward a due strati nascosti (256 neuroni).
2.  **Output:** Genera sia la media (`mean`) che la deviazione standard logaritmica
    (`log_std`) della distribuzione Gaussiana.
3.  **Reparameterization Trick:** Il metodo `sample` utilizza il *reparameterization trick*
    e la funzione di attivazione `tanh` (squashing) per generare azioni all'interno di
    un intervallo limitato [-1, 1], calcolando anche la log-probabilità corretta.
"""


import torch
import torch.nn as nn
import torch.nn.functional as F

class PolicyNetwork(nn.Module):
    def __init__(self, state_dim, action_dim, log_std_min=-20, log_std_max=2):
        super(PolicyNetwork, self).__init__()
        self.fc1 = nn.Linear(state_dim, 256)
        self.fc2 = nn.Linear(256, 256)
        
        self.mean = nn.Linear(256, action_dim)
        self.log_std = nn.Linear(256, action_dim)
        
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        
    def forward(self, state):
        x = F.relu(self.fc1(state))
        x = F.relu(self.fc2(x))
        
        mean = self.mean(x)
        log_std = self.log_std(x)
        log_std = torch.clamp(log_std, self.log_std_min, self.log_std_max)
        std = torch.exp(log_std)
        
        return mean, std
    
    def sample(self, state):
        mean, std = self.forward(state)
        normal = torch.distributions.Normal(mean, std)
        x = normal.rsample()
        action = torch.tanh(x)
        
        log_prob = normal.log_prob(x)
        log_prob -= torch.log(1 - action.pow(2) + 1e-6)
        log_prob = log_prob.sum(1, keepdim=True)
        
        return action, log_prob
