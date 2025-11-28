"""
Riassunto del Codice: Analisi del Monitoraggio Addestramento Braccio Robotico

Questo script Python è progettato per analizzare e visualizzare i dati di monitoraggio
provenienti dall'addestramento di un braccio robotico (probabilmente in un ambiente di
Reinforcement Learning).

1.  Carica i Dati: Legge un file CSV chiamato 'Monitoraggio.csv' contenente le metriche di addestramento.
2.  Preparazione Dati: Identifica le colonne per gli assi X (Episodi), Y1 (Reward) e Y2 (Successi) e ne assicura la conversione in formato numerico.
3.  Visualizzazione: Genera due grafici affiancati (Reward e Successi) per visualizzare l'andamento delle metriche rispetto al numero di episodi, consentendo una rapida valutazione delle performance del modello durante l'addestramento.
"""

import pandas as pd
import matplotlib.pyplot as plt

# Caricamento e Preparazione dei Dati

# Definisce il percorso del file CSV di monitoraggio.
# Il parametro 'header=1' indica che l'intestazione (i nomi delle colonne)
# si trova sulla seconda riga del file (indice 1), saltando la prima riga.

# Nota: cambiare i seguenti path con il file di log desiderato
Monitor = pd.read_csv('/home/gabri/tm5_900/checkpoints/train_8/Monitoraggio.csv', header=1)
#Monitor = pd.read_csv('/home/gabri/tm5_900/checkpoints/train_8_and_obs_2/Monitoraggio.csv', header=0)

# Stampa le prime righe del DataFrame per verifica iniziale dei dati caricati.
print(Monitor.head())

# Identifica i nomi delle colonne da utilizzare.
# Si assume che la prima colonna (indice 0) sia l'asse X (Episodi),
# la seconda (indice 1) sia Y1 (Reward) e la terza (indice 2) sia Y2 (Successi).
x_col = Monitor.columns[0]
y1_col = Monitor.columns[1]
y2_col = Monitor.columns[2]

# Assicura che i dati delle colonne selezionate siano trattati come valori numerici.
# 'errors='coerce'' sostituisce eventuali valori non convertibili con 'NaN'.
Monitor[x_col] = pd.to_numeric(Monitor[x_col], errors='coerce')
Monitor[y1_col] = pd.to_numeric(Monitor[y1_col], errors='coerce')
Monitor[y2_col] = pd.to_numeric(Monitor[y2_col], errors='coerce')

# Creazione dei Grafici

# Crea una figura (fig) e un set di due subplot (axes) disposti su 1 riga e 2 colonne.
# La dimensione della figura è impostata su 17x6 per un'ampia visualizzazione.
fig, axes = plt.subplots(1, 2, figsize=(17, 6))

# --- Primo grafico: Andamento del Reward (Asse a Sinistra) ---
# Traccia i dati: X = Episodi, Y = Reward. Si usa .to_numpy() per l'efficienza di matplotlib.
axes[0].plot(Monitor[x_col].to_numpy(), Monitor[y1_col].to_numpy(), label="Reward", color='tab:blue')
# Imposta l'etichetta dell'asse X.
axes[0].set_xlabel("Episodi (da 100 step)")
# Imposta l'etichetta dell'asse Y.
axes[0].set_ylabel("Reward (finestre temporali da 25 episodi)")
# Imposta il titolo del grafico.
axes[0].set_title("Andamento Reward")
# Abilita la griglia.
axes[0].grid(True)
# Mostra la legenda del grafico.
axes[0].legend()

# --- Secondo grafico: Andamento dei Successi (Asse a Destra) ---
# Traccia i dati: X = Episodi, Y = Successi.
axes[1].plot(Monitor[x_col].to_numpy(), Monitor[y2_col].to_numpy(), label="Successi", color='tab:green')
# Imposta l'etichetta dell'asse X.
axes[1].set_xlabel("Episodi (da 100 step)")
# Imposta l'etichetta dell'asse Y.
axes[1].set_ylabel("Successi (finestre temporali da 25 episodi)")
# Imposta il titolo del grafico.
axes[1].set_title("Andamento Successi")
# Abilita la griglia.
axes[1].grid(True)
# Mostra la legenda.
axes[1].legend()

# Adatta automaticamente i parametri del subplot per prevenire sovrapposizioni.
plt.tight_layout()

# Mostra la figura con entrambi i grafici.
plt.show()