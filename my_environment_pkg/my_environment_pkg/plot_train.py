import pandas as pd
import matplotlib.pyplot as plt

# Leggi il CSV
Monitor = pd.read_csv('/home/gabri/dynamics/robotic_arm_environment/checkpoints/train_8/Monitoraggio.csv', header=1)

print(Monitor.head())

x_col = Monitor.columns[0]
y1_col = Monitor.columns[1]
y2_col = Monitor.columns[2]

# Assicurati che i dati siano numerici
Monitor[x_col] = pd.to_numeric(Monitor[x_col], errors='coerce')
Monitor[y1_col] = pd.to_numeric(Monitor[y1_col], errors='coerce')
Monitor[y2_col] = pd.to_numeric(Monitor[y2_col], errors='coerce')

# Crea due subplot affiancati
fig, axes = plt.subplots(1, 2, figsize=(17, 6))

# --- Primo grafico: Reward ---
axes[0].plot(Monitor[x_col].to_numpy(), Monitor[y1_col].to_numpy(), label="Reward", color='tab:blue')
axes[0].set_xlabel("Episodi (da 100 step)")
axes[0].set_ylabel("Reward (finestre temporali da 25 episodi)")
axes[0].set_title("Andamento Reward")
axes[0].grid(True)
axes[0].legend()

# --- Secondo grafico: Successi ---
axes[1].plot(Monitor[x_col].to_numpy(), Monitor[y2_col].to_numpy(), label="Successi", color='tab:green')
axes[1].set_xlabel("Episodi (da 100 step)")
axes[1].set_ylabel("Successi (finestre temporali da 25 episodi)")
axes[1].set_title("Andamento Successi")
axes[1].grid(True)
axes[1].legend()

# Adatta layout
plt.tight_layout()
plt.show()
