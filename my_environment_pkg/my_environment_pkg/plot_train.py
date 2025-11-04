import pandas as pd
import matplotlib.pyplot as plt

Monitor = pd.read_csv('/home/gabri/dynamics/robotic_arm_environment/checkpoints/train_8/Monitoraggio.csv', header=2)

print(Monitor.head())


x_col = Monitor.columns[0]
y1_col = Monitor.columns[1]
y2_col = Monitor.columns[2]

# 3. Assicurati che i dati siano numerici (opzionale ma consigliato)
Monitor[x_col] = pd.to_numeric(Monitor[x_col], errors='coerce')
Monitor[y1_col] = pd.to_numeric(Monitor[y1_col], errors='coerce')
Monitor[y2_col] = pd.to_numeric(Monitor[y2_col], errors='coerce')

#  plot reward
plt.figure(figsize=(8, 4))  # Imposta le dimensioni del grafico

# Traccia entrambe le linee
plt.plot(Monitor[x_col].to_numpy(), Monitor[y1_col].to_numpy(), label="Reward")

# 5. Aggiungi etichette e legenda
plt.xlabel("Episodi (da 100 step)")  # Etichetta asse X
plt.ylabel("Reward")  # Etichetta asse Y
plt.title("Andamento Reward")  # Titolo
plt.grid(True)  # Aggiunge una griglia
plt.show()


#plot successi

plt.figure(figsize=(8, 4))  # Imposta le dimensioni del grafico

# Traccia entrambe le linee
plt.plot(Monitor[x_col].to_numpy(), Monitor[y2_col].to_numpy(), label="Successi")

# 5. Aggiungi etichette e legenda
plt.xlabel("Episodi (da 100 step)")  # Etichetta asse X
plt.ylabel("Successi")  # Etichetta asse Y
plt.title("Andamento Successi")  # Titolo
plt.grid(True)  # Aggiunge una griglia
plt.show()