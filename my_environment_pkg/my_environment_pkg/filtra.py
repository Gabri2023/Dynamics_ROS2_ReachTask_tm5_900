import pandas as pd

Data = pd.read_csv('/home/gabri/dynamics/robotic_arm_environment/checkpoints/train_8/Monitoraggio.csv', header=0)
Dataprev = Data
print(Data.head())
print(Data.shift(1).head())


nome_colonna = Data.columns[1] 
nome_colonna_2 = Data.columns[2] 

Data[nome_colonna] = - Data[nome_colonna] 
print(Data.head())

Data[nome_colonna] = Data[nome_colonna] - Data[nome_colonna].shift(1)
Data[nome_colonna_2] = Data[nome_colonna_2] - Data[nome_colonna_2].shift(1)

Data[nome_colonna_2] = Data[nome_colonna_2].astype('Int64')

Data[nome_colonna] = - Data[nome_colonna] 

print(Data.head())

Data.to_csv('/home/gabri/dynamics/robotic_arm_environment/checkpoints/train_8/Monitoraggio2.csv', index=False)
