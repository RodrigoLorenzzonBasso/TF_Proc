import pandas as pd
import csv
import matplotlib.pyplot as plt

headers = ['i','temp','umid','ppm']

df = pd.read_csv('output.csv',names=headers)

x = df['i']
y_temp = df['temp']
y_umid = df['umid']
y_ppm = df['ppm']

plt.plot(x,y_temp)
plt.title('Grafico de Temperatura')
plt.ylabel('Temperatura (C)')
plt.figure()

plt.plot(x,y_umid)
plt.title('Grafico de Umidade')
plt.ylabel('Umidade (%)')
plt.figure()

plt.plot(x,y_ppm)
plt.title('Monóxido de Carbono no ar')
plt.ylabel('Monóxido (ppm)')
plt.figure()