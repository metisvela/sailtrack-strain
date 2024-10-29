import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Dati presi dal tuo file CSV (simulato per esempio)
data = {
    'load': [0, 60, 75, 90, 120, 150, 170],
    'measure': [38680, -38000, -56000, -64000, -90000, -115000, -130000]
}

# Creiamo un DataFrame da questi dati
df = pd.DataFrame(data)

# Ordiniamo i dati per 'load' per avere una rappresentazione corretta nel grafico
df = df.sort_values(by='load')

# Estraiamo i dati per il grafico
load = df['load']
measure = df['measure']

# Creiamo un array di valori x per la interpolazione
x = np.linspace(0, 400, 800)

# Interpolazione lineare
linear_coeffs = np.polyfit(load, measure, 1)
linear_interp = np.polyval(linear_coeffs, x)
linear_eq = f'Linear: {linear_coeffs[0]:.10f}x + {linear_coeffs[1]:.10f}'

# Interpolazione quadratica
quadratic_coeffs = np.polyfit(load, measure, 2)
quadratic_interp = np.polyval(quadratic_coeffs, x)
quadratic_eq = f'Quadratic: {quadratic_coeffs[0]:.10f}x² + {quadratic_coeffs[1]:.10f}x + {quadratic_coeffs[2]:.10f}'

# Interpolazione cubica
cubic_coeffs = np.polyfit(load, measure, 3)
cubic_interp = np.polyval(cubic_coeffs, x)
cubic_eq = f'Cubic: {cubic_coeffs[0]:.10f}x³ + {cubic_coeffs[1]:.10f}x² + {cubic_coeffs[2]:.10f}x + {cubic_coeffs[3]:.10f}'

# Plot dei dati originali
plt.figure(figsize=(12, 8))
plt.scatter(load, measure, label='Dati originali', color='red')

# Plot delle interpolazioni
plt.plot(x, linear_interp, label=linear_eq, linestyle='--', color='blue')
plt.plot(x, quadratic_interp, label=quadratic_eq, linestyle='-.', color='green')
plt.plot(x, cubic_interp, label=cubic_eq, linestyle=':', color='purple')

# Labels
plt.xlabel('load')
plt.ylabel('measure')
plt.title('Interpolazioni lineare, quadratica e cubica')

# Legend
plt.legend()
plt.grid(True)
plt.show()
