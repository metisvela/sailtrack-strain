import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Simulated data (replace with your own CSV import if needed)
data = {
    'measure': [-280000, -331000, -386000, -410000],
    'load': [0, 36, 88, 117]
}

# Create a DataFrame from the data
df = pd.DataFrame(data)

# Sort data for better graphical representation
df = df.sort_values(by='measure')

# Extract data for plotting
measure = df['measure']
load = df['load']

# Create an array of x values for interpolation, spanning the range of your measure values
x = np.linspace(measure.min()-100000, measure.max()+2000, 800)

# Linear interpolation
linear_coeffs = np.polyfit(measure, load, 1)
linear_interp = np.polyval(linear_coeffs, x)
linear_eq = f'Linear: {linear_coeffs[0]:.10f}x + {linear_coeffs[1]:.10f}'

# Quadratic interpolation
quadratic_coeffs = np.polyfit(measure, load, 2)
quadratic_interp = np.polyval(quadratic_coeffs, x)
quadratic_eq = f'Quadratic: {quadratic_coeffs[0]:.10f}x² + {quadratic_coeffs[1]:.10f}x + {quadratic_coeffs[2]:.10f}'

# Cubic interpolation
cubic_coeffs = np.polyfit(measure, load, 3)
cubic_interp = np.polyval(cubic_coeffs, x)
cubic_eq = f'Cubic: {cubic_coeffs[0]:.10f}x³ + {cubic_coeffs[1]:.10f}x² + {cubic_coeffs[2]:.10f}x + {cubic_coeffs[3]:.10f}'

# Print the equations of the functions
print("Equations of the interpolated functions:")
print(linear_eq)
print(quadratic_eq)
print(cubic_eq)

# Plotting the data
plt.figure(figsize=(12, 8))
plt.scatter(measure, load, label='Original Data', color='red', marker='o')

# Plot the interpolations
plt.plot(x, linear_interp, label=linear_eq, linestyle='--', color='blue')
plt.plot(x, quadratic_interp, label=quadratic_eq, linestyle='-.', color='green')
plt.plot(x, cubic_interp, label=cubic_eq, linestyle=':', color='purple')

# Labels and title
plt.xlabel('Measure')
plt.ylabel('Load')
plt.title('Linear, Quadratic, and Cubic Interpolations')

# Display legend and grid
plt.legend()
plt.grid(True, linestyle='--', alpha=0.5)
plt.show()
