import matplotlib.pyplot as plt
import numpy as np

# Angles in degrees and their names
angles_deg = [0, -1.12, 0.26, -0.11]
names = ['Alpha/Beta Plane', 'Fuselage', 'Airfoil', "Tail"]

# Slopes of the lines (5 times the angle in degrees)
slopes = [5 * np.tan(np.radians(angle)) for angle in angles_deg]

# Create an xy plot with the origin centered
fig, ax = plt.subplots(figsize=(8, 8))

# Setting the limits for the plot. Since we want the origin centered, we find the max slope to set symmetrical limits.
xlim = max(abs(slope) for slope in slopes) + 1
ax.set_xlim(-xlim, xlim)
ax.set_ylim(-xlim, xlim)

# Adjusting the plot: removing axis labels, using bold colors, and placing names (and values) in a legend

# Define bold colors for each line
bold_colors = ['blue', 'green', 'red', 'magenta']

# Create an xy plot with the origin centered
fig, ax = plt.subplots(figsize=(8, 8))

# Setting the limits for the plot to be symmetrical
ax.set_xlim(-xlim, xlim)
ax.set_ylim(-xlim/2, xlim/2)

# Plotting each line that passes through the origin and adding to the legend
for slope, name, color, angle in zip(slopes, names, bold_colors, angles_deg):
    # Define the x values
    x_values = np.linspace(-xlim, xlim, 400)
    # Calculate the y values based on the slope
    y_values = slope * x_values
    # Plot the line
    ax.plot(x_values, y_values, color=color, label=f"{name} ({angle:.2f}°)")

# Remove axis labels
ax.set_xticklabels([])
ax.set_yticklabels([])
ax.set_xlabel("")
ax.set_ylabel("")

# Drawing the x and y axis in the center
ax.axhline(0, color='black', linewidth=0.5)
ax.axvline(0, color='black', linewidth=0.5)

# Set equal aspect ratio
ax.set_aspect('equal')

# Place the legend outside the plot
ax.legend(loc='upper right', bbox_to_anchor=(1.1, 1.1))

# Display the plot
plt.show()
