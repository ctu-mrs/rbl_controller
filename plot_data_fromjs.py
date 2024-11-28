import numpy as np
import matplotlib.pyplot as plt
import json

# Load data
data = np.load('uav_data.npz')
id1_x = data['id1_x']
id1_y = data['id1_y']
id3_x = data['id3_x']
id3_y = data['id3_y']
id4_x = data['id4_x']
id4_y = data['id4_y']
final_x = data['final_x']
final_y = data['final_y']

# Load colors
with open('colors.json', 'r') as json_file:
    color_data = json.load(json_file)

colors1 = color_data['colors1']
colors3 = color_data['colors3']
colors4 = color_data['colors4']
final_colors = color_data['final_colors']

# Plot data
plt.figure(figsize=(10, 8))

# Plot ID 1 data
plt.plot(id1_x, id1_y, color='red', linewidth=1, label='UAV 3')
plt.scatter(id1_x, id1_y, c=colors1, marker='o', s=100)

# Plot ID 3 data
plt.plot(id3_x, id3_y, color='blue', linewidth=1, label='UAV 1')
plt.scatter(id3_x, id3_y, c=colors3, marker='o', s=100)

# Plot ID 4 data
plt.plot(id4_x, id4_y, color='orange', linewidth=1, label='UAV 4')
plt.scatter(id4_x, id4_y, c=colors4, marker='o', s=100)

# Plot final data
plt.plot(final_x, final_y, color='green', linewidth=2, label='UAV 2')
plt.scatter(final_x, final_y, c=final_colors, marker='o', s=100)

# Add labels and legend
plt.xlabel('Position X')
plt.ylabel('Position Y')
plt.title('UAV Path Visualization')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Show plot
plt.show()
