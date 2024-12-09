import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# Load the CSV data into a pandas DataFrame
file_path = '/home/eliott-frohly/Robot5A_BT/src/robot_visual/logs/transform_log.csv'
data = pd.read_csv(file_path)

# Extract x, y, z, and timestamp columns
x = data['x']
y = data['y']
z = data['z']
timestamp = data['timestamp']

# Create a 3D scatter plot
fig = plt.figure(figsize=(10, 7))
ax = fig.add_subplot(111, projection='3d')

# Plot the points with the color representing the timestamp
scatter = ax.scatter(x, y, z, c=timestamp, cmap='plasma', marker='o')

# Add colorbar for the timestamp values
cbar = plt.colorbar(scatter, ax=ax, pad=0.1)
cbar.set_label('Timestamp')

# Set labels
ax.set_title('3D Trajectory Visualization')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Show the plot
plt.show()
