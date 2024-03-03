import matplotlib.pyplot as plt
import numpy as np

# Define parameters
num_points = 100
radius = 1.0
turn_angle = np.pi
line_length = 2
num_points_line = 100
# Generate data for U-turn
theta1 = np.linspace(0, np.pi, num_points)
#theta2 = np.linspace(np.pi, 2*np.pi, num_points)
x1 = radius * np.cos(theta1)
y1 = radius * np.sin(theta1)
y_line = -np.linspace(0, line_length, num_points_line)
x_line = -1*np.ones(num_points_line)
x_out_line= np.ones(num_points_line)
#x2 = radius * np.cos(theta2)
#y2 = radius * np.sin(theta2)

# Plot U-turn
plt.figure(figsize=(8, 6))
plt.plot(x1, y1, 'b-', label='U-turn')
plt.plot(x_line, y_line, 'b-', label='Straight Line')  # Plot straight line
plt.plot(x_out_line, y_line, 'b-', label='Straight Line')  # Plot straight line
# plt.plot(x2, y2, 'b-')
#plt.plot([x1[-1], x2[0]], [y1[-1], y2[0]], 'b-')  # Joining the two arcs
# plt.plot(x2[0], y2[0], 'bo')    # Startpoint

# Set aspect ratio to be equal
plt.gca().set_aspect('equal', adjustable='box')

# Set labels and title
plt.title('U-turn Plot')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.legend()

# Show plot
plt.show()