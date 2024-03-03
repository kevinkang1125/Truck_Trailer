import numpy as np
import matplotlib.pyplot as plt

num_points_line = 50
num_points_circle = 100
line_length = 3
radius = 2.0
# Generate data for straight line
x_line = np.linspace(0, line_length, num_points_line)
y_line = np.zeros(num_points_line)

# Generate data for circle
theta = np.linspace(0, 2*np.pi, num_points_circle)
x_circle = radius * np.cos(theta-0.5*np.pi) + line_length
y_circle = radius * np.sin(theta-0.5*np.pi)+2

# generate data for out road
x_out = np.linspace(2, line_length+2, num_points_line)
y_out = 5*np.ones(num_points_line)
# Plot the figure
plt.figure(figsize=(8, 6))
plt.plot(x_line, y_line, 'b-', label='Straight Line')  # Plot straight line
plt.plot(x_circle, y_circle, 'r-', label='Circle')      # Plot circle
plt.plot(y_out,x_out,'b-',label = 'out line')
#plt.plot([line_length, line_length + radius], [0, 0], 'b-')  # Joining line to circle
#plt.plot(line_length, 0, 'bo')                           # Endpoint of straight line
#plt.plot(line_length + radius, 0, 'ro')                  # Starting point of circle

# Set aspect ratio to be equal
plt.gca().set_aspect('equal', adjustable='box')

# Set labels and title
plt.title('Plot with Straight Line, Circle, and Straight Line')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.legend()
print(x_circle)
# Show plot
plt.show()