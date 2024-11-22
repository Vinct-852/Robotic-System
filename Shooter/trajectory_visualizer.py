import numpy as np
import matplotlib.pyplot as plt
from ctypes import cdll, c_double, POINTER
import math

court_length = 15.24          # Length from baseline to free-throw line (15 feet)
court_width = 14.02           # Width of the court (15 feet)

# Set pyplot configuration
fig = plt.figure(figsize=(10, 5))
ax = fig.add_subplot(projection='3d')
ax.set_aspect('auto')           # Set aspect ratio to auto for better visualization


######################## basketball hoop ########################
inner_radius = 0.23           # Inner radius of the hoop in meters (230mm)
n = 100                        # Number of points for the circle
hoop_x = court_length          # X position of the hoop
hoop_y = court_width / 2       # Y position of the hoop
hoop_z = 2.43                  # Height of the hoop in meters

points = inner_radius * np.exp(1j * np.linspace(0, 2 * np.pi, n))
u, v = np.real(points), np.imag(points)
w = np.full_like(u, hoop_z)   # Set all z-coordinates to hoop_z

# Draw the hoop as a circle in Z-plane at hoop height
ax.plot(u + hoop_x, v + hoop_y, w, color='red')

######################## shooter ########################
shooter_x = 0
shooter_y = court_width/2
shooter_z = 1.5
ax.scatter(shooter_x, shooter_y, shooter_z, color='blue', s=10, label='Shooter Position')

######################## trajectory ########################
horizontal_distance = np.sqrt((hoop_x - shooter_x)**2 + (hoop_y - shooter_y)**2)
ax.plot([shooter_x, hoop_x], [shooter_y, hoop_y], color='green', linestyle='--', 
        label=f'Horizontal Distance: {horizontal_distance:.2f} m')

# Load the shared library
lib = cdll.LoadLibrary('./Shooter/libtrajectory.so')

class Trajectory(object):
    def __init__(self):
        # Create an instance of trajectory
        self.obj = lib.create_trajectory()

    def __del__(self):
        # Clean up by destroying the trajectory instance
        lib.destroy_trajectory(self.obj)

    def calculate_velocity(self, D, H_shooter, angle):
        # Call calculate_velocity from the C++ library
        lib.calculate_velocity.argtypes = [POINTER(c_double), c_double, c_double, c_double]
        lib.calculate_velocity.restype = c_double
        
        # Call the function and return the result
        velocity = lib.calculate_velocity(self.obj, c_double(D), c_double(H_shooter), c_double(angle))
        return velocity

trajectory = Trajectory()
angle = math.radians(45)
velocity = trajectory.calculate_velocity(horizontal_distance, shooter_z, angle)
print(f"Calculated Velocity: {velocity:.2f} m/s")

# Set limits for better visualization
ax.set_xlim(0, court_length)
ax.set_ylim(0, court_width)
ax.set_zlim(0, 3)   # Limit Z-axis to show height up to a bit above the rim

# Labeling axes
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.set_zlabel('Z (m)')
ax.set_title('3D Basketball Hoop Visualization')
ax.legend()

# Show the plot
plt.show()