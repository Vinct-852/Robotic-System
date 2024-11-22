import numpy as np
import matplotlib.pyplot as plt
import ctypes
from ctypes import *
import math

trajectorylib = cdll.LoadLibrary('Robotic-System/Shooter/trajectory.so')
g = 9.81  # Acceleration due to gravity (m/s^2)

# Define the trajectory class
class Trajectory:
    def __init__(self):
        self.obj = trajectorylib.create_trajectory()
        
    def __del__(self):
        self.destroy_trajectory(self.obj)

    def destroy_trajectory(self, t):
        trajectorylib.destroy_trajectory(self.obj)
        
    def calculate_velocity(self, D, H_shooter, angle):
        trajectorylib.calculate_velocity.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        trajectorylib.calculate_velocity.restype = ctypes.c_double
        
        return trajectorylib.calculate_velocity(self.obj, D, H_shooter, angle)
    
    def calculate_angle(self, D, H_shooter, velocity):
        trajectorylib.calculate_angle.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
        trajectorylib.calculate_angle.restype = ctypes.c_double
        
        return trajectorylib.calculate_angle(self.obj, D, H_shooter, velocity)

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
shooter_z = 1.0
ax.scatter(shooter_x, shooter_y, shooter_z, color='blue', s=10, label='Shooter Position')

######################## trajectory ########################
horizontal_distance = np.sqrt((hoop_x - shooter_x)**2 + (hoop_y - shooter_y)**2)
ax.plot([shooter_x, hoop_x], [shooter_y, hoop_y], color='green', linestyle='--', 
        label=f'Horizontal Distance: {horizontal_distance:.2f} m')

traj = Trajectory()
angle_degrees = 45
velocity = traj.calculate_velocity(horizontal_distance, shooter_z, angle_degrees)
print(f"horizontal_distance: {horizontal_distance}")
print(f"Calculated Velocity: {velocity}")

# angle = traj.calculate_angle(horizontal_distance, shooter_z, velocity)
# print(f"Calculated Angle: {angle}")

######################## setting ########################
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


######################## 2d diagram ########################
angle_radians = np.radians(angle_degrees)
t_flight = (velocity * np.sin(angle_radians) + 
             np.sqrt((velocity * np.sin(angle_radians))**2 + 2 * g * shooter_z)) / g

# Time intervals
t = np.linspace(0, t_flight, num=500)

# Trajectory equations
x = velocity * np.cos(angle_radians) * t
y = shooter_z + velocity * np.sin(angle_radians) * t - (0.5 * g * t**2)

# Plotting the trajectory
plt.figure(figsize=(10, 5))
plt.plot(x, y)
plt.title('Projectile Trajectory')
plt.xlabel('Horizontal Distance (m)')
plt.ylabel('Height (m)')
plt.axhline(0, color='black', lw=0.5)  # Ground line
plt.axvline(horizontal_distance, color='red', linestyle='--', label='Target Distance')
plt.legend()
plt.grid()
plt.xlim(0, horizontal_distance + 10)
plt.ylim(0, max(y) + 5)

# Show the plot
plt.show()