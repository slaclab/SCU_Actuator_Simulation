import numpy as np
import matplotlib.pyplot as plt
import math

class LinearActuatorSimulator:
   def __init__(self, max_acceleration, goal_position, max_velocity):
       self.max_acceleration = max_acceleration
       self.max_velocity = max_velocity
       self.goal_position = goal_position
       self.current_position = 0


   def generate_motion_profile(self, time_step):
       time_to_max_velocity = max_velocity / max_acceleration

       distance_to_max_velocity = (max_acceleration / 2) * (time_to_max_velocity ** 2)
       
       # Handle Triangle Motion Profile if necessary
       if (distance_to_max_velocity * 2 > goal_position):
           distance_to_max_velocity = goal_position / 2
           time_to_max_velocity = math.sqrt((2 * distance_to_max_velocity)/max_acceleration)
           self.max_velocity = max_acceleration * time_to_max_velocity
           const_v_time = 0
       
       # If not calculate time with no accel
       else:
            const_v_time = (goal_position - (2 * distance_to_max_velocity))/max_velocity

       total_time = (2 * time_to_max_velocity) + const_v_time
       
       # Generate time array
       time_array = np.arange(0, total_time, time_step)

       accelerations = []
       for t in time_array:
           if t < time_to_max_velocity: # Before fully accelerated
               acceleration = max_acceleration
           elif t < (time_to_max_velocity + const_v_time): # Before time to decelerate
               acceleration = 0
           else:
               acceleration = -max_acceleration # Time to Decelerate
           accelerations.append(acceleration)

       # Integrate accelerations to get velocities
       velocities = np.cumsum(accelerations) * time_step

       # Integrate velocities to get positions
       positions = np.cumsum(velocities) * time_step

       return time_array, positions, velocities, accelerations

# USER INPUT
max_acceleration = 3.0
max_velocity = 2.3 
goal_position = 12.0
time_step = 0.01

# FUNCTION CALLS
actuator_simulator = LinearActuatorSimulator(max_acceleration, goal_position, max_velocity)
time_array, positions, velocities, accelerations = actuator_simulator.generate_motion_profile(time_step)

# PLOTTING
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
plt.plot(time_array, positions, label='Position')
plt.title('Linear Actuator Motion Profile')
plt.ylabel('Position')

plt.subplot(3, 1, 2)
plt.plot(time_array, velocities, label='Velocity')
plt.ylabel('Velocity')

plt.subplot(3, 1, 3)
plt.plot(time_array, accelerations, label='Acceleration')
plt.xlabel('Time (s)')
plt.ylabel('Acceleration')

plt.tight_layout()
plt.show()
