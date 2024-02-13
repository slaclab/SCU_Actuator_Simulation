import math
import matplotlib.pyplot as plt

class LinearActuatorSimulator:
   def __init__(self, max_acceleration, goal_position, max_velocity):
       self.max_acceleration = abs(max_acceleration)
       self.max_velocity = abs(max_velocity)
       self.goal_position = abs(goal_position)
       self.current_position = 0
       

   def generate_motion_profile(self):
       time_to_max_velocity = self.max_velocity / self.max_acceleration

       distance_to_max_velocity = (max_acceleration * 0.5) * (time_to_max_velocity ** 2)
       
       # Handle Triangle Motion Profile if necessary
       if (distance_to_max_velocity * 2 > self.goal_position):
           distance_to_max_velocity = self.goal_position / 2
           time_to_max_velocity = math.sqrt((2 * distance_to_max_velocity)/self.max_acceleration)
           self.max_velocity = self.max_acceleration * time_to_max_velocity
           const_v_time = 0
       
       # If not calculate time with no accel
       else:
            const_v_time = (goal_position - (2 * distance_to_max_velocity))/max_velocity

       total_time = (2 * time_to_max_velocity) + const_v_time
       time_array = [0, time_to_max_velocity, time_to_max_velocity+const_v_time, total_time]
       accelerations = [max_acceleration, 0, -max_acceleration, 0]
       
       return time_array, accelerations

# USER INPUT
max_acceleration = 2
max_velocity = 4
goal_position = 12

# FUNCTION CALLS
actuator_simulator = LinearActuatorSimulator(max_acceleration, goal_position, max_velocity)
time_array, accelerations = actuator_simulator.generate_motion_profile()

print("time_array:", time_array)
print("accelerations:", accelerations)