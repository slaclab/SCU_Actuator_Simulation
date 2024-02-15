import math
import matplotlib.pyplot as plt
import numpy as np

class LinearActuatorSimulator:
   def __init__(self, max_acceleration, goal_position, max_velocity):
        self.max_acceleration = abs(max_acceleration)
        self.max_deceleration = abs(max_deceleration)
        self.max_velocity = abs(max_velocity)
        self.goal_position = abs(goal_position)
        self.current_position = 0
        
        LinearActuatorSimulator.generate_motion_profile(self.max_acceleration, self.goal_position, self.max_velocity)
       
   @staticmethod    
   def generate_motion_profile(max_acceleration, goal_position, max_velocity, max_deceleration):
       time_to_max_velocity = max_velocity / max_acceleration
       time_to_no_velocity = max_velocity / max_deceleration

       distance_to_max_velocity = (max_acceleration * 0.5) * (time_to_max_velocity ** 2)
       distance_to_no_velocity = (max_deceleration * 0.5) * (time_to_max_velocity ** 2)
       
       # Handle Triangle Motion Profile if necessary
       if (distance_to_max_velocity + distance_to_no_velocity > goal_position):
           distance_to_max_velocity = (max_deceleration/(max_deceleration+max_acceleration)) * goal_position
           time_to_max_velocity = math.sqrt((2 * distance_to_max_velocity)/max_acceleration)
           max_velocity = max_acceleration * time_to_max_velocity
           time_to_no_velocity = max_velocity / max_deceleration
           distance_to_no_velocity = (max_deceleration * 0.5) * (time_to_no_velocity ** 2)
           const_v_time = 0
       
       # If not calculate time with no accel
       else:
            const_v_time = (goal_position - (distance_to_no_velocity + distance_to_max_velocity))/max_velocity

       total_time = time_to_no_velocity + time_to_max_velocity + const_v_time
       time_array = [0, time_to_max_velocity, time_to_max_velocity+const_v_time, total_time]
       accelerations = [max_acceleration, 0, -max_deceleration, 0]
       
       LinearActuatorSimulator.discretize_data(time_array, sample_rate, accelerations)
       
       return time_array, accelerations
   
   def discretize_data(time_array, sample_rate, accelerations):
       a = 0
       total_time = time_array[3]
       discretized_accel = []
       time_step = (1.0/sample_rate)
       discretized_times = np.arange(0,total_time+time_step,time_step)      
       for a in (discretized_times):
           if a == 0:
               discretized_accel.append(accelerations[3]) # start at rest
           elif a < time_array[1]:
               discretized_accel.append(accelerations[0]) # go to accel
           elif a < time_array[2]:
               discretized_accel.append(accelerations[1]) # go to no accel
           elif a < time_array[3]:
               discretized_accel.append(accelerations[2]) # go to decel
           else:
               discretized_accel.append(accelerations[3]) # end at rest

       velocities = np.cumsum(discretized_accel) * time_step
       positions = np.cumsum(velocities) * time_step
       
       print("discretized final time:", discretized_times[-1])
       print("discretized final accel: ", discretized_accel[-1])
       print("discretized final vel: ", velocities[-1])
       print("discretized final pos: ", positions[-1])
       print("relative time array:", time_array)
       print("relative time accelerations:", accelerations)  
       
       LinearActuatorSimulator.plot_data(discretized_times, positions, velocities, discretized_accel)
       
   def plot_data(discretized_times, positions, velocities, discretized_accel):
        # PLOTTING
        plt.figure(figsize=(10, 6))

        plt.subplot(3, 1, 3)
        plt.plot(discretized_times, positions, label='Position')
        plt.xlabel('Time (s)')
        plt.ylabel('Position')

        plt.subplot(3, 1, 2)
        plt.plot(discretized_times, velocities, label='Velocity')
        plt.ylabel('Velocity')

        plt.subplot(3, 1, 1)
        plt.plot(discretized_times, discretized_accel, label='Acceleration')
        plt.title('Linear Actuator Motion Profile')
        plt.ylabel('Acceleration')

        plt.tight_layout()
        plt.show()    

# USER INPUT
max_acceleration = 3.0
max_deceleration = 9.0
max_velocity = 60.0
goal_position = 4.0

sample_rate = 12000.0
init_pos = 0
init_vel = 0

# FUNCTION CALLS
time_array, accelerations = LinearActuatorSimulator.generate_motion_profile(max_acceleration, goal_position, max_velocity, max_deceleration)

# Motor1 = LinearActuatorSimulator(max_acceleration, goal_position, max_velocity)

# make function that returns values from time array once per simulated time increment
# make function that holds motor at value when not in motion
# mke function that simulates time this function sim(time, init pos, init vel)
# set initial values to plot
# call move function and calculate accel array
# call stop function and recalculate accel array
# do nothing and stay where it is


# TO STORE ACCELS
# make linspace array from 0 to total sim time
# accels should be 0 until moveabs or stop, velocities should be init_vel until moveabs or stop, init_pos should be init_pos + init_vel * time
# accels and vels need to end at exactly 0


# IN PROGRESS
def simulate(sim_total_time, init_pos, init_vel):
    t = 0
    while t < sim_total_time:
        current_accel = output_data()
        #if 0.9 < t < 1.1: run absmove once
        #elif 2.9 < t < 3.1 run stop once
        t+=0.0001

i = -1
def output_data():
    i=+1
    return accelerations[i]
