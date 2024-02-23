import matplotlib.pyplot as plt
import numpy as np
from sympy import *

class LinearActuatorSimulator:
   @staticmethod
   def generate_motion_profile(
       user_acceleration,
       user_goal_position,
       user_velocity,
       user_deceleration,
       initial_position,
       initial_velocity,
   ):
       valid = LinearActuatorSimulator.validate_input(user_acceleration, user_deceleration, user_velocity, user_goal_position, initial_position, initial_velocity)
       if not valid:
           print("not a valid movement")
           exit()
  
       relative_goal_position = user_goal_position - initial_position
       relative_user_velocity = user_velocity - initial_velocity


       time_to_user_velocity = relative_user_velocity / user_acceleration
       distance_to_user_velocity = (0.5 * user_acceleration) * (time_to_user_velocity**2) + (initial_velocity * time_to_user_velocity)


       time_to_no_velocity = user_velocity / user_deceleration
       distance_to_no_velocity = (0.5 * user_deceleration) * (time_to_no_velocity**2)


       # Handle Triangle Motion Profile if necessary
       if distance_to_user_velocity + distance_to_no_velocity > relative_goal_position:
           try:
               time_to_user_velocity, time_to_no_velocity = LinearActuatorSimulator.solve_for_time(user_acceleration, user_deceleration, relative_goal_position, initial_velocity)
           except Exception:
               print("not a valid movement")
               exit()
           const_v_time = 0
           time_under_initial = initial_velocity / user_deceleration
           total_time = time_to_no_velocity + time_to_user_velocity + time_under_initial


       # If not calculate time with no accel
       else:
           const_v_time = (relative_goal_position - (distance_to_no_velocity + distance_to_user_velocity)) / (user_velocity)
           total_time = time_to_no_velocity + time_to_user_velocity + const_v_time


       time_array = [0, time_to_user_velocity, time_to_user_velocity + const_v_time, total_time]
       accelerations = [user_acceleration, 0, -user_deceleration, 0]


       LinearActuatorSimulator.discretize_data(time_array, sample_rate, accelerations, initial_position, initial_velocity)


       return time_array, accelerations


   @staticmethod
   def solve_for_time(user_acceleration, user_deceleration, relative_goal_position, initial_velocity):
       time_to_user_velocity, time_to_no_velocity = symbols("time_to_user_velocity time_to_no_velocity")
       equation1 = Eq(
           initial_velocity * time_to_user_velocity
           + 0.5 * user_acceleration * time_to_user_velocity**2
           + initial_velocity * time_to_no_velocity
           + 0.5 * user_deceleration * time_to_no_velocity**2
           + (initial_velocity**2) / (2 * user_deceleration),
           relative_goal_position,
       )
       equation2 = Eq(
           (user_acceleration * time_to_user_velocity)
           - (user_deceleration * time_to_no_velocity),
           0,
       )
       solutions = solve((equation1, equation2), (time_to_user_velocity, time_to_no_velocity))


       # Filter out negative solutions
       positive_solutions = [
           solution for solution in solutions if solution[0] > 0 and solution[1] > 0
       ]


       # If there are positive solutions return
       if positive_solutions:
           print("Positive solution for time_to_user_velocity and time_to_no_velocity:")
           print(positive_solutions)
           return positive_solutions[0]
       else:
           return None


   @staticmethod
   def validate_input(user_acceleration, user_deceleration, user_velocity, user_goal_position, initial_position, initial_velocity):
       if user_acceleration == 0 or user_deceleration == 0:
           print("Acceleration and Deceleration must be greater than 0")
           return False
       if user_velocity == 0:
           print("Velocity must be greater than 0")
           return False
       if user_goal_position < initial_position:
           print("Goal position must be greater than or equal to initial position")
           return False
       return True


   @staticmethod
   def discretize_data(time_array, sample_rate, accelerations, initial_position, initial_velocity):
       a = 0
       total_time = time_array[3]
       discretized_accel = []
       time_step = 1.0 / sample_rate
       discretized_times = np.arange(0, total_time + time_step, time_step)
       for a in discretized_times:
           if a == 0:
               discretized_accel.append(accelerations[3])  # start at no accel
           elif a < time_array[1]:
               discretized_accel.append(accelerations[0])  # go to accel
           elif a < time_array[2]:
               discretized_accel.append(accelerations[1])  # go to no accel
           elif a < time_array[3]:
               discretized_accel.append(accelerations[2])  # go to decel
           else:
               discretized_accel.append(accelerations[3])  # end at rest


       velocities = np.cumsum(discretized_accel) * time_step + initial_velocity
       positions = np.cumsum(velocities) * time_step + initial_position


       print("discretized final time:", discretized_times[-1])
       print("discretized final accel: ", discretized_accel[-1])
       print("discretized final vel: ", velocities[-1])
       print("discretized final pos: ", positions[-1])
       print("relative time array:", time_array)
       print("relative time accelerations:", accelerations)


       LinearActuatorSimulator.plot_data(discretized_times, positions, velocities, discretized_accel)
  
   @staticmethod
   def plot_data(discretized_times, positions, velocities, discretized_accel):
       plt.figure(figsize=(10, 6))


       plt.subplot(3, 1, 3)
       plt.plot(discretized_times, positions, label="Position")
       plt.xlabel("Time (s)")
       plt.ylabel("Position")


       plt.subplot(3, 1, 2)
       plt.plot(discretized_times, velocities, label="Velocity")
       # plt.plot(discretized_times[:-1], np.diff(positions)/np.diff(discretized_times), '.', label='Vel_int')
       plt.ylabel("Velocity")


       plt.subplot(3, 1, 1)
       plt.plot(discretized_times, discretized_accel, label="Acceleration")
       plt.title("Linear Actuator Motion Profile")
       plt.ylabel("Acceleration")


       plt.tight_layout()
       plt.show()




def main():
   # USER INPUT
   user_acceleration = 10
   user_deceleration = 1
   user_velocity = 100
   user_goal_position = 100


   initial_position = 3
   initial_velocity = 10


   # FUNCTION CALL
   time_array, accelerations = LinearActuatorSimulator.generate_motion_profile(
       user_acceleration,
       user_goal_position,
       user_velocity,
       user_deceleration,
       initial_position,
       initial_velocity,
   )
sample_rate = 12000.0
main()


# TODO
# function that takes in sim time, init pos and vel
# stay in a while loop while incrementing through "sim time" until total sim time
# at each increment return velocity position and acceleration values
# need some type of real-time array that can be updated in time by motion profile or stop
# in output_data it should calculate initial changes
# plot after real-time array is finished generating


def simulate(sim_total_time, initial_position, initial_velocity):
   t = 0
   while t < sim_total_time:
       LinearActuatorSimulator.generate_motion_profile(max, max, initial_velocity, 0)
       initial_position = user_goal_position
       initial_velocity = 0
       # elif t == 3 run stop once
       initial_position = user_goal_position
       initial_velocity = 0
       t += 1 / sample_rate




def stop_motion_profile(initial_position, initial_velocity):
   relative_goal_position = initial_position + (initial_velocity / user_deceleration)
   LinearActuatorSimulator.generate_motion_profile(
       user_acceleration, relative_goal_position, user_velocity, user_deceleration
   )




i = 0
def output_data():
   i += 1
   return accelerations[i], velocities[i], positions[i]




