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
      # Ignore signs of user input
      user_acceleration = abs(user_acceleration)
      user_goal_position = abs(user_goal_position)
      user_velocity = abs(user_velocity)
      user_deceleration = abs(user_deceleration)
         
      # If initial velocity is greater than user, start out decelerating
      if(initial_velocity > user_velocity):
          user_acceleration = -user_deceleration


      # Calculate relative values
      relative_goal_position = user_goal_position - initial_position
      relative_user_velocity = user_velocity - initial_velocity


      # Calculate time and distances to extremes
      time_to_max_velocity = relative_user_velocity / user_acceleration
      distance_to_max_velocity = (0.5 * user_acceleration) * (time_to_max_velocity**2) + (initial_velocity * time_to_max_velocity)

      time_to_no_velocity = user_velocity / user_deceleration
      distance_to_no_velocity = (0.5 * user_deceleration) * (time_to_no_velocity**2)
     
      # Validate User Input is physically possible
      valid = LinearActuatorSimulator.validate_input(user_acceleration, user_deceleration, user_velocity, user_goal_position, initial_position, initial_velocity, time_to_max_velocity, time_to_no_velocity)
      if not valid:
          print("Not a valid movement")
          exit()


      # Handle Triangle Motion Profile if necessary
      if distance_to_max_velocity + distance_to_no_velocity > relative_goal_position:
          time_to_max_velocity, time_to_no_velocity = LinearActuatorSimulator.solve_for_time(user_acceleration, user_deceleration, relative_goal_position, initial_velocity)
          const_v_time = 0
          time_under_initial = initial_velocity / user_deceleration
          total_time = time_to_no_velocity + time_to_max_velocity + time_under_initial


      # If not calculate time with no accel
      else:
          const_v_time = (relative_goal_position - (distance_to_no_velocity + distance_to_max_velocity)) / (user_velocity)
          total_time = time_to_no_velocity + time_to_max_velocity + const_v_time


      time_array = [0, time_to_max_velocity, time_to_max_velocity + const_v_time, total_time]
      accelerations = [user_acceleration, 0, -user_deceleration, 0]


      # Dicretize the continuous data
      positions, velocities, discretized_accel, discretized_times = LinearActuatorSimulator.discretize_data(time_array, sample_rate, accelerations, initial_position, initial_velocity)


      return time_array, accelerations, positions, velocities, discretized_accel, discretized_times


  @staticmethod
  def solve_for_time(user_acceleration, user_deceleration, relative_goal_position, initial_velocity):
      time_to_max_velocity, time_to_no_velocity = symbols("time_to_max_velocity time_to_no_velocity")
      equation1 = Eq(
          initial_velocity * time_to_max_velocity
          + 0.5 * user_acceleration * time_to_max_velocity**2
          + initial_velocity * time_to_no_velocity
          + 0.5 * user_deceleration * time_to_no_velocity**2
          + (initial_velocity**2) / (2 * user_deceleration),
          relative_goal_position,
      )
      equation2 = Eq(
          (user_acceleration * time_to_max_velocity)
          - (user_deceleration * time_to_no_velocity),
          0,
      )
      solutions = solve((equation1, equation2), (time_to_max_velocity, time_to_no_velocity))


      # Filter out negative solutions
      positive_solutions = [solution for solution in solutions if solution[0] > 0 and solution[1] > 0]


      # If there are positive solutions return
      if positive_solutions:
          print("Positive solution for time_to_max_velocity and time_to_no_velocity:")
          print(positive_solutions)
          return positive_solutions[0]
      else:
          print("No positive solutions")
          exit()
        

  @staticmethod
  def validate_input(user_acceleration, user_deceleration, user_velocity, user_goal_position, initial_position, initial_velocity, time_to_max_velocity, time_to_no_velocity):
      if user_acceleration == 0 or user_deceleration == 0:
          print("Acceleration and Deceleration must be greater than 0")
          return False
      if user_velocity == 0:
          print("Velocity must be greater than 0")
          return False
      if user_goal_position < initial_position:
          print("Goal position must be greater than or equal to initial position")
          return False
      if 0.5 * user_deceleration * time_to_no_velocity ** 2 > user_goal_position - initial_position:
          print("Not able to decelerate in time :( ")
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
      return positions, velocities, discretized_accel, discretized_times
  
  
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


class Operations:
   def __init__(self):
       self.last_value = 0
       self.accelerations = []


   @staticmethod
   def step(self, acceleration, t):
       deltatime = t - self.last_value
       self.last_value = t
       self.accelerations.append(acceleration)
       velocities = np.cumsum(self.accelerations) * deltatime
       positions = np.cumsum(velocities) * deltatime
       return positions[-1], velocities[-1]
   

def simulate(sim_total_time, initial_position, initial_velocity, sample_rate):
   t = 0
   final_accelerations = []
   final_velocities = [initial_velocity]
   final_positions = [initial_position]
   current_accel = 0


   commands = {
       3 : (1, 100, 5, 2),
       9 : (2, 50, 5, 2)
   }


   while t < sim_total_time:
       current_position, current_velocity = Operations.step(current_accel, t)
       final_positions.append(current_position)
       final_velocities.append(current_velocity)


       for i in commands:
           if commands[i][0] > t - 1/sample_rate and commands[i][0] < t + 1/sample_rate:
               time_array, accelerations, positions, velocities, discretized_accel, discretized_times = LinearActuatorSimulator.generate_motion_profile(
                   commands[i][1], commands[i][2], commands[i][3], commands[i][4], final_positions[-1], final_velocities[-1]
               )
               current_accel = accelerations[-1]
               t += time_array[-1]


       t += 1/sample_rate


   array = np.arange(0, sim_total_time, 1/sample_rate)
 

def main():
  # simulate(50, 0, 2, sample_rate)
  # FUNCTION CALL
  time_array, accelerations, positions, velocities, discretized_accel, discretized_times = LinearActuatorSimulator.generate_motion_profile(
      user_acceleration = 3,
      user_goal_position = 20,
      user_velocity = 15,
      user_deceleration = 10,
      initial_position = 0,
      initial_velocity = 20,
  )
sample_rate = 1000.0
main()


# TODO
  # while t < sim_total_time:
  #     if t < 3 - 1/sample_rate:
  #         final_accelerations.append(0)
  #         final_velocities.append(initial_velocity)
  #         final_positions.append(np.cumsum(final_velocities) * t + initial_position)
         
  #     elif t < 3 + 1/sample_rate and t > 3 - 1/sample_rate:
  #         print("here")
  #         time_array, accelerations, positions, velocities, discretized_accel, discretized_times = LinearActuatorSimulator.generate_motion_profile(
  #             user_acceleration = 1,
  #             user_goal_position = 10,
  #             user_velocity = 5,
  #             user_deceleration = 2,
  #             initial_position = final_positions[-1],
  #             initial_velocity = final_velocities[-1],
  #          )      
  #         final_accelerations.append(discretized_accel)
  #         final_velocities.append(velocities)
  #         final_positions.append(positions)
  #         t += time_array[-1]          
  #     else:
  #         final_accelerations.append(0)
  #         final_velocities.append(0)
  #         final_positions.append(final_positions[-1])         


