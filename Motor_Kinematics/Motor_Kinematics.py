from matplotlib import pyplot as plt
import numpy as np
from sympy import symbols, Eq, solve
import time


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
      LinearActuatorSimulator.discretize_data(time_array, sample_rate, accelerations, initial_position, initial_velocity)


      return time_array, accelerations


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
          # print("Positive solution for time_to_max_velocity and time_to_no_velocity:")
          # print(positive_solutions)
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
      # if 0.5 * user_deceleration * time_to_no_velocity ** 2 > user_goal_position - initial_position:
      #     print("Not able to decelerate in time :( ")
      #     return False
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


      # print("discretized final time:", discretized_times[-1])
      # print("discretized final accel: ", discretized_accel[-1])
      # print("discretized final vel: ", velocities[-1])
      # print("discretized final pos: ", positions[-1])
      # print("relative time array:", time_array)
      # print("relative time accelerations:", accelerations)
     
      # LinearActuatorSimulator.plot_data(discretized_times, positions, velocities, discretized_accel)
  
  
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

def display_values(final_accelerations, final_velocities, final_positions, t, j):
      if j % sample_rate == 0:
          print()
          print(t)
          print()
          print("current acceleration: ", final_accelerations[-1])
          print("current velocity: ", final_velocities[-1])
          print("current position: ", final_positions[-1])
       

class Operations:
   def __init__(self):
       self.last_value = 0
       self.acceleration = 0
       self.accelerations = []
       self.initial_velocity = 0

   def step(self, acceleration, t):
       deltatime = t - self.last_value
       self.accelerations.append(acceleration)
       velocities = np.cumsum(self.accelerations) * deltatime + self.initial_velocity
       positions = np.cumsum(velocities) * deltatime
       self.last_value = t
       return positions[-1], velocities[-1]
   
   

def simulate(sim_total_time, initial_position, initial_velocity, sample_rate):
   t = 0
   j = 0
   final_accelerations = []
   final_velocities = []
   final_positions = []
   simulated_times = []
   initial = True

    # Time : (user_accel, user_goal_pos, user_velocity, user_decel)
   move_commands = {
       3 : (6, 50, 2, 8),
       25 : (2, 125, 50, 4)
   }
   
   Motor1 = Operations()
   
   # t_arr = np.arange(0, sim_total_time, 1/sample_rate)
   # acc_arr = np.zeros_like(t_arr)
   # command_time = np.array(move_commands.keys())
   # j =0 
   # time = []
   # acceleration = []
   # for i,t in enumerate(t_arr):
   #     acc_arr[i] = t;
   #     if t > command_time[j]:
   #         time_array, accelerations = LinearActuatorSimulator.generate_motion_profile(
   #                 move_commands[command_time[j]][0], move_commands[command_time[j]][1], move_commands[command_time[j]][2], move_commands[command_time[j]][3], final_positions[-1], final_velocities[-1]
   #             )
   #         j = j+1
   #     #check if there is anext acceleration
   #     step(a,act_time)
   
   while t < sim_total_time:
       stepped = False
       # Use initial values before command is called
       if initial:
           final_positions.append(initial_position + initial_velocity*t)
           final_velocities.append(initial_velocity)
       
       # Use final position after command is called, but force v = 0
       else:    
           final_positions.append(final_positions[-1])
           final_velocities.append(0)
       final_accelerations.append(0)
       
       # current_position, current_velocity = Motor1.step(final_accelerations[-1], t) # deltatime is wrong because step gets called twice in one iteration the first time gen motion is called

       # When command is called
       for i in move_commands:
           # If time is within command time
           if i > t - 1/(2*sample_rate) and i < t + 1/(2*sample_rate):
               # if command is not first command, force initial vel = 0
               if initial == True: Motor1.initial_velocity = final_velocities[-1]
               else: final_velocities[-1] = 0
               initial = False
               
               time_array, accelerations = LinearActuatorSimulator.generate_motion_profile(
                   move_commands[i][0], move_commands[i][1], move_commands[i][2], move_commands[i][3], final_positions[-1], final_velocities[-1]
               )
               
               # discretize continuous acceleration calculations
               first_t = t
               #idx_next_time = np.where(time_array>t)[0][0]
               while t < first_t + time_array[-1]:
                   if(t < first_t + time_array[1]): final_accelerations.append(accelerations[0])
                   elif(t < first_t + time_array[2]): final_accelerations.append(accelerations[1])
                   elif(t < first_t + time_array[3]): final_accelerations.append(accelerations[2])
                   else: final_accelerations.append(accelerations[3])

                   # update position and velocity
                   stepped = True
                   current_position, current_velocity = Motor1.step(final_accelerations[-1], t)
                   final_positions.append(current_position + initial_position)
                   final_velocities.append(current_velocity)
                   
                   # increment (j is used to display values every second, t is used to increment time array)
                   j+=1
                   display_values(final_accelerations, final_velocities, final_positions, t, j)
                   # time.sleep(1/sample_rate)
                   simulated_times.append(t)
                   t += 1/sample_rate
       
       # Check if iteration already called step  
       if not stepped:
            current_position, current_velocity = Motor1.step(final_accelerations[-1], t) # deltatime is wrong because step gets called twice in one iteration the first time gen motion is called
            stepped = True

       # increment (j is used to display values every second, t is used to increment time array)
       j+=1
       display_values(final_accelerations, final_velocities, final_positions, t, j)
       # time.sleep(1/sample_rate)
       simulated_times.append(t)
       t += 1/sample_rate 
       
   LinearActuatorSimulator.plot_data(simulated_times, final_positions, final_velocities, final_accelerations)

 

def main():
  simulate(40, 10, 5, sample_rate)
sample_rate = 250.0
main()

  # FUNCTION CALL
  # time_array, accelerations = LinearActuatorSimulator.generate_motion_profile(
  #     user_acceleration = 3,
  #     user_goal_position = 20,
  #     user_velocity = 15,
  #     user_deceleration = 10,
  #     initial_position = 0,
  #     initial_velocity = 20,
  # )
