from matplotlib import pyplot as plt
import numpy as np
from math import sqrt

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
        user_velocity = abs(user_velocity)
        user_deceleration = abs(user_deceleration)
        
        sign = 1
        time_under_initial = 0

      # If initial velocity is greater than user velocity, start out decelerating
        if initial_velocity > user_velocity:
            user_acceleration = -user_deceleration

      # Calculate relative values
        relative_goal_position = user_goal_position - initial_position
                
        (
         time_to_max_velocity, 
         distance_to_max_velocity, 
         time_to_no_velocity, 
         distance_to_no_velocity, 
         user_acceleration, 
         user_deceleration, 
         relative_goal_position, 
         sign) = LinearActuatorSimulator.calculate_time_distance(
             initial_position, 
             user_goal_position, 
             initial_velocity, 
             user_velocity, 
             user_acceleration, 
             user_deceleration, 
             relative_goal_position, 
             sign
             ) 
         
      # Validate User Input is physically possible
        valid = LinearActuatorSimulator.validate_input(
            user_acceleration,
            user_deceleration,
            user_velocity,
            relative_goal_position,
            initial_velocity,
            sign
            )
        if not valid:
            print('Not a valid movement')
            return [0,0,0,0], [0,0,0,0]
        
        # Handle triangle profile if necessary
        if distance_to_max_velocity + distance_to_no_velocity > relative_goal_position:
            if initial_position > user_goal_position:    
                user_acceleration = abs(user_acceleration)
                user_deceleration = abs(user_deceleration)
                initial_velocity = -initial_velocity
            time_to_max_velocity, time_to_no_velocity, const_v_time, time_under_initial = LinearActuatorSimulator.triangle_profile(
                user_acceleration, 
                user_deceleration, 
                initial_velocity, 
                relative_goal_position
            )
                
        else:
            const_v_time = abs((relative_goal_position - distance_to_max_velocity - distance_to_no_velocity) / user_velocity)
  
        total_time = time_to_no_velocity + time_to_max_velocity + const_v_time + time_under_initial
        time_array = [0, time_to_max_velocity, time_to_max_velocity + const_v_time, total_time]
        accelerations = [sign * user_acceleration, 0, sign * -user_deceleration, 0]

        return time_array, accelerations

    @staticmethod
    def validate_input(
        user_acceleration,
        user_deceleration,
        user_velocity,
        relative_goal_position,
        initial_velocity,
        sign
        ):
        
        if user_acceleration == 0 or user_deceleration == 0:
            print('Acceleration and Deceleration must be greater than 0')
            return False
        if user_velocity == 0:
            print('Velocity must be greater than 0')
            return False
        if relative_goal_position < 0.5 * abs(user_deceleration) * (initial_velocity/user_deceleration) ** 2 and sign == 1:
            print('Goal position is too close to the initial position')
            return False
        elif relative_goal_position < 0.5 * abs(user_acceleration) * (initial_velocity/user_acceleration) ** 2 and sign == -1:
            print('Goal position is too close to the initial position')
            return False
        return True


    @staticmethod
    def triangle_profile(user_acceleration, user_deceleration, initial_velocity, relative_goal_position):
        numerator = ((sqrt(user_deceleration * (user_acceleration + user_deceleration)
                                * (2 * user_acceleration * relative_goal_position + initial_velocity**2))
                                - user_acceleration * initial_velocity - user_deceleration * initial_velocity))
                                   
        time_to_max_velocity = numerator / (user_acceleration * (user_acceleration + user_deceleration))
        time_to_no_velocity = numerator / (user_deceleration * (user_deceleration + user_acceleration))
        time_under_initial = initial_velocity / user_deceleration
        
        return time_to_max_velocity, time_to_no_velocity, 0, time_under_initial
    
    @staticmethod
    def calculate_time_distance(initial_position, user_goal_position, initial_velocity, user_velocity, user_acceleration, user_deceleration, relative_goal_position, sign):
        if initial_position > user_goal_position:
            sign = -1
            relative_goal_position = abs(relative_goal_position)

            if initial_velocity < 0:
                time_to_no_velocity = user_velocity / user_acceleration
                distance_to_no_velocity = 0.5 * user_acceleration * time_to_no_velocity ** 2

                if initial_velocity < -user_velocity:
                    user_acceleration, user_deceleration = -user_acceleration, user_acceleration
                elif initial_velocity > -user_velocity:
                    user_deceleration, user_acceleration = user_acceleration, user_deceleration

                time_to_max_velocity = abs((user_velocity + initial_velocity) / user_acceleration)
                distance_to_max_velocity = 0.5 * user_acceleration * time_to_max_velocity ** 2 + abs(initial_velocity) * time_to_max_velocity

            else:
                time_to_max_velocity = (initial_velocity + user_velocity) / user_deceleration

                if initial_velocity < user_velocity:
                    user_deceleration, user_acceleration = user_acceleration, user_deceleration
                    distance_to_max_velocity = 0.5 * user_acceleration * (user_velocity / user_acceleration) ** 2 - 0.5 * user_acceleration * (initial_velocity / user_acceleration) ** 2

                elif initial_velocity > user_velocity:
                    user_acceleration, user_deceleration = user_deceleration, user_deceleration
                    distance_to_max_velocity = 0.5 * user_deceleration * time_to_max_velocity ** 2 - initial_velocity * time_to_max_velocity

                time_to_no_velocity = abs(user_velocity / user_deceleration)    
                distance_to_no_velocity = 0.5 * user_deceleration * time_to_no_velocity ** 2

        else:        
            time_to_max_velocity = (user_velocity - initial_velocity) / user_acceleration
            distance_to_max_velocity = 0.5 * user_acceleration * time_to_max_velocity ** 2 + initial_velocity * time_to_max_velocity
            time_to_no_velocity = user_velocity / user_deceleration
            distance_to_no_velocity = 0.5 * user_deceleration * time_to_no_velocity ** 2

        return time_to_max_velocity, distance_to_max_velocity, time_to_no_velocity, distance_to_no_velocity, user_acceleration, user_deceleration, relative_goal_position, sign
    

class Operations:
    def __init__(self, initial_velocity, initial_position):
        self.last_value = 0
        self.last_acceleration = 0
        self.current_velocity = initial_velocity
        self.current_position = initial_position

    def step(self, acceleration, t):
        # RK2 > Predict-Correct > Euler
        # deltatime = t - self.last_value
        # self.current_velocity += self.last_acceleration * deltatime
        # self.current_position += self.current_velocity * deltatime + 0.5 * acceleration * deltatime ** 2
        # self.last_value = t
        # self.last_acceleration = acceleration
        # return self.current_position, self.current_velocity
        
        # RK2 
        deltatime = t - self.last_value
        k1_v = self.last_acceleration
        k1_p = self.current_velocity

        self.current_velocity += k1_v * deltatime / 2
        k2_v = acceleration
        k2_p = self.current_velocity + k1_p * deltatime / 2

        self.current_velocity += k2_v * deltatime - k1_v * deltatime / 2
        self.current_position += k2_p * deltatime

        self.last_value = t
        self.last_acceleration = acceleration
        return self.current_position, self.current_velocity
    

def plot_data(
    discretized_times,
    positions,
    velocities,
    discretized_accel,
    ):

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


def simulate(
    sim_total_time,
    initial_position,
    initial_velocity,
    ):
    
    sim_time_array = np.arange(0, sim_total_time, 1/sample_rate)
    sim_accelerations = np.zeros_like(sim_time_array)
    sim_positions = np.zeros_like(sim_time_array)
    sim_velocities = np.zeros_like(sim_time_array)

    Motor1 = Operations(initial_velocity, initial_position)

    # Time : (user_accel, user_goal_pos, user_velocity, user_decel)
    move_commands = {10: (4, 0, 10, 8),
                    40: (2, 100, 10, 4)}

    command_time = list(move_commands.keys())
    j = 0

    for (i, t) in enumerate(sim_time_array):
        if j < len(command_time) and t > command_time[j] - 1 / (2 * sample_rate) and t < command_time[j] + 1 / (2 * sample_rate):
            continuous_time_array, continuous_accelerations = LinearActuatorSimulator.generate_motion_profile(
                *move_commands[command_time[j]],
                sim_positions[i - 1],
                sim_velocities[i - 1])
            continuous_time_array += t
            j += 1

            for k in range(3):
                indices = np.where((sim_time_array > continuous_time_array[k]) 
                                   & (sim_time_array < continuous_time_array[k + 1]))[0]
                sim_accelerations[indices] = continuous_accelerations[k]
                
        sim_positions[i], sim_velocities[i] = Motor1.step(sim_accelerations[i], t)
        
    print(sim_accelerations[-1])
    print(sim_velocities[-1])
    print(sim_positions[-1])

    plot_data(sim_time_array, sim_positions, sim_velocities, sim_accelerations)


def main():
    # (sim_total_time, initial_position, initial_velocity)
    simulate(60, 150, -12)

sample_rate = 1000.0
main()