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
        user_goal_position = abs(user_goal_position)
        user_velocity = abs(user_velocity)
        user_deceleration = abs(user_deceleration)
        
        sign = 1
        sign_2 = 1

      # If initial velocity is greater than user velocity, start out decelerating
        if initial_velocity > user_velocity:
            user_acceleration = -user_deceleration

      # Calculate relative values
        relative_goal_position = user_goal_position - initial_position
        relative_user_velocity = user_velocity - initial_velocity

        if relative_goal_position < 0:
            sign = -1
            relative_goal_position = abs(relative_goal_position)

      # Calculate time and distances to extremes
        time_to_max_velocity = relative_user_velocity / user_acceleration
        distance_to_max_velocity = 0.5 * user_acceleration * time_to_max_velocity ** 2 + initial_velocity * time_to_max_velocity

        time_to_no_velocity = user_velocity / user_deceleration
        distance_to_no_velocity = 0.5 * user_deceleration * time_to_no_velocity ** 2

      # Validate User Input is physically possible
        valid = LinearActuatorSimulator.validate_input(
            user_acceleration,
            user_deceleration,
            user_velocity,
            relative_goal_position,
            initial_velocity,
            )
        if not valid:
            print('Not a valid movement')
            return [0,0,0,0], [0,0,0,0]

      # Handle Triangle Motion Profile if necessary
        if distance_to_max_velocity + distance_to_no_velocity > abs(relative_goal_position):
            numerator = ((sqrt(user_deceleration * (user_acceleration + user_deceleration)
                                    * (2 * user_acceleration * relative_goal_position + initial_velocity**2))
                                    - user_acceleration * initial_velocity - user_deceleration * initial_velocity))
                                    
            time_to_max_velocity = numerator / (user_acceleration * (user_acceleration + user_deceleration))
            time_to_no_velocity = numerator / (user_deceleration * (user_deceleration + user_acceleration))
            
            const_v_time = 0
            time_under_initial = initial_velocity / user_deceleration
              
      # If not calculate time with no accel
        else:
            const_v_time = abs(((relative_goal_position
                            - (distance_to_no_velocity
                            + distance_to_max_velocity)) / user_velocity))
            time_under_initial = 0
            
        if initial_position > user_goal_position:
            if initial_velocity < -user_velocity and initial_velocity < -0.1:
                user_deceleration = user_acceleration
                time_to_max_velocity = (abs(initial_velocity) - user_velocity) / user_acceleration
                time_to_no_velocity = user_velocity / user_acceleration
                distance_to_max_velocity = 0.5 * -user_acceleration * time_to_max_velocity ** 2 + initial_velocity * time_to_max_velocity
                distance_to_no_velocity = 0.5 * -user_acceleration * time_to_no_velocity ** 2
                const_v_time = abs((user_goal_position - initial_position - distance_to_max_velocity - distance_to_no_velocity) / user_velocity)
                time_under_initial = 0
                sign_2 = -1
            
            elif initial_velocity > -user_velocity and initial_velocity < -0.1:
                time_to_max_velocity = abs((user_velocity + initial_velocity)/ user_deceleration)
                distance_to_max_velocity = 0.5 * user_deceleration * time_to_max_velocity ** 2 + abs(initial_velocity) * time_to_max_velocity
                time_to_no_velocity = user_velocity / user_acceleration
                distance_to_no_velocity = 0.5 * user_acceleration * time_to_no_velocity ** 2
                const_v_time = abs((initial_position -user_goal_position - distance_to_max_velocity - distance_to_no_velocity) / user_velocity)
                time_under_initial = 0
                sign_2 = -1
                sign *= -1
                user_deceleration, user_acceleration = -user_acceleration, user_deceleration

            elif initial_velocity < user_velocity and initial_velocity > 0.1:
                time_to_max_below = abs(initial_velocity) / user_acceleration
                time_to_max_above = user_velocity / user_acceleration
                time_to_max_velocity = time_to_max_below + time_to_max_above
                distance_to_max_velocity = 0.5 * user_acceleration * time_to_max_above ** 2 - 0.5 * user_acceleration * time_to_max_below ** 2
                time_to_no_velocity = user_velocity / user_deceleration
                distance_to_no_velocity = 0.5 * user_deceleration * time_to_no_velocity ** 2
                time_under_initial = 0
            

            elif initial_velocity > user_velocity and initial_velocity > 0.1:
                user_acceleration = user_deceleration
                time_to_max_velocity = (abs(initial_velocity) + user_velocity) / user_deceleration
                time_to_no_velocity = user_velocity / user_deceleration
                distance_to_max_velocity = 0.5 * user_deceleration * time_to_max_velocity ** 2 - initial_velocity * time_to_max_velocity
                distance_to_no_velocity = 0.5 * user_deceleration * time_to_no_velocity ** 2
                const_v_time = abs((initial_position - user_goal_position - distance_to_max_velocity - distance_to_no_velocity) / user_velocity)
                time_under_initial = 0
                sign_2 = -1
                user_acceleration = -user_acceleration

            
        total_time = time_to_no_velocity + time_to_max_velocity + const_v_time + time_under_initial
        time_array = [0, time_to_max_velocity, time_to_max_velocity + const_v_time, total_time]
        accelerations = [sign_2*sign*user_acceleration, 0, sign*-user_deceleration, 0]

        return time_array, accelerations

    @staticmethod
    def validate_input(
        user_acceleration,
        user_deceleration,
        user_velocity,
        relative_goal_position,
        initial_velocity,
        ):
        
        if user_acceleration == 0 or user_deceleration == 0:
            print('Acceleration and Deceleration must be greater than 0')
            return False
        if user_velocity == 0:
            print('Velocity must be greater than 0')
            return False
        if abs(relative_goal_position) < 0.5 * user_deceleration * (initial_velocity/user_deceleration) ** 2:
            print('Goal position is too close to the initial position')
            return False
        return True


class Operations:

    def __init__(self, initial_velocity, initial_position):
        self.last_value = 0
        self.last_acceleration = 0
        self.current_velocity = initial_velocity
        self.current_position = initial_position

    def step(self, acceleration, t):
        # RK2 > Predict-Correct > Verlet > Euler
        deltatime = t - self.last_value
        self.current_velocity += self.last_acceleration * deltatime
        self.current_position += self.current_velocity * deltatime + 0.5 * acceleration * deltatime**2
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
    plt.plot(discretized_times, discretized_accel,
    label='Acceleration')
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
    move_commands = {1: (6, 120, 5, 8),
                     30: (2, 110, 20, 4)}

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
    simulate(60, 0, 6)

sample_rate = 10000.0
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