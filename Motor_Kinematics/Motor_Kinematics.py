from matplotlib import pyplot as plt
import numpy as np
from sympy import symbols, Eq, solve


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

      # If initial velocity is greater than user velocity, start out decelerating
        if initial_velocity > user_velocity:
            user_acceleration = -user_deceleration

      # Calculate relative values
        relative_goal_position = user_goal_position - initial_position
        relative_user_velocity = user_velocity - initial_velocity

      # if relative_goal_position < 0:
      #     user_acceleration = -user_acceleration

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
            user_goal_position,
            initial_position,
            initial_velocity,
            time_to_max_velocity,
            time_to_no_velocity,
            )
        if not valid:
            print ('Not a valid movement')
            exit()

      # Handle Triangle Motion Profile if necessary
        if distance_to_max_velocity + distance_to_no_velocity > relative_goal_position:
            time_to_max_velocity, time_to_no_velocity = LinearActuatorSimulator.solve_for_time(user_acceleration,
                    user_deceleration, relative_goal_position,
                    initial_velocity)
            const_v_time = 0
            time_under_initial = initial_velocity / user_deceleration
        else:

      # If not calculate time with no accel
            const_v_time = (relative_goal_position
                            - (distance_to_no_velocity
                            + distance_to_max_velocity)) / user_velocity
            time_under_initial = 0
            
        total_time = time_to_no_velocity + time_to_max_velocity + const_v_time + time_under_initial
        time_array = [0, time_to_max_velocity, time_to_max_velocity + const_v_time, total_time]
        accelerations = [user_acceleration, 0, -user_deceleration, 0]

        return time_array, accelerations

    @staticmethod
    def solve_for_time(
        user_acceleration,
        user_deceleration,
        relative_goal_position,
        initial_velocity,
        ):
        (time_to_max_velocity, time_to_no_velocity) = symbols('time_to_max_velocity time_to_no_velocity')
        equation1 = Eq(initial_velocity * time_to_max_velocity + 0.5
                       * user_acceleration * time_to_max_velocity ** 2
                       + initial_velocity * time_to_no_velocity + 0.5
                       * user_deceleration * time_to_no_velocity ** 2
                       + initial_velocity ** 2 / (2
                       * user_deceleration), relative_goal_position)
        equation2 = Eq(user_acceleration * time_to_max_velocity
                       - user_deceleration * time_to_no_velocity, 0)
        solutions = solve((equation1, equation2),
                          (time_to_max_velocity, time_to_no_velocity))

      # Filter out negative solutions
        positive_solutions = [solution for solution in solutions
                              if solution[0] > 0 and solution[1] > 0]

      # If there are positive solutions return
        if positive_solutions:
            return positive_solutions[0]
        else:
            print('No positive solutions')
            exit()

    @staticmethod
    def validate_input(
        user_acceleration,
        user_deceleration,
        user_velocity,
        user_goal_position,
        initial_position,
        initial_velocity,
        time_to_max_velocity,
        time_to_no_velocity,
        ):
        
        if user_acceleration == 0 or user_deceleration == 0:
            print('Acceleration and Deceleration must be greater than 0')
            return False
        if user_velocity == 0:
            print('Velocity must be greater than 0')
            return False

        return True

    @staticmethod
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


class Operations:

    def __init__(self, initial_velocity, initial_position):
        self.last_value = 0
        self.acceleration = 0
        self.acceleration_array = []
        self.initial_velocity = initial_velocity
        self.initial_position = initial_position

    def step(self, acceleration, t):
        deltatime = t - self.last_value
        self.acceleration_array.append(acceleration)
        velocities = np.cumsum(self.acceleration_array) * deltatime + self.initial_velocity
        positions = np.cumsum(velocities) * deltatime + velocities[-1] * deltatime + self.initial_position
        self.last_value = t
        return positions[-1], velocities[-1]


def simulate(
    sim_total_time,
    initial_position,
    initial_velocity,
    ):
    
    sim_time_array = np.arange(0, sim_total_time, 1 / sample_rate)
    sim_accelerations = np.zeros_like(sim_time_array)
    sim_positions = np.zeros_like(sim_time_array)
    sim_velocities = np.zeros_like(sim_time_array)

    Motor1 = Operations(initial_velocity, initial_position)

    # Time : (user_accel, user_goal_pos, user_velocity, user_decel)
    move_commands = {3: (6, 50, 5, 8), 
                     25: (2, 185, 50, 4)}

    command_time = list(move_commands.keys())
    j = 0

    for (i, t) in enumerate(sim_time_array):
        if j < len(command_time):
            if t > command_time[j] - 1 / (2 * sample_rate) and t < command_time[j] + 1 / (2 * sample_rate):
                (continuous_time_array, continuous_accelerations) = LinearActuatorSimulator.generate_motion_profile(
                    move_commands[command_time[j]][0],
                    move_commands[command_time[j]][1],
                    move_commands[command_time[j]][2],
                    move_commands[command_time[j]][3],
                    sim_positions[i - 1],
                    sim_velocities[i - 1],
                    )

                continuous_time_array += t
                j += 1

                for k in range(3):
                    indices = np.where((sim_time_array > continuous_time_array[k]) & (sim_time_array < continuous_time_array[k + 1]))[0]
                    sim_accelerations[indices] = continuous_accelerations[k]

        sim_positions[i], sim_velocities[i] = Motor1.step(sim_accelerations[i], t)

    print(sim_accelerations[-1])
    print(sim_velocities[-1])
    print(sim_positions[-1])

    LinearActuatorSimulator.plot_data(sim_time_array, sim_positions, sim_velocities, sim_accelerations)


def main():
    simulate(40, 10, 6)

sample_rate = 40.0
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
