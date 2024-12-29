import json
import matplotlib.pyplot as plt
from math import *
import numpy as np
import sys
import time

class FABRIK:
    def __init__(self, config, tolerance=1e-3):
        """
        Initialize the FABRIK solver.

        :param config: Configuration loaded from JSON defining joints and linkers.
        :param tolerance: Convergence tolerance for the end effector.
        """
        self.joints, self.angles, self.limits, self.link_lengths = self.parse_config(config)
        self.tolerance = tolerance

    def normalize(self, theta):
        """
        Normaliza un ángulo para que esté en el rango [-pi, pi].
        Parámetros:
            theta (float): Ángulo a normalizar.
        Retorna:
            float: Ángulo normalizado en el rango [-pi, pi].
        """
        # si se sale del rango [-pi, pi] se normaliza
        if theta < -pi or theta > pi:
            if theta % pi == 0:
                if theta % (2 * pi) == 0:
                    theta = 0
                else:
                    sign = -1 if theta > 0 else 1
                    theta = sign * pi
            else:
                theta = (theta + pi) % (2 * pi) - pi
        return theta

    def parse_config(self, config):
        """
        Parse the configuration to extract joints and link lengths.

        :param config: Configuration dictionary from JSON.
        :return: Tuple of joints (initial positions), angles, limits, and link lengths.
        """
        joints = []
        angles = []
        limits = []
        link_lengths = []
        current_position = np.array([0.0, 0.0])
        current_angle = 0.0

        for key, element in config.items():
            if element['type'] == 'joint':
                joints.append(current_position.copy())
                joint_limits = element['limits']
                if joint_limits != [None, None]:
                    joint_limits[0] = self.normalize(radians(joint_limits[0]))
                    joint_limits[1] = self.normalize(radians(joint_limits[1]))
                else:
                    joint_limits = [-pi, pi]
                limits.append(joint_limits)
                value = self.normalize(radians(element['value']))
                if joint_limits[0] <= value <= joint_limits[1]:
                    angles.append(value)
                else:
                    angles.append(0.0)
            elif element['type'] == 'linker':
                length = element['value']
                link_lengths.append(length)
                current_angle += angles[-1]
                current_position += np.array([length * cos(current_angle), length * sin(current_angle)])
            elif element['type'] == 'final':
                joints.append(current_position.copy())
                angles.append(0.0)
                limits.append([0.0, 0.0])

        return np.array(joints), np.array(angles), np.array(limits), link_lengths

    def plot_joints(self, target, iteration, stage, disconnected_links=None, stop=False):
        """
        Plot the current state of the joints and links.

        :param target: Target position.
        :param iteration: Current iteration number.
        :param stage: Current stage (e.g., Forward/Backward).
        :param disconnected_links: List of indices for disconnected links.
        :param active_link: Index of the currently active link (being adjusted).
        """
        plt.clf()
        arm_length = sum(self.link_lengths) + 1
        x = [joint[0] for joint in self.joints]
        y = [joint[1] for joint in self.joints]

        # Plot links
        for i in range(len(self.joints) - 1):
            if disconnected_links is not None and i == disconnected_links:
                # Plot disconnected link as dashed
                plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], 'r--', label='Disconnected' if i == 0 else "")
            else:
                # Plot normal link
                plt.plot([x[i], x[i + 1]], [y[i], y[i + 1]], 'b-', label='Connected' if i == 0 else "")

        # Plot joints 
        plt.plot(x, y, 'ko', markersize=10) 
        plt.plot(target[0], target[1], 'r*', markersize=15, label='Target') 
        plt.xlim(-arm_length, arm_length) 
        plt.ylim(-arm_length, arm_length) 
        plt.grid(True) 
        plt.title(f"{stage} - Iteration {iteration+1}") 
        plt.legend() 
        plt.pause(0.1)
        #if stop:
        #    plt.show()

    def solve(self, target):
        """
        Perform FABRIK algorithm to reach the target, visualizing disconnection and reconnection.

        :param target: Target position (x, y).
        :return: List of joint positions after convergence.
        """
        target = np.array(target)
        iteration = 0

        if np.linalg.norm(self.joints[-1] - target) > sum(self.link_lengths):
            print("Target unreachable!")
            return self.joints

        while True:
            print(f"Iteration {iteration}")
            # Plot initial state for this iteration
            self.plot_joints(target, iteration, "Initial")

            # Forward reaching (disconnect links)
            self.joints[-1] = target  # Move end effector to target
            disconnected_links = []
            for i in range(len(self.joints) - 2, -1, -1):
                r = np.linalg.norm(self.joints[i + 1] - self.joints[i])
                lambda_ = self.link_lengths[i] / r
                self.joints[i] = (1 - lambda_) * self.joints[i + 1] + lambda_ * self.joints[i]
                disconnected_links.append(i)
                self.plot_joints(target, iteration, "Forward", disconnected_links=i)

            # Backward reaching (reconnect links)
            self.joints[0] = np.array([0.0, 0.0])  # Reset root to initial position
            disconnected_links = []
            for i in range(len(self.joints) - 1):
                r = np.linalg.norm(self.joints[i + 1] - self.joints[i])
                lambda_ = self.link_lengths[i] / r
                self.joints[i + 1] = (1 - lambda_) * self.joints[i] + lambda_ * self.joints[i + 1]
                disconnected_links.append(i)
                self.plot_joints(target, iteration, "Backward", disconnected_links=i)

            # Check convergence
            diff = np.linalg.norm(self.joints[-1] - target)
            if diff <= self.tolerance:
                break

            iteration += 1

        # Final plot
        self.plot_joints(target, iteration, "Final", stop=True)
        return self.joints


def load_configuration_from_json(file_path):
    """Load arm configuration from a JSON file."""
    with open(file_path, 'r') as file:
        data = json.load(file)
    return data


# Example usage
if __name__ == "__main__":
    if len(sys.argv) != 4:
        print("Usage: python fabrik.py <arm_configuration> <target_x> <target_y>")
        sys.exit(1)

    # Load configuration from JSON
    config = load_configuration_from_json(sys.argv[1])

    # Initialize FABRIK solver
    fabrik = FABRIK(config)

    # Solve and visualize
    target_x, target_y = float(sys.argv[2]), float(sys.argv[3])
    start = time.time()
    fabrik.solve((target_x, target_y))
    end = time.time()
    print(f"Execution time: {end - start:.2f} seconds")