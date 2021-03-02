import numpy as np
import matplotlib.pyplot as plt

k = 2.0  # control gain
kp = 0.01  # speed proportional gain
dt = 0.1  # [s] time difference
l = 30.0  # [m] wheel base of vehicle
max_steer = np.radians(20.0)  # [rad] max steering angle

show_animation = True

class state(object):
    """
    class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """instantiate the object."""
        super(state, self).__init__()
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def update(self, acceleration, delta):
        """
        update the state of the vehicle.

        stanley control uses bicycle model.

        :param acceleration: (float) acceleration
        :param delta: (float) steering
        """
        delta = np.clip(delta, -max_steer, max_steer)

        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / l * np.tan(delta) * dt
        self.yaw = normalize_angle(self.yaw)
        self.v += acceleration * dt


def pid_control(target, current):
    """
    proportional control for the speed.

    :param target: (float)
    :param current: (float)
    :return: (float)
    """
    return kp * (target - current)


def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    stanley steering control.

    :param state: (state object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # steering control
    delta = theta_e + theta_d

    return delta, current_target_idx


def normalize_angle(angle):
    """
    normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle


def calc_target_index(state, cx, cy):
    """
    compute index in the trajectory list of the target.

    :param state: (state object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # calc front axle position
    fx = state.x + l * np.cos(state.yaw)
    fy = state.y + l * np.sin(state.yaw)

    # search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # project rms error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle
