import numpy as np
import matplotlib.pyplot as plt

# paramters
dt = 0.1

k = 0.5  # control gain

# GV70 PARAMETERS
LENGTH = 4.715
WIDTH = 1.910
L = 2.875
BACKTOWHEEL = 1.0
WHEEL_LEN = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
TREAD = 0.8  # [m]


def normalize_angle(angle):
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

# x, y, yaw: rear current position
# v: velocity estimate
# map_xs, map_ys, map_yaws: reference path, map_yaws = heading
def stanley_control(x, y, yaw, v, map_xs, map_ys, map_yaws):
    # find nearest waypoint
    min_dist = 1e9
    min_index = 0
    n_points = len(map_xs)

    # convert rear_wheel x-y coordinate to front_wheel x-y coordinate
    # L = wheel base
    front_x = x + L * np.cos(yaw)
    front_y = y + L * np.sin(yaw)

    for i in range(n_points):
        # calculating distance (map_xs, map_ys) - (front_x, front_y)
        dx = front_x - map_xs[i]
        dy = front_y - map_ys[i]
        dist = np.sqrt(dx**2 + dy**2)

        if dist < min_dist:
            min_dist = dist
            min_index = i

    # compute cte at front axle
    map_x = map_xs[min_index]
    map_y = map_ys[min_index]
    map_yaw = map_yaws[min_index]
    # nearest x-y coordinate (map_x, map_y) - front_wheel coordinate (front_x, front_y)
    dx = map_x - front_x
    dy = map_y - front_y

    perp_vec = [np.cos(yaw + np.pi/2), np.sin(yaw + np.pi/2)]
    cte = np.dot([dx, dy], perp_vec)

    # control law
    # heading error = yaw_term
    yaw_term = normalize_angle(map_yaw - yaw)
    cte_term = np.arctan2(k*cte, v)

    # steering
    steer = yaw_term + cte_term
    return steer


