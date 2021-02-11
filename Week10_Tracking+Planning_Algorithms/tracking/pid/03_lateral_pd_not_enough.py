import numpy as np
import matplotlib.pyplot as plt

# paramters
dt = 0.1
L = 3.0


class LateralPlant(object):
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

        self.max_steering = np.radians(30)

    def update(self, steer, a=0):
        steer += np.radians(3)  # input bias
        steer = np.clip(steer, -self.max_steering, self.max_steering)
        self.x += self.v * np.cos(self.yaw) * dt
        self.y += self.v * np.sin(self.yaw) * dt
        self.yaw += self.v / L * np.tan(steer) * dt
        self.yaw = self.yaw % (2.0 * np.pi)
        self.v += a * dt


if __name__ == "__main__":
    plant = LateralPlant(x=0.0, y=0.0, yaw=0.0, v=1.0)
    target_y = 1.0
    kp = 0.4
    kd = 2.0

    plt.figure()
    ys = []
    ts = []
    prev_error = plant.y - target_y
    for step in range(400):
        # plt.clf()
        t = step * dt
        error = plant.y - target_y
        diff_error = error - prev_error
        prev_error = error

        steer = - kp * error - kd * diff_error/dt
        plant.update(steer)

        ys.append(plant.y)
        ts.append(t)

    plt.plot([0, ts[-1]], [target_y, target_y], 'r-', label="reference")
    plt.plot(ts, ys, 'b-', label="PD controller")
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.legend(loc="best")
    plt.axis("equal")

    plt.show()

