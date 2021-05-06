#!/usr/bin/env python

from model import *
import numpy as np
from collections import deque
import numpy as np
class DQN:

	def __init__(self, input_size, hidden_size, output_size, pth_number, pth_path, sonar_max):
		study_init(input_size, hidden_size, output_size, "DQN")
		self.load_pth(pth_path, pth_number)
		self.speed = 0
		self.angle = 0
		self.maxlen = 3
		self.q = []
		q0 = deque(maxlen=self.maxlen)
		q1 = deque(maxlen=self.maxlen)
		q2 = deque(maxlen=self.maxlen)
		q3 = deque(maxlen=self.maxlen)
		q4 = deque(maxlen=self.maxlen)
		self.q.append(q0)
		self.q.append(q1)
		self.q.append(q2)
		self.q.append(q3)
		self.q.append(q4)
		self.sonar_max = sonar_max

	def load_pth(self, path, pth_number):
		study_model_load(path=path, episode=pth_number)

	def set_motor(self, angle, speed):
		self.angle = angle
		self.speed = speed

	def set_data(self, data, dqn_name="jdy1"):
		angle = self.angle
		ultra_data = data[0]
		angle = data[1]
		state = [float(ultra_data["L"]), float(ultra_data["FL"]), float(ultra_data["FM"]), float(ultra_data["FR"]), float(ultra_data["R"]), float(angle)]
		for i in range(len(state) - 1):
			self.q[i].append(state[i])
		for i in range(len(state) - 1):
			val = np.mean(self.q[i])
			state[i] = float(val) if val < self.sonar_max else self.sonar_max
		
		print(state)

		if dqn_name == "xytron":
			np.array(state) * 20 / 9
			state.append(0.0)

		state = np.array(state)

		if dqn_name == "jdy1" or dqn_name == "jdy2":
			state[3] -= 40
			state_sum = sum(state) - state[-1]
			state = state / state_sum
			state[-1] *= state_sum / 50

		print(state)
		action = study_get_action(state)
		action_set = [[-1,-1], [0,0], [1,1], [-1,-1], [0,0], [1,1], [-1,-1], [0,0], [1,1]]
		act = action_set[action]
		if dqn_name == "jdy1":
			if action == 2:
				angle += 20.0
			elif action == 1:
				angle = 0
			elif action == 0:
				angle -= 30.0
			speed = 39
		elif dqn_name == "xytron" or dqn_name == "jdy2":
			if act[0] == -1:
				angle -= 15.0
			elif act[0] == 0:
				angle = 0
			elif act[0] == 1:
				angle += 15.0
			
			if act[1] == -1:
				speed = -35
			elif act[1] == 0:
				speed = 5
			elif act[1] == 1:
				speed = 35

		angle = angle if abs(angle) < 50 else 50 * abs(angle)/angle
		print(action)
		self.set_motor(angle, speed)

	def get_motor(self):
		return self.angle, self.speed

