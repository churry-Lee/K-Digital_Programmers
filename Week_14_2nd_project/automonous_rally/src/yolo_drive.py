#!/usr/bin/env python
import math

class YOLO:
    
    def __init__(self):
	self.x = 320
	self.angle = 0
	self.speed = 0
	self.follow = "jdy"
	self.not_follow = "traffic light"
	self.obj = "jdy"
	self.route = "center"
	self.other = "traffic light"
	self.other_x = 320
	self.obj_x = 0

    def set_motor(self, angle, speed):
        self.angle = angle
        self.motor = speed

    def set_data(self, data):
	qrcodes = data[0]
	bbs = data[1]
	us = data[2]
	l0_sonar = us["L"]
	r4_sonar = us["R"]
	l1_sonar = us["FL"]
	r3_sonar = us["FR"]

	diff1 = l0_sonar - r4_sonar
	lane_width = 60
	width = l0_sonar + r4_sonar
  	print(width)
	diff2 = l1_sonar - r3_sonar
	for qr in qrcodes:
		print qr
		if qr.data == "6 yolo pottedplant":
			self.follow = "pottedplant"
		elif qr.data == "5 yolo bottle":
			self.follow = "bottle"
		elif qr.data == "4 dqn dqn_drive_end":
			pass
			
	for bb in bbs:
		if bb["class"] == self.follow:
			self.obj = self.follow
			self.obj_x = (bb["xmin"] + bb["xmax"]) // 2
			angle_ = (self.obj_x - 320) // 4
			if self.other_x < self.obj_x:
				self.route = "right"
			else:
				self.route = "left"
		else:
			angle_ = 0
		#elif bb["class"] == self.not_follow:
			
		#	self.other_x = (bb["xmin"] + bb["xmax"]) // 2
	#h = math.atan2(diff2, diff1)
	#he = abs(34 - math.radians(h))
	#if diff2 == 0:
	#	sign = 0
	#else:
	#	sign = abs(diff2) / diff2
	ratio = lane_width / width
	if ratio > 1:
		ratio = 1
	else:
		pass
	raw_angle = math.acos(ratio)
	yaw =  int(round(math.degrees(raw_angle) * 5 / 3))
	if diff1 < 0:
		yaw_term = yaw
	else:
		yaw_term = - yaw

	if self.route == "right":
		cte_term = (30 - diff1) * math.cos(raw_angle)
		 
	elif self.route == "left":
		cte_term = (-30 - diff1) * math.cos(raw_angle)

	else:
		cte_term = -diff1 / 3

	angle = yaw_term + cte_term
	#self.x = (bb["xmin"] + bb["xmax"]) // 2
	print(bbs, self.follow)
	print(self.obj_x, self.other_x)
	print(self.route, self.obj, diff1, diff2)
	#angle = (self.x - 320) // 4
	speed = 35
	self.set_motor(angle_, speed)

    def get_motor(self):
        return self.angle, self.motor
