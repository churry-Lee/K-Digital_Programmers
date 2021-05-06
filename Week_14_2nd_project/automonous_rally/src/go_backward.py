#!/usr/bin/env python
import time
class BACKWARD:
    angle = 0
    speed = 0
    count = 0 
    sucess = False
    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed

    def set_data(self, data):
        distance= data
        if not self.sucess: 
            if distance["FM"]<150:
                time.sleep(1)
                if self.count%2==0:
                    self.set_motor(50,-30)
                    if distance["BM"]<50 :
                        self.count +=1
                else:
                    self.set_motor(-50,30)
                    if distance["FM"]<50 :
                        self.count +=1
                
            else:
                self.sucess = True
        else:
            if distance["FL"] < distance["FR"]:
                self.set_motor(20,30)
            elif abs(distance["FL"] - distance["FR"])<5:
                self.set_motor(0,30)
            else:
                self.set_motor(-20,30)
        

    def get_motor(self):
        return self.angle, self.speed
