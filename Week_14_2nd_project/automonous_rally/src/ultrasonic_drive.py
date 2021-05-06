#!/usr/bin/env python
import time
class ULTRASONIC:
    angle = 0
    speed = 0
    back = False
    before_angle = 0 
    def __init__(self):
        pass

    def set_motor(self, angle, speed):
        self.angle = angle
        self.speed = speed

    def set_data(self, distance):
        if min(distance["FM"],distance["FL"]+15,distance["FR"]+15)<35 or self.back :
            if not self.back:
                self.before_angle =  -self.angle  
            self.back = True

            if distance["FM"]>45:
                self.back = False
                #self.before_angle = 0         
            self.set_motor(self.before_angle,-30)
        elif abs(distance["L"] - distance["R"]) >100:           
            if distance["L"] < distance["R"]:
                self.set_motor(30,50)
            else:
                self.set_motor(-30,50)

           

        elif distance["FM"]<60:
            d = distance["FR"] - distance["FL"]
            if distance["FL"] < distance["FR"]:
                #self.set_motor((d/2 if d<50 else 50),35)
                self.set_motor(50,40)
            else:
                #self.set_motor((d/2 if d>-50 else -50),35)
                self.set_motor(-50,40)
        else:
            if distance["FL"] < distance["FR"]:
                self.set_motor(20,50)
            elif abs(distance["FL"] - distance["FR"])<5:
                self.set_motor(0,50)
            else:
                self.set_motor(-20,50)

        

    def get_motor(self):
        return self.angle, self.speed
