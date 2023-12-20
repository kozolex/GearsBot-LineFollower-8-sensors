#!/usr/bin/env python3

# Import the necessary libraries
import time
import math
from ev3dev2.motor import *
from ev3dev2.sound import Sound
from ev3dev2.sensor import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor.virtual import *

# Create the sensors and motors objects
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
left_motor = motorA
right_motor = motorB
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)

spkr = Sound()
radio = Radio()

cs_0l = ColorSensor(INPUT_1)
cs_1l = ColorSensor(INPUT_2)
cs_2l = ColorSensor(INPUT_3)
cs_3l = ColorSensor(INPUT_4)
cs_0p = ColorSensor(INPUT_5)
cs_1p = ColorSensor(INPUT_6)
cs_2p = ColorSensor(INPUT_7)
cs_3p = ColorSensor(INPUT_8)
touch_sensor = TouchSensor(INPUT_9)

def line_position(trashHoldBlackLine = 90):
    sensorsList = [
        cs_3l,
        cs_2l,
        cs_1l,
        cs_0l,
        cs_0p,
        cs_1p,
        cs_2p,
        cs_3p]
        
    linePosValue = 0
    oneDetect = False
    
    for idx, cs in enumerate(sensorsList[1:-1]):
        
        if cs.reflected_light_intensity < trashHoldBlackLine:
            linePosValue+= (idx-3)
            oneDetect = True
            #print(idx)
    #print (linePosValue)
    return oneDetect, linePosValue

def smoothSpeed(maxSpeed, acc=2):
    for speed in range(maxSpeed):
        steering_drive.on(0, -speed)
        time.sleep(.01*acc)
    return maxSpeed
    
lineValue = 0
last_line_position = 0
steringValue = 0
P = 0 
I = 0
D = 0

Kp = 14
Kd = 10
Ki = 0.05


startTime = time.time()
speed = smoothSpeed(75)

while touch_sensor.is_pressed == False:
    
    oneDetect, lv = line_position()
    if oneDetect:
        P = lv* Kp
        D = Kd * (lv - last_line_position)
        I = 0
        last_line_position = lv
        #print(lv)

    
    steringValue = P + I + D
    
    I += steringValue * Ki
    #print(I)
    if steringValue > 100: steringValue = 100
    if steringValue < -100: steringValue = -100

    steering_drive.on(steringValue, -speed)

steering_drive.off(brake=True)

print(time.time() - startTime)
