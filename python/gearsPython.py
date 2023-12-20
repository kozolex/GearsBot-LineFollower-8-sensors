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
motorA = LargeMotor(OUTPUT_A)  # Create a LargeMotor object for motor A
motorB = LargeMotor(OUTPUT_B)  # Create a LargeMotor object for motor B
left_motor = motorA  # Assign motor A as the left motor
right_motor = motorB  # Assign motor B as the right motor
tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)  # Create a MoveTank object for controlling both motors
steering_drive = MoveSteering(OUTPUT_A, OUTPUT_B)  # Create a MoveSteering object for controlling both motors

spkr = Sound()  # Create a Sound object for playing sounds
radio = Radio()  # Create a Radio object for radio communication

# Create ColorSensor objects for each color sensor
cs_0l = ColorSensor(INPUT_1)
cs_1l = ColorSensor(INPUT_2)
cs_2l = ColorSensor(INPUT_3)
cs_3l = ColorSensor(INPUT_4)
cs_0p = ColorSensor(INPUT_5)
cs_1p = ColorSensor(INPUT_6)
cs_2p = ColorSensor(INPUT_7)
cs_3p = ColorSensor(INPUT_8)
touch_sensor = TouchSensor(INPUT_9)  # Create a TouchSensor object for the touch sensor

# Function to determine the position of the line
def line_position(trashHoldBlackLine = 20):
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
    
    # Loop through each sensor in the list
    for idx, cs in enumerate(sensorsList):
        # If the reflected light intensity is less than the threshold
        if cs.reflected_light_intensity < trashHoldBlackLine:
            linePosValue+= (idx-4)
            oneDetect = True
    return oneDetect, linePosValue  # Return whether a line was detected and the line position

# Function to smoothly increase the speed of the robot
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

Kp = 15  # Proportional gain
Kd = 12  # Derivative gain
Ki = 0.07  # Integral gain

startTime = time.time()  # Record the start time
speed = smoothSpeed(85)  # Start the robot moving

# Main loop
while touch_sensor.is_pressed == False:  # Continue until the touch sensor is pressed
    
    oneDetect, lv = line_position()  # Get the line position
    if oneDetect:  # If a line was detected
        P = lv* Kp  # Calculate the proportional term
        D = Kd * (lv - last_line_position)  # Calculate the derivative term
        I = 0  # Reset the integral term
        last_line_position = lv  # Update the last line position
    
    steringValue = P + I + D  # Calculate the steering value
    I += steringValue * Ki  # Update the integral term
    if steringValue > 100: steringValue = 100  # Limit the steering value to 100
    if steringValue < -100: steringValue = -100  # Limit the steering value to -100
    steering_drive.on(steringValue, -speed)  # Update the steering and speed

steering_drive.off(brake=True)  # Stop the motors when the touch sensor is pressed

print(time.time() - startTime)  # Print the elapsed time
