import RPi.GPIO as GPIO
import time
import math
import numpy as np

sensorRight = 18 
sensorLeft = 21 

##Left motor
motorLeft_1 = 12
motorLeft_2 = 20

##Right motor
motorRight_1 = 19
motorRight_2 = 26


GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(sensorLeft,GPIO.IN)
GPIO.setup(sensorRight,GPIO.IN)

GPIO.setup(motorLeft_1,GPIO.OUT)
GPIO.setup(motorLeft_2,GPIO.OUT)
GPIO.setup(motorRight_1,GPIO.OUT)
GPIO.setup(motorRight_2,GPIO.OUT)

pwmLeftForward = GPIO.PWM(motorLeft_1,50)
pwmLeftForward.start(0)
pwmRightForward = GPIO.PWM(motorRight_1,50)
pwmRightForward.start(1)

dutyCycleLeft=30
pwmLeftForward.ChangeDutyCycle(dutyCycleLeft) ##Left_motor
GPIO.output(motorLeft_2,False)
time.sleep(0.1)

dutyCycleRight=30
pwmRightForward.ChangeDutyCycle(dutyCycleRight) ##Right_motor
GPIO.output(motorRight_2,False)
time.sleep(0.1)

countLeft = 0
rpmLeft = 0
deltaLeft = 0
startLeft = 0
endLeft = 0
rpmLeftDes=70
previousErrorLeft=0
totalErrorLeft=0

countRight = 0
rpmRight = 0
deltaRight = 0
startRight = 0
endRight = 0
previousErrorRight=0
totalErrorRight=0
rpmRightDes = 50

sampleRight=100
sampleLeft=100



def get_rpmLeft(d):
    global countLeft
    global rpmLeft
    global startLeft
    global endLeft
    global previousErrorLeft
    global totalErrorLeft

    if not countLeft:
        startLeft = time.time()
        countLeft = countLeft +1

    else:
        countLeft = countLeft+1

    if countLeft == sampleLeft:
        endLeft = time.time()
        deltaLeft = endLeft - startLeft
        deltaLeft = deltaLeft/60
        rpmLeft = (sampleLeft/deltaLeft)/341.4
        print("rpmLeft Left =", rpmLeft)

        controller_LeftOut=controller_Left(rpmLeft,previousErrorLeft,totalErrorLeft)
        dutyCycleLeft=controller_LeftOut[0]
        previousErrorLeft=controller_LeftOut[1]
        totalErrorLeft=totalErrorLeft+previousErrorLeft
        
        pwmLeftForward.ChangeDutyCycle(dutyCycleLeft) ##Left_motor
        GPIO.output(motorLeft_2,False)
        countLeft=0
    return rpmLeft

def get_rpm_right(c):
    global countRight
    global rpmRight
    global startRight
    global endRight
    global previousErrorRight
    global totalErrorRight

    if not countRight:
        startRight = time.time()
        countRight = countRight +1

    else:
        countRight = countRight+1

    if countRight == sampleRight:
        endRight = time.time()
        deltaRight = endRight - startRight
        deltaRight = deltaRight/60
        rpmRight = (sampleRight/deltaRight)/341.4
        print("rpmRight Right =", rpmRight)

        
        controller_RightOut=controller_Right(rpmRight,previousErrorRight,totalErrorRight)
        dutyCycleRight=controller_RightOut[0]
        previousErrorRight=controller_RightOut[1]
        totalErrorRight=totalErrorRight+previousErrorRight
   
        pwmRightForward.ChangeDutyCycle(dutyCycleRight) ##left_motor
        GPIO.output(motorRight_2,False)
        countRight=0
    return rpmRight

def controller_Left(rpmLeft,previousErrorLeft,totalErrorLeft):
    global dutyCycleLeft
    kp=0.25
    kd=0.05
    ki=0.2
    errorLeft=rpmLeftDes-rpmLeft

    dutyCycleLeft=kp*errorLeft+kd*(errorLeft-previousErrorLeft)+ki*totalErrorLeft
    if dutyCycleLeft>98:
        dutyCycleLeft=98
    elif dutyCycleLeft< 4:
        dutyCycleLeft= 4

    return dutyCycleLeft,errorLeft

def controller_Right(rpmRight,previousErrorRight,totalErrorRight):
    global dutyCycleRight
    kp=0.25
    kd=0.05
    ki=0.2
    error=rpmRightDes-rpmRight
    
    dutyCycleRight=kp*error+kd*(error-previousErrorRight)+ki*totalErrorRight
    if dutyCycleRight>98:
        dutyCycleRight=98
    elif dutyCycleRight<1:
        dutyCycleRight=1

    return dutyCycleRight,error

GPIO.add_event_detect(sensorLeft,GPIO.RISING, callback = get_rpmLeft)
GPIO.add_event_detect(sensorRight,GPIO.RISING, callback = get_rpm_right)

try:
    while True:
        time.sleep(0.1)

except KeyboardInterrupt:
    print("QUIT")
    GPIO.cleanup()
