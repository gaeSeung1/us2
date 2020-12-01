import cv2
import time
from ar_markers import detect_markers
import numpy as np
import RPi.GPIO as GPIO
import mpu9250
from queue import Queue
import threading
import math

#motor init
GPIO.setmode(GPIO.BCM)
motor11=22
motor12=27
motor21=11
motor22=9
motor31=15
motor32=14
motor41=24
motor42=25
pwm1=17
pwm2=10
pwm3=18
pwm4=23

GPIO.setup(motor11,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor12,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor21,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor22,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor31,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor32,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor41,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(motor42,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(pwm1,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(pwm2,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(pwm3,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(pwm4,GPIO.OUT,initial=GPIO.LOW)

p1=GPIO.PWM(pwm1,100)
p2=GPIO.PWM(pwm2,100)
p3=GPIO.PWM(pwm3,100)
p4=GPIO.PWM(pwm4,100)

p1.start(0)
p2.start(0)
p3.start(0)
p4.start(0)

#motor action init
speed = 80
HALF=0
MOTOR_SPEEDS = {
    "1": (-1, 1, -1, 1), "2": (1, -1, 1, -1),
    "q": (0, 1, 1, 0), "w": (1, 1, 1, 1), "e": (1, 0, 0, 1),
    "a": (-1, 1, 1, -1), "s": (0, 0, 0, 0), "d": (1, -1, -1, 1),
    "z": (-1, 0, 0, -1), "x": (-1, -1, -1, -1), "c": (0, -1, -1, 0),
}

print('Press "esc" to quit')

# ultrasonic init
GPIO_TRIGGER = 14
GPIO_ECHO    = 15
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
GPIO.setup(GPIO_ECHO,GPIO.IN)
GPIO.output(GPIO_TRIGGER, False)

# MPU init
# Set up class
gyro = 250      # 250, 500, 1000, 2000 [deg/s]
acc = 2         # 2, 4, 7, 16 [g]
tau = 0.98
mpu = mpu9250.MPU(gyro, acc, tau)
# Set up sensor and calibrate gyro with N points
mpu.setUp()
mpu.calibrateGyro(500)

# cam init
capture = cv2.VideoCapture(0)
capture.set(3,320)
capture.set(4,240)

# Information of cam calibration 
DIM=(640, 480)
K=np.array([[423.84678614672646, 0.0, 328.574274970577], [0.0, 426.2990638388311, 181.11011683414546], [0.0, 0.0, 1.0]])
D=np.array([[-0.041247028858065506], [-0.3207044136410278], [1.2676834972699325], [-1.7300690469126907]])

# mobilenet, AR Marker dictionary
dic = {"bike":0, "car":0, "cat":0, "person":0, "train":0, 114:0}


def captured(img):
    cv2.imwrite(time.strftime('%m%d%H%M%S')+'.jpg', img)

def undistort(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return img

def motor(action):
    try:
        pw1 = speed * MOTOR_SPEEDS[action][0]
        pw2 = speed * MOTOR_SPEEDS[action][1]
        pw3 = speed * MOTOR_SPEEDS[action][2]
        pw4 = speed * MOTOR_SPEEDS[action][3]

        if pw1>0:
            GPIO.output(motor11,GPIO.HIGH)
            GPIO.output(motor12,GPIO.LOW)
        elif pw1<0:
            GPIO.output(motor11,GPIO.LOW)
            GPIO.output(motor12,GPIO.HIGH)
        if pw2>0:
            GPIO.output(motor21,GPIO.HIGH)
            GPIO.output(motor22,GPIO.LOW)
        elif pw2<0:
            GPIO.output(motor21,GPIO.LOW)
            GPIO.output(motor22,GPIO.HIGH)
        if pw3>0:
            GPIO.output(motor31,GPIO.HIGH)
            GPIO.output(motor32,GPIO.LOW)
        elif pw3<0:
            GPIO.output(motor31,GPIO.LOW)
            GPIO.output(motor32,GPIO.HIGH)
        if pw4>0:
            GPIO.output(motor41,GPIO.HIGH)
            GPIO.output(motor42,GPIO.LOW)
        elif pw4<0:
            GPIO.output(motor41,GPIO.LOW)
            GPIO.output(motor42,GPIO.HIGH)
            
        p1.ChangeDutyCycle(abs(pw1))
        p2.ChangeDutyCycle(abs(pw2))
        p3.ChangeDutyCycle(abs(pw3))
        p4.ChangeDutyCycle(abs(pw4))        

        if action == "s":
            print('stop')
        elif action == "a":
            print('left')
        elif action == "d":
            print('right')
        elif action == "q":
            print('forward-left')
        elif action == "w":
            print('forward')
        elif action == "e":
            print('forward-right')
        elif action == "z":
            print('backward-left')
        elif action == "x":
            print('backward')
        elif action == "c":
            print('backward-right')
        elif action == "1":
            print('spin left')                                                                                   
        elif action == "2":
            print('spin right')
    except:
        pass

def ultrasonic():
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()
    timeOut = start

    while GPIO.input(GPIO_ECHO)==0:
        start = time.time()
        if time.time()-timeOut > 0.012:
            return -1

    while GPIO.input(GPIO_ECHO)==1:
        if time.time()-start > 0.012:
            return -1
        stop = time.time()

    elapsed = stop-start
    distance = (elapsed * 34300)/2

    return distance


def main():
    while True:
        #mpu
        motorDeg = mpu.compFilter()

        #ultrasonic
        ultra = ultrasonic()
            
        #Threading
        qdata, evt = q.get()
        evt.set()
        q.task_done()

        try:
            #qdata in {"bike":0, "car":0, "cat":0, "person":0, "train":0, 114:0}
            #count # of detection
            if qdata in dic:
                dic[qdata] = dic[qdata] + 1

            #dic list
            dic_keys = list(dic.keys())
            dic_values = list(dic.values())
            max_value = max(dic_values)

            #obj detection
            obj = dic_keys[dic_values.index(max_value)]

            #10th detection
            if max_value >= 10:
                if obj == "bike":
                    #two turns
                    motorDegBefore = motorDeg
                    while abs(motorDeg - motorDegBefore) < 2 * 360:
                        motor('1')
                        motorDeg = mpu.compFilter()

                elif obj == "car":
                    #square
                    motor('a')
                    time.sleep(3)
                    motor('w')
                    time.sleep(3)
                    motor('d')
                    time.sleep(3)
                    motor('x')
                    time.sleep(3)

                elif obj == 114:
                    #triangle
                    motor('q')
                    time.sleep(3)
                    motor('z')
                    time.sleep(3)
                    motor('d')
                    time.sleep(3*math.sqrt(2))

                elif obj == "person":
                    #headbutt
                    checkTimeBefore = int(time.strftime('%S'))
                    while ultrasonic() > 5:
                        motor('w')
                        checkTime = int(time.strftime('%S'))
                    #come back
                    motor('x')
                    time.sleep(checkTime - checkTimeBefore)

                elif obj == "train":
                    #line
                    motor('a')
                    time.sleep(3)
                    motor('d')
                    time.sleep(3)
                
                else:
                    #stop
                    motor('s')

                #dic re-init
                dic = {"bike":0, "car":0, "cat":0, "person":0, "train":0, 114:0}
                dic_keys = list(dic.keys())
                dic_values = list(dic.values())
                max_value = max(dic_values)

        except:
            print('no qdata')
