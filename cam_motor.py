import cv2
import time
from ar_markers import detect_markers
import numpy as np
import RPi.GPIO as GPIO
from http.client import HTTPConnection

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
    ord("1"): (-1, 1, -1, 1), ord("2"): (1, -1, 1, -1),
    ord("q"): (0, 1, 1, 0), ord("w"): (1, 1, 1, 1), ord("e"): (1, 0, 0, 1),
    ord("a"): (-1, 1, 1, -1), ord("s"): (0, 0, 0, 0), ord("d"): (1, -1, -1, 1),
    ord("z"): (-1, 0, 0, -1), ord("x"): (-1, -1, -1, -1), ord("c"): (0, -1, -1, 0),
}


print('Press "esc" to quit')
# cam init
capture = cv2.VideoCapture(0)


# Information of cam calibration 
DIM=(320, 240)
K=np.array([[221.67707080340955, 0.0, 155.52536738055687], [0.0, 223.2977065361501, 155.07273676797982], [0.0, 0.0, 1.0]])
D=np.array([[-0.0984827193194895], [0.09959390944563078], [-0.2826711250733146], [0.23938836491127113]])

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

        if action == ord("s"):
            print('stop')
        elif action == ord("a"):
            print('left')
        elif action == ord("d"):
            print('right')
        elif action == ord("1"):
            print('1')
    except:
        pass

def main():
    motor_key = 115
    while True:
        frame_captured, frame = capture.read()
        frame = cv2.flip(frame,-1)
        frame = cv2.resize(frame,(320,240))
        #undistort
        undistorted_image = undistort(frame)

        #ar_marker
        markers = detect_markers(undistorted_image)
        for marker in markers:
            marker.highlite_marker(undistorted_image)

        cv2.imshow('Frame', undistorted_image)
        key = cv2.waitKey(1) & 0xFF

        print(key)

        if key == 27:
            break
        elif key == ord("\t"):
            captured(undistorted_image)
        elif key in MOTOR_SPEEDS:
            motor_key = key
            motor(motor_key)
        elif key == 255:
            motor(motor_key)


        # When everything done, release the capture
    capture.release()
    cv2.destroyAllWindows()


main()