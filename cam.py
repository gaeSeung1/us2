import cv2
import time
from ar_markers import detect_markers
import numpy as np

print('Press "q" to quit')
# cam init
capture = cv2.VideoCapture(0)


# Information of cam calibration 
DIM=(640, 480)
K=np.array([[423.84678614672646, 0.0, 328.574274970577], [0.0, 426.2990638388311, 181.11011683414546], [0.0, 0.0, 1.0]])
D=np.array([[-0.041247028858065506], [-0.3207044136410278], [1.2676834972699325], [-1.7300690469126907]])

def captured(img):
    cv2.imwrite(time.strftime('%m%d%H%M%S')+'.jpg', img)

def undistort(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return img

while True:
    frame_captured, frame = capture.read()
    frame = cv2.flip(frame,-1)
    frame = cv2.resize(frame,(320,240))
    #undistort
    undistorted_image = frame#undistort(frame)

    #ar_marker
    markers = detect_markers(undistorted_image)
    for marker in markers:
        marker.highlite_marker(undistorted_image)

    cv2.imshow('Frame', undistorted_image)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord("\t"):
        captured(undistorted_image)

	# When everything done, release the capture
capture.release()
cv2.destroyAllWindows()
