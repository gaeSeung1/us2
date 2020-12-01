#Import the neccesary libraries
import numpy as np
import argparse
import cv2 
from queue import Queue
import threading
import ar_markers


# Information of cam calibration 
DIM=(320, 240)
K=np.array([[221.67707080340955, 0.0, 155.52536738055687], [0.0, 223.2977065361501, 155.07273676797982], [0.0, 0.0, 1.0]])
D=np.array([[-0.0984827193194895], [0.09959390944563078], [-0.2826711250733146], [0.23938836491127113]])

def undistort(img):
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    return img

def main():
        
    # construct the argument parse 
    parser = argparse.ArgumentParser(
        description='Script to run MobileNet-SSD object detection network ')
    parser.add_argument("--video", help="path to video file. If empty, camera's stream will be used")
    parser.add_argument("--prototxt", default="MobileNetSSD_deploy.prototxt",
                                    help='Path to text network file: '
                                        'MobileNetSSD_deploy.prototxt for Caffe model or '
                                        )
    parser.add_argument("--weights", default="MobileNetSSD_deploy.caffemodel",
                                    help='Path to weights: '
                                        'MobileNetSSD_deploy.caffemodel for Caffe model or '
                                        )
    parser.add_argument("--thr", default=0.2, type=float, help="confidence threshold to filter out weak detections")
    args = parser.parse_args()

    # Labels of Network.
    classNames = { 0: 'background',
        1: 'aeroplane', 2: 'bicycle', 3: 'bird', 4: 'boat',
        5: 'bottle', 6: 'bus', 7: 'car', 8: 'cat', 9: 'chair',
        10: 'cow', 11: 'diningtable', 12: 'dog', 13: 'horse',
        14: 'motorbike', 15: 'person', 16: 'pottedplant',
        17: 'sheep', 18: 'sofa', 19: 'train', 20: 'tvmonitor' }

    # Open video file or capture device. 
    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(0)

    #Load the Caffe model 
    net = cv2.dnn.readNetFromCaffe(args.prototxt, args.weights)

    while True:
        #switch init
        switch = 0

        # Capture frame-by-frame
        ret, frame = cap.read()
        size = (120,90)
        frame = cv2.flip(frame,-1)
        frame_resized = cv2.resize(frame,size) # resize frame for prediction
        frame = undistort(frame)
        # MobileNet requires fixed dimensions for input image(s)
        # so we have to ensure that it is resized to 300x300 pixels.
        # set a scale factor to image because network the objects has differents size. 
        # We perform a mean subtraction (127.5, 127.5, 127.5) to normalize the input;
        # after executing this command our "blob" now has the shape:
        # (1, 3, 300, 300)
        blob = cv2.dnn.blobFromImage(frame_resized, 0.007843, size, (127.5, 127.5, 127.5), False)
        #Set to network the input blob 
        net.setInput(blob)
        #Prediction of network
        detections = net.forward()

        #Size of frame resize (300x300)
        cols = frame_resized.shape[1] 
        rows = frame_resized.shape[0]

        #For get the class and location of object detected, 
        # There is a fix index for class, location and confidence
        # value in @detections array .
        for i in range(detections.shape[2]):
            confidence = detections[0, 0, i, 2] #Confidence of prediction 
            if confidence > args.thr: # Filter prediction 
                class_id = int(detections[0, 0, i, 1]) # Class label

                # Object location 
                xLeftBottom = int(detections[0, 0, i, 3] * cols) 
                yLeftBottom = int(detections[0, 0, i, 4] * rows)
                xRightTop   = int(detections[0, 0, i, 5] * cols)
                yRightTop   = int(detections[0, 0, i, 6] * rows)

                # Factor for scale to original size of frame
                heightFactor = frame.shape[0]/size[0] 
                widthFactor = frame.shape[1]/size[1]
                # Scale object detection to frame
                xLeftBottom = int(widthFactor * xLeftBottom) 
                yLeftBottom = int(heightFactor * yLeftBottom)
                xRightTop   = int(widthFactor * xRightTop)
                yRightTop   = int(heightFactor * yRightTop)
                # Draw location of object  
                cv2.rectangle(frame, (xLeftBottom, yLeftBottom), (xRightTop, yRightTop),
                            (0, 255, 0))

                # Draw label and confidence of prediction in frame resized
                if class_id in classNames:
                    label = classNames[class_id] + ": " + str(confidence)
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

                    yLeftBottom = max(yLeftBottom, labelSize[1])
                    cv2.rectangle(frame, (xLeftBottom, yLeftBottom - labelSize[1]),
                                        (xLeftBottom + labelSize[0], yLeftBottom + baseLine),
                                        (255, 255, 255), cv2.FILLED)
                    cv2.putText(frame, label, (xLeftBottom, yLeftBottom),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0))

                    print(label) #print class and confidence
                    switch = 1

        #AR marker
        try:
            markers = ar_markers.detect_markers(undistorted_image)
            for marker in markers:             
                marker.highlite_marker(undistorted_image)
            if marker.id > 0:
                switch = 2
        except:
            pass

        #Threading                
        evt = threading.Event()
        #mobilenet
        if switch = 1:
            qdata = label
        elif switch = 2:
            qdata = marker.id
        q.put((qdata, evt))

        #imshow
        cv2.namedWindow("frame", cv2.WINDOW_NORMAL)
        cv2.imshow("frame", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


q = Queue()
thread_one = threading.Thread(target=main, args=(q,))
thread_two = threading.Thread(target=cam_motor_dog.main, args=(q,))
thread_two.daemon = True

thread_one.start()
thread_two.start()

q.join()