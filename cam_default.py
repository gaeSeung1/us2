import cv2

print('Press "q" to quit')
capture = cv2.VideoCapture(0)


while True:

    frame_captured, frame = capture.read()
    frame = cv2.resize(frame,(320,240))
    frame = cv2.flip(frame,-1)
    cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture

capture.release()
cv2.destroyAllWindows()
