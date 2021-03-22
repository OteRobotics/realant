# Licensed under MIT licence, see LICENSE for details.
# Copyright Ote Robotics Ltd. 2020
# Copyright Aalto University 2021

import cv2
import datetime

now = datetime.datetime.now().strftime("%Y-%m-%d")
i = 0
past = None

cap = cv2.VideoCapture(0)

# these should be same as used in showaruco_board.py
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 60)

cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 0)
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)

while(True):
    ret, frame = cap.read()
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2BGRA)

    flag = False
    n = datetime.datetime.now()
    if past != None:
        print((n - past).total_seconds())
        if (n - past).total_seconds() > 3:
            flag = True
            past = n
    else:
        past = n

    cv2.imshow('frame', rgb)
    key = cv2.waitKey(1) & 0xFF

    if key == ord('c') or flag:
        fn = "cam_calib_%s_%02d.jpg" % (now, i)
        out = cv2.imwrite(fn, frame)
        print("wrote %s" % fn)
        i += 1
    elif key == ord('q'):
        break


cap.release()
cv2.destroyAllWindows()
