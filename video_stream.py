# Licensed under MIT licence, see LICENSE for details.
# Copyright Ote Robotics Ltd. 2020
# Copyright Aalto University 2021

import cv2
import threading


class VideoStream:
    def __init__(self, src=0, autofocus=False, auto_exposure=True):
        self.stream = cam = cv2.VideoCapture(src) # + cv2.CAP_V4L2)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        # change camera parameters to get a better resolution & fast shutter speed
        if cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280):
            print('Set frame width to 1280')
        if cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720):            
            print('Set frame height to 720')
        if cam.set(cv2.CAP_PROP_FPS, 60):
            print('Set FPS to 60')

        # Tweak camera focus and exposure for better tracking
        if autofocus:
            if cam.set(cv2.CAP_PROP_AUTOFOCUS, 1):
                print('Disabled autofocus')
        else:
            if cam.set(cv2.CAP_PROP_AUTOFOCUS, 0):
                print('Disabled autofocus')
            if cam.set(cv2.CAP_PROP_FOCUS, 0):
                print('Set focus to 0')

        if auto_exposure:
            if cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75):
                print('Enabled auto exposure')
        else:
            if cam.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25):
                print('Disabled auto exposure')
            if cam.set(cv2.CAP_PROP_EXPOSURE, -7):
                print('Set exposure to -7')

        '''
        # UVC Camera tweaks for Logitech Brio 4K, 
        # these can be devised e.g. with guvcview
        import subprocess
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c white_balance_temperature_auto=0",shell=True)
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_auto=1",shell=True)
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_absolute=100",shell=True)
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c exposure_auto_priority=0",shell=True)
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c power_line_frequency=1",shell=True)
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c focus_auto=0",shell=True)
        subprocess.check_call("v4l2-ctl -d /dev/video0 -c focus_absolute=0",shell=True)
        '''
     
        (self.grabbed, self.frame) = self.stream.read()
        self.pos = cam.get(cv2.CAP_PROP_POS_MSEC)
        self.fps = cam.get(cv2.CAP_PROP_FPS)

        self.stopped = False
        
    def start(self):
        threading.Thread(target=self.update, args=()).start()
        return self

    def update(self):
        last_time = None
        while True:
            if self.stopped:
                return
            (self.grabbed, self.frame) = self.stream.read()
            self.pos = self.stream.get(cv2.CAP_PROP_POS_MSEC)
            self.fps = self.stream.get(cv2.CAP_PROP_FPS)
            
            utcnowms = self.stream.get(cv2.CAP_PROP_POS_MSEC)

    def read(self):
        return self.frame, self.pos

    def stop(self):
        self.stopped = True