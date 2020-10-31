import cv2
from cv2 import aruco
import pickle
import numpy as np
import zmq
import datetime
from collections import OrderedDict
import json
import threading

smbm_len = 3.6 # small board marker length, cm
bigbm_len = 3.4 # big board marker length, cm 
smbm_sep = smbm_len * 0.2  # small board marker separation length, cm
bigbm_sep = bigbm_len * 0.2 # big board marker separation length, cm


ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.connect("tcp://127.0.0.1:3002")

aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_ARUCO_ORIGINAL )
arucoParams = aruco.DetectorParameters_create()

# Speed tweaks to get aruco detection as fast as possible.
arucoParams.minOtsuStdDev = 0.0
arucoParams.adaptiveThreshWinSizeMin = 20
arucoParams.adaptiveThreshWinSizeMax = 25
arucoParams.adaptiveThreshWinSizeStep = 5
arucoParams.adaptiveThreshConstant = 60
#arucoParams.cornerRefinementWinSize = 3


calib = pickle.load(open("logitechbrio4k/cam_calib.pkl", "rb"))
camera_matrix = calib["mtx"]
dist_coeffs = calib["dist"]

# configure board layouts
board_small = aruco.GridBoard_create(2, 2, smbm_len, smbm_sep, aruco_dict, 305)
board_big = aruco.GridBoard_create(7, 5, bigbm_len, bigbm_sep, aruco_dict)

last_time = None
last_x = []
last_y = []
last_z = []
last_timestamps = []
last_x_unfiltered = []
last_pitch_unfiltered = []

logfile = open('showarucoboard.py_log_%s.json' % datetime.datetime.utcnow(), 'w')

class VideoStream:
    def __init__(self, src=0):
        # Linux specific initialization
        self.stream = cam = cv2.VideoCapture(src + cv2.CAP_V4L2)
        cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

        # change camera parameters to get a better resolution & fast shutter speed
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        cam.set(cv2.CAP_PROP_FPS, 60)

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
            #fps = 0
            #if last_time != None:
            #    fps = 1000/(utcnowms - last_time)
            #last_time = utcnowms
            #print("fps %2.1f " % fps)


    def read(self):
        return self.frame, self.pos, self.fps

    def stop(self):
        self.stopped = True

def show_webcam(mirror=False):
    global last_x, last_y, last_z, last_x_unfiltered, last_pitch_unfiltered, last_time, last_timestamps


    vs = VideoStream(src=0).start()

    slow_z = 0
    
    while True:
        img,cappos,capfps = vs.read()
        utcnowms = cappos #int((datetime.datetime.utcnow() - datetime.datetime(1970, 1, 1)).total_seconds() * 1000)
        fps = 0
        if last_time != None:
            if last_time == utcnowms:
                continue
            fps = 1000/(utcnowms - last_time)

        last_time = utcnowms

        #(h, w) = img.shape[:2]
        #print(h,w)
        #center = (w/2,h/2)
        #M = cv2.getRotationMatrix2D(center, 180, 1)
        #img = cv2.warpAffine(img, M, (w, h))

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)

        if ids is not None: # if aruco marker detected
            #rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # For a single marker
            img = aruco.drawDetectedMarkers(img, corners, ids, (0,255,0))

            v, rvec, tvec = aruco.estimatePoseBoard(corners, ids, board_small, camera_matrix, dist_coeffs, None, None, useExtrinsicGuess=False)
            #print("board small", v, "rvec", rvec, "tvec", tvec)

            if v > 0:
                corners2, ids2, rejected2, recovered2 = aruco.refineDetectedMarkers(gray, board_small, corners, ids, rejectedImgPoints, camera_matrix, dist_coeffs, parameters=arucoParams)
                v, rvec, tvec = aruco.estimatePoseBoard(corners2, ids2, board_small, camera_matrix, dist_coeffs, None, None, useExtrinsicGuess=False)
                #img = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 3.21 / 2)


                # move small_board origin to slightly forward to robot body origin
                small_board_center = (smbm_len + smbm_sep/2) # cm, see above
                tag_to_body_center_height = 0 #2 + 0.5 # cm
                rot,_ = cv2.Rodrigues(rvec)
                centering_vec = np.dot(rot, np.array([[small_board_center + 2, small_board_center, -tag_to_body_center_height]]).T)
                tvec[0] += centering_vec[0]
                tvec[1] += centering_vec[1]
                tvec[2] += centering_vec[2]
                img = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec, tvec, 3.21 / 2)

            v2, rvec2, tvec2 = aruco.estimatePoseBoard(corners, ids, board_big, camera_matrix, dist_coeffs, None, None, useExtrinsicGuess=False)
            #print("board big", v2, rvec2, tvec2)
            if v2 > 0:
                corners2, ids2, rejected2, recovered2 = aruco.refineDetectedMarkers(gray,  board_big, corners, ids, rejectedImgPoints, camera_matrix, dist_coeffs, parameters=arucoParams)
                v2, rvec2, tvec2 = aruco.estimatePoseBoard(corners2, ids2, board_big, camera_matrix, dist_coeffs, None, None, useExtrinsicGuess=False)
                img = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec2, tvec2, 3.21 / 2)  

            t = None # 16
            m = None # 684, 305

            if False:
                for i,e in enumerate(ids):
                    #print(e)
                    if e == 684:
                        img = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec[i], tvec[i], 4.8 / 2)  
                        t = [rvec[i], tvec[i]]

                    if e == 305:
                        m = [rvec[i], tvec[i]]

            if v > 0 and v2 > 0:
                # both the floor board (v2) and agent board (v) have been detected
                rv, tv = get_relative_position(rvec, tvec, rvec2, tvec2)
                #print("tv", tv)
                #print("rv", rv)
                #print("dist", np.linalg.norm(tv))
                dist = np.linalg.norm(tv)

                if True:
                    linedist = 40
                    cv2.putText(img, 
                    "Ant distance from reference to ant: %2.1fcm" % dist, 
                    (20,40), 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=linedist/30, color=(0,0,255), thickness=2)
                    cv2.putText(img, 
                    "Ant position: x: %2.1fcm y: %2.1fcm z: %2.1fcm" % (tv[0], tv[1], tv[2]), 
                    (20,40+linedist), 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=linedist/30, color=(0,0,255), thickness=2)
                    cv2.putText(img, 
                    "Ant orientation: roll: %2.1fdeg pitch: %2.1fdeg yaw: %2.1fdeg" % (rv[0]/np.pi*180, rv[1]/np.pi*180, rv[2]/np.pi*180), 
                    (20,40+linedist*2), 
                    fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=linedist/30, color=(0,0,255), thickness=2)

                a = 0.3
                slow_z = a*tv[2] + (1-a)*slow_z

                d = OrderedDict()
                d["id"] = "external_tag_tracking_camera"
                d["server_epoch_ms"] = utcnowms
                d["dist"] = float(dist)
                d["x"] = float(tv[0]) / 100. # in m
                d["y"] = float(tv[1]) / 100. # in m
                d["z"] = float(slow_z) / 100. # in m
                d["roll"] = float(rv[0]/np.pi*180)
                d["pitch"] = float(rv[1]/np.pi*180)
                d["yaw"] = float(rv[2]/np.pi*180)
                d["sent"] = False


                # filtering
                

                last_x_unfiltered.append(d["x"])
                last_x_unfiltered = last_x_unfiltered[-3:]

                if len(last_x_unfiltered) > 2:
                    xabsmaxdiff = np.abs(np.amax(np.diff(last_x_unfiltered)))
                else:
                    xabsmaxdiff = 0


                last_pitch_unfiltered.append(d["pitch"])
                last_pitch_unfiltered = last_pitch_unfiltered[-3:]

                if len(last_pitch_unfiltered) > 2:
                    pitchabsmaxdiff = np.abs(np.amax(np.diff(last_pitch_unfiltered)))
                else:
                    pitchabsmaxdiff = 0

                if d["z"] < -0.05:
                    print("negative z, less than -0.05m, ignoring")
                elif xabsmaxdiff > 0.15:
                    print("more than 15cm jump in x, ignoring")
                elif pitchabsmaxdiff > 20:
                    print("more than 20deg jump in pitch, ignoring")
                else:
                    N = 5
                    last_x.append(d["x"])
                    last_x = last_x[-N:]
                    last_y.append(d["y"])
                    last_y = last_y[-N:]
                    last_z.append(d["z"])
                    last_z = last_z[-N:]

                    last_timestamps.append(d["server_epoch_ms"] * 1e-3) # in s
                    last_timestamps = last_timestamps[-N:]

                    if len(last_x) >= N:
                        xvel = (last_x[-1] - last_x[-2]) / (last_timestamps[-1] - last_timestamps[-2])
                        xvel_hd5 = holoborodko_diff(np.array(last_x), float(np.mean(np.diff(np.array(last_timestamps)))))

                        d["xvel_raw"] = xvel
                        d["xvel_hd5"] = xvel_hd5
                        d["xvel"] = d["xvel_hd5"]

                        yvel = (last_y[-1] - last_y[-2]) / (last_timestamps[-1] - last_timestamps[-2])
                        yvel_hd5 = holoborodko_diff(np.array(last_y), float(np.mean(np.diff(np.array(last_timestamps)))))

                        d["yvel_raw"] = yvel
                        d["yvel_hd5"] = yvel_hd5
                        d["yvel"] = d["yvel_hd5"]

                        zvel = (last_z[-1] - last_z[-2]) / (last_timestamps[-1] - last_timestamps[-2])
                        zvel_hd5 = holoborodko_diff(np.array(last_z), float(np.mean(np.diff(np.array(last_timestamps)))))

                        d["zvel_raw"] = zvel
                        d["zvel_hd5"] = zvel_hd5
                        d["zvel"] = d["zvel_hd5"]

                        sock.send_json(d)
                    d["sent"] = True


                print("fps %5.1f " % fps, end='')
                print("- %d dist %2.1f x %1.3fm y %1.3fm z %1.3fm roll %3.0f pitch %3.0f yaw %3.0f" % tuple([d[x] for x in ["server_epoch_ms", "dist","x","y","z","roll","pitch","yaw"]]),end='')
                if "xvel" in d:
                    print(" xvel%6.3f %6.3f y%6.3f z%6.3f" % (d["xvel"], d["xvel_raw"], d["yvel"], d["zvel"]))
                else:
                    print()

                logfile.write(json.dumps(d)+'\n')
                

        if mirror: 
            img = cv2.flip(img, 1)

        if True:
            cv2.imshow('ant', img)
            if cv2.waitKey(1) == 27: 
                break  # esc to quit
    cv2.destroyAllWindows()


def main():
    show_webcam(mirror=False)

def holoborodko_diff(x,dt,M=2):
    """
    Implement a smooth noise-robust differentiator.

    For details, see http://www.holoborodko.com/pavel/numerical-methods/numerical-derivative/smooth-low-noise-differentiators/
    """
    assert M == 2, "other filter lengths not implemented"
    dxdt = (2*(x[-2] - x[-4]) + x[-1] - x[-5]) / (8*dt)
    return dxdt


# Next two functions are based on https://github.com/aliyasineser/GraduationProjectII/blob/master/RelativePositionTest.py
# Copyright (c) 2018 Ali Yasin Eser, under MIT license

def invert_perspective(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    invTvec = np.dot(-np.matrix(R).T, np.matrix(tvec))
    invRvec, _ = cv2.Rodrigues(np.matrix(R).T)
    return invRvec, invTvec


def get_relative_position(rvec1, tvec1, rvec2, tvec2):
    rvec1, tvec1 = rvec1.reshape((3, 1)), tvec1.reshape((3, 1))
    rvec2, tvec2 = rvec2.reshape((3, 1)), tvec2.reshape((3, 1))

    # Inverse the second marker, the right one in the image
    invRvec, invTvec = invert_perspective(rvec2, tvec2)

    #orgRvec, orgTvec = invert_perspective(invRvec, invTvec)
    # print("rvec: ", rvec2, "tvec: ", tvec2, "\n and \n", orgRvec, orgTvec)

    info = cv2.composeRT(rvec1, tvec1, invRvec, invTvec)
    composedRvec, composedTvec = info[0], info[1]

    composedRvec = composedRvec.reshape((3, 1))
    composedTvec = composedTvec.reshape((3, 1))
    return composedRvec, composedTvec


if __name__ == '__main__':
    main()
