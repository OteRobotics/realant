# Licensed under MIT licence, see LICENSE for details.
# Copyright Ote Robotics Ltd. 2020
# Copyright Aalto University 2021

import cv2
from cv2 import aruco
import numpy as np
import pickle
from collections import OrderedDict, deque


smbm_len = 3.6 # small board marker length, cm
bigbm_len = 3.4 # big board marker length, cm 
smbm_sep = smbm_len * 0.2  # small board marker separation length, cm
bigbm_sep = bigbm_len * 0.2 # big board marker separation length, cm

aruco_dict = aruco.getPredefinedDictionary( aruco.DICT_ARUCO_ORIGINAL )
arucoParams = aruco.DetectorParameters_create()

# Speed tweaks to get aruco detection as fast as possible.
arucoParams.minOtsuStdDev = 0.0
arucoParams.adaptiveThreshConstant = 60

calib = pickle.load(open("logitechbrio4k/cam_calib.pkl", "rb"))
camera_matrix = calib["mtx"]
dist_coeffs = calib["dist"]

# configure board layouts
board_small = aruco.GridBoard_create(2, 2, smbm_len, smbm_sep, aruco_dict, 305)
board_big = aruco.GridBoard_create(7, 5, bigbm_len, bigbm_sep, aruco_dict)


class RealAntPoseEstimator:
    def __init__(self):
        self.N = 5
        self.last_x = deque(maxlen=self.N)
        self.last_y = deque(maxlen=self.N)
        self.last_z = deque(maxlen=self.N)
        self.last_timestamps = deque(maxlen=self.N)
        self.last_x_unfiltered = deque(maxlen=3)
        self.last_pitch_unfiltered = deque(maxlen=3)
        self.last_roll_unfiltered = deque(maxlen=3)
        self.slow_z = 0

    def get_pose(self, img, timestamp):
        d, img = self.estimate_pose(img, timestamp)
        if d is not None:
            d = self.filter_pose(d)
        return d, img

    def estimate_pose_board(self, img_gray, corners, ids, rejectedImgPoints, board):
        nmarkers, rvec, tvec = estimatePoseBoard(corners, ids, board, camera_matrix, dist_coeffs, None, None, useExtrinsicGuess=False)
        if nmarkers > 0:
            corners2, ids2, _, _ = aruco.refineDetectedMarkers(img_gray, board, corners, ids, rejectedImgPoints, camera_matrix, dist_coeffs, parameters=arucoParams)
            nmarkers, rvec, tvec = estimatePoseBoard(corners2, ids2, board, camera_matrix, dist_coeffs, None, None, useExtrinsicGuess=False)
        return nmarkers, rvec, tvec

    def estimate_pose(self, img, timestamp):
        # Detect markers
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(img_gray, aruco_dict, parameters=arucoParams)
        if ids is None: # if aruco markers are not detected
            return None, img

        # Estimate robot and floor pose
        nmarkers_robot, rvec_robot, tvec_robot = self.estimate_pose_board(img_gray, corners, ids, rejectedImgPoints, board_small)
        nmarkers_floor, rvec_floor, tvec_floor = self.estimate_pose_board(img_gray, corners, ids, rejectedImgPoints, board_big)

        # Move small_board origin to slightly forward to robot body origin
        if nmarkers_robot > 0:
            small_board_center = (smbm_len + smbm_sep/2) # cm, see above
            tag_to_body_center_height = 0
            rot, _ = cv2.Rodrigues(rvec_robot)
            centering_vec = np.dot(rot, np.array([[small_board_center + 2, small_board_center, -tag_to_body_center_height]]).T)
            tvec_robot[0] += centering_vec[0]
            tvec_robot[1] += centering_vec[1]
            tvec_robot[2] += centering_vec[2]

        # Draw markers and pose axes on image
        img = aruco.drawDetectedMarkers(img, corners, ids, (0, 255, 0))
        if nmarkers_robot > 0:
            img = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec_robot, tvec_robot, 3.21 / 2)
        if nmarkers_floor > 0:
            img = aruco.drawAxis(img, camera_matrix, dist_coeffs, rvec_floor, tvec_floor, 3.21 / 2)

        if not (nmarkers_robot > 0 and nmarkers_floor > 0): # if robot and floor markers are not detected
            return None, img

        # Compute relative pose
        rvec, tvec = get_relative_position(rvec_robot, tvec_robot, rvec_floor, tvec_floor)

        a = 0.3
        self.slow_z = a * tvec[2] + (1-a) * self.slow_z

        d = OrderedDict()
        d["id"] = "external_tag_tracking_camera"
        d["server_epoch_ms"] = timestamp
        d["dist"] = float(np.linalg.norm(tvec))
        d["x"] = float(tvec[0]) / 100. # in m
        d["y"] = float(tvec[1]) / 100. # in m
        d["z"] = float(self.slow_z) / 100. # in m
        d["roll"] = float(rvec[0] / np.pi * 180) # in degrees
        d["pitch"] = float(rvec[1] / np.pi * 180) # in degrees
        d["yaw"] = float(rvec[2] / np.pi * 180) # in degrees
        d["sent"] = False

        linedist = 40
        text_style = dict(fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=linedist/30, color=(0, 0, 255), thickness=2)
        cv2.putText(img,
            "Ant distance from reference to ant: %2.1fcm" % d["dist"],
            (20, 40), **text_style)
        cv2.putText(img, 
            "Ant position: x: %2.1fcm y: %2.1fcm z: %2.1fcm" % (tvec[0], tvec[1], tvec[2]),
            (20, 40 + linedist), **text_style)
        cv2.putText(img, 
            "Ant orientation: roll: %2.1fdeg pitch: %2.1fdeg yaw: %2.1fdeg" % (d["roll"], d["pitch"], d["yaw"]),
            (20, 40 + linedist*2), **text_style)
        return d, img

    def filter_pose(self, d):
        if d["z"] < -0.05:
            print("negative z, less than -0.05m, ignoring")
            return

        self.last_x_unfiltered.append(d["x"])
        if len(self.last_x_unfiltered) > 2:
            xabsmaxdiff = np.abs(np.amax(np.diff(self.last_x_unfiltered)))
            if xabsmaxdiff > 0.15:
                print("more than 15cm jump in x, ignoring")
                return

        self.last_pitch_unfiltered.append(d["pitch"])
        if len(self.last_pitch_unfiltered) > 2:
            pitchabsmaxdiff = np.abs(np.amax(np.diff(self.last_pitch_unfiltered)))
            if pitchabsmaxdiff > 20:
                print("more than 20deg jump in pitch, ignoring")
                return

        self.last_roll_unfiltered.append(d["roll"])
        if len(self.last_roll_unfiltered) > 2:
            rollabsmaxdiff = np.abs(np.amax(np.diff(self.last_roll_unfiltered)))
            if rollabsmaxdiff > 20:
                print("more than 20deg jump in roll, ignoring")
                return

        self.last_x.append(d["x"])
        self.last_y.append(d["y"])
        self.last_z.append(d["z"])
        self.last_timestamps.append(d["server_epoch_ms"] * 1e-3) # in s

        if len(self.last_x) >= self.N:
            xvel = (self.last_x[-1] - self.last_x[-2]) / (self.last_timestamps[-1] - self.last_timestamps[-2])
            xvel_hd5 = holoborodko_diff(np.array(self.last_x), float(np.mean(np.diff(np.array(self.last_timestamps)))))

            d["xvel_raw"] = xvel
            d["xvel"] = d["xvel_hd5"] = xvel_hd5

            yvel = (self.last_y[-1] - self.last_y[-2]) / (self.last_timestamps[-1] - self.last_timestamps[-2])
            yvel_hd5 = holoborodko_diff(np.array(self.last_y), float(np.mean(np.diff(np.array(self.last_timestamps)))))

            d["yvel_raw"] = yvel
            d["yvel"] = d["yvel_hd5"] = yvel_hd5

            zvel = (self.last_z[-1] - self.last_z[-2]) / (self.last_timestamps[-1] - self.last_timestamps[-2])
            zvel_hd5 = holoborodko_diff(np.array(self.last_z), float(np.mean(np.diff(np.array(self.last_timestamps)))))

            d["zvel_raw"] = zvel
            d["zvel"] = d["zvel_hd5"] = zvel_hd5
            return d


def estimatePoseBoard(corners, ids, board, cameraMatrix, distCoeffs, rvec=None, tvec=None, useExtrinsicGuess=False):
    assert len(corners) == len(ids)
    objPoints, imgPoints = aruco.getBoardObjectAndImagePoints(board, corners, ids)
    if objPoints is None or len(objPoints) == 0:
        return 0, None, None

    assert len(imgPoints) == len(objPoints)
    _, rvec, tvec = cv2.solvePnP(objPoints, imgPoints, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_IPPE)
    return int(len(objPoints) / 4), rvec, tvec

def holoborodko_diff(x, dt, M=2):
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
