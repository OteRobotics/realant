# Licensed under MIT licence, see LICENSE for details.
# Copyright Ote Robotics Ltd. 2020
# Copyright Aalto University 2021

import cv2
import zmq
import datetime
import json

from video_stream import VideoStream
from pose_estimation import RealAntPoseEstimator


def main():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.PUB)
    sock.connect("tcp://127.0.0.1:3002")

    logfile = open('showarucoboard.py_log_%s.json' % datetime.datetime.utcnow().strftime("%Y_%m_%d_%H_%M_%S"), 'w')

    vs = VideoStream(src=1).start()
    pe = RealAntPoseEstimator()

    last_time = None
    while True:
        # Read latest image
        img, utcnowms = vs.read()

        # Compute fps
        fps = 0
        if last_time != None:
            if last_time == utcnowms:
                continue
            fps = 1000 / (utcnowms - last_time)
        last_time = utcnowms

        # Estimate pose
        d, img = pe.get_pose(img, utcnowms)

        if d is not None:
            sock.send_json(d)
            d["sent"] = True

            print("fps %5.1f " % fps, end='')
            print("- %d dist %2.1f x %1.3fm y %1.3fm z %1.3fm roll %3.0f pitch %3.0f yaw %3.0f" % tuple([d[x] for x in ["server_epoch_ms", "dist","x","y","z","roll","pitch","yaw"]]),end='')
            if "xvel" in d:
                print(" xvel%6.3f %6.3f y%6.3f z%6.3f" % (d["xvel"], d["xvel_raw"], d["yvel"], d["zvel"]))
            else:
                print()

            logfile.write(json.dumps(d)+'\n')

        cv2.imshow('ant', img)
        if cv2.waitKey(1) == 27: 
            break  # esc to quit

    vs.stop()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
