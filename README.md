RealAnt
=======

The RealAnt robot platform from Ote Robotics is designed for real-world 
reinforcement learning research and development. It is a complete solution 
with a web camera based tracking system. It is aimed to be a low-cost starting
point for anybody interested in bringing reinforcement learning to practical
real robots.

![RealAnt-v1](/../master/stl/RealAnt-v1.jpeg?raw=true)

This repository provides 3D models and build instructions
for making your own RealAnt, firmware code for the robot microcontroller, and
Python code for the robot interface and ArUco tag based pose estimation.

If you want to get the robot ready assembled, you can buy them from
<https://shop.oterobotics.com>.

For an example implementation of reinforcement learning that learns
to stand, turn and walking, see <https://github.com/AaltoVision/realant-rl>.


# Getting Started

You can get to reinforcement learning in the real world with five steps:

1. Get or build your own RealAnt.
1. Calibrate and test the web camera for pose estimation.
1. Setup testing scene with adequate lighting.
1. Start Python Server scripts.
1. Start experiments.


## Get Your RealAnt

If you want to get the robot ready assembled, you can buy them from
<https://shop.oterobotics.com>.


To build your own, you need to obtain 8 Robotis Dynamixel AX-12A's and
one Robotis OpenCM9.04A board + accessory set. For building,
you need additionally some two-wire cable for power, a soldering iron,
side cutters, a Phillips screwdriver and thread-locking fluid.

The 3D model files are under `stl` folder. You need to print two
body torso plates and four leg assemblies.

For PrusaSlicer, use 20% gyroid infill, 0.2mm layer height, no top 
and bottom layers.

Install Arduino and setup Robotis OpenCM9.04 board support,
and then upload the OpenCM9.04 firmware from `ant11_cmd_dxl` folder.


## Pose Estimation Calibration and Testing

Print calibration chessboard from `markers` on
e.g. A4 and glue them to a flat surface (e.g. piece of cardboard). 
Print the calibration chessboard at 100% scale, this
produces 4cm chessboard pattern. (If you use a different scaling, change
`calibrate_camera.py` script's marker size accordingly.)
Clipping at the edges of the outermost squares doesn't matter,
as only the inner corners of the squares are used.

Capture various poses of the chessboard (15+ images)
with `capture.py` and then run `calibrate_camera.py`.
Move the position of the chessboard around, so that it would
cover the whole field of view of the camera. Tilting
the chessboard also can yield better calibration.
Make sure your chessboard poses are free from motion blur,
delete the ones that are blurry.

After this, run `calibrate_camera.py` to obtain `cam_calib.pkl` file.

```
# mkdir CAMERANAME
# cd CAMERANAME
# python ../capture.py
# python ../calibrate_camera.py 
```

After calibrating the camera, adjust `showaruco_board.py` such
that it uses the correct `cam_calib.pkl` file.

Print and attach reference and moving agent markers to 
floor and the ant, adjust their sizes in the `showaruco_board.py` script
and run the script to estimate distance. Make sure to use correct
camera calibration file.

```
# python showaruco_board.py
```


## Setup The Test Scene

Make sure you have adequate lighting so that there is no motion blur in
pose estimation during robot movement.

For example, 2x 50W LED floodlights rated at 3800 lumens 
mounted 1 m above the robot should yield around 3000 lux illuminance
at the robot level, which is enough for reinforcement learning purposes
with Logitech Brio 4K USB camera.


## Python Interface Server

For starting trials, start the following scripts in their own terminal windows:

```
# python antproxy.py         # zmq pub-sub communication proxy
# python ant_server.py       # server for serial communication to the physical ant
# python showaruco_board.py  # pose estimation
```

You might need to adjust `ant_server.py` for the correct serial port device.
It defaults to `/dev/ttyACM0` on Linux.

Test scripts:

```
# python ant_send_cmd.py     # send simple test movements to ant
```


## Reinforcement Learning

For an example implementation of reinforcement learning that learns
to stand, turn and walking, see https://github.com/AaltoVision/realant-rl.

Clone that repository, and then run the scripts:

```
# rollout_server.py             # random exploration & agent evaluation
# train_client.py --task walk   # train for walking
```


# Further Interface Implementation Details

The main components are

 * `antproxy.py` (the main pub-sub network proxy, this should work as-is, as it just receives and forwards packages as a message hub — just run this on the background),
 * `showaruco_board.py` (this sends orientation and location info) and
 * `ant_server.py` (this processes the s1...s8 servo commands, and sends joint position values and (optional) foot sensors).
 
The pub-sub network protocol is very simple.
Each message is a list of byte strings.
The beginning of the first string is the "topic" in ZMQ terms
(used with `zmq.SUBSCRIBE` to filter in messages, `""` receives everything):

 * `"cmd"` is a command that the `ant_server.py` receives. This takes a 
   multipart string argument that is forwarded to the firmware. 
   Currently supported commands are `"sN X sM Y ..."` where N,M = servo number
   in [1,8] and X,Y = angle setpoint in [224...800], 512 = middle,
   and `"reset"`, `"attach_servos"` and `"detach_servos"`.
 * `"{"` which is json data, which is used both for all measurements 
   (and recorded setpoints) from `ant_server.py` and also for 
   `showaruco_board.py` measurement packets.



# Copyright and License

Unless otherwise noted, all source code, documentation and data in this
repository is Copyright (c) 2020 Ote Robotics Ltd and is licensed under
MIT license.

See LICENSE for details.
