#! /usr/bin/env bash

# This script displays the video stream from all four cameras.

# Back Left (Away from hinge)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),height=540,width=960, format=NV12, framerate=60/1" ! nvvidconv ! xvimagesink -ev & 

# Back Right (Away from hinge)
gst-launch-1.0 nvarguscamerasrc sensor-id=1 ! "video/x-raw(memory:NVMM),height=540,width=960, format=NV12, framerate=60/1" ! nvvidconv ! xvimagesink -ev &

# Front Left (Close to hinge)
gst-launch-1.0 nvarguscamerasrc sensor-id=2 ! "video/x-raw(memory:NVMM),height=540,width=960, format=NV12, framerate=60/1" ! nvvidconv ! xvimagesink -ev &

# Front Right (Close to hinge)
gst-launch-1.0 nvarguscamerasrc sensor-id=3 ! "video/x-raw(memory:NVMM),height=540,width=960, format=NV12, framerate=60/1" ! nvvidconv ! xvimagesink -ev &
