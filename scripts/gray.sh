#! /usr/bin/env bash

# This script displays the video stream from all four cameras.

# Front Right (Close to hinge)
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! "video/x-raw(memory:NVMM),height=(int)540,width=(int)960,format=(string)NV12, framerate=60/1" ! nvvidconv ! "video/x-raw, format=(string)GRAY8" ! videoconvert ! xvimagesink -ev &
