#! /usr/bin/env python3

import sys
import numpy as np
import cv2
import tempfile
import yaml
import shutil
import os.path
import tkinter as tk
import tkinter.filedialog as fd
from collections import namedtuple
from PIL import Image, ImageTk

IMAGE_DISPLAY_SHAPE = (960, 540)

class Camera:
    def __init__(self, cam_id, image_size):
        self.image_size = image_size
        self.id = cam_id;
        self.cap = self._gs_cap(self.id)
        self.current_image = None

    def capture(self):
        return self.cap.read()

    def release(self):
        self.cap.release()

    def _gs_pipe(self, cam_id):
        return "nvarguscamerasrc sensor-id={} ! "\
               "video/x-raw(memory:NVMM),width=(int){},height=(int){},framerate=60/1 ! "\
               "nvvidconv ! video/x-raw, format=(string)I420 ! videoconvert ! "\
               "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True".format(cam_id, self.image_size[0], self.image_size[1])

    def _gs_cap(self, cam_id):
        cap = cv2.VideoCapture(self._gs_pipe(cam_id), cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print('Failed to open nav camera')
            exit(0)
        return cap

class GUI:
    def __init__(self, calibration_file):
        self.parse_calibration(calibration_file)
        self.init_rectification()
        self.tmpdir = tempfile.mkdtemp()
        self.saved_img_count = 0

        self.root = tk.Tk()
        self.root.title("Navigation Camera Calibration GUI")

        # Image panels
        self.left_cam = Camera(self.left_cam_id, self.image_size)
        self.left_panel = tk.Label(self.root)
        self.left_panel.grid(row=0, column=0, padx=10, pady=10)

        self.right_cam = Camera(self.right_cam_id, self.image_size)
        self.right_panel = tk.Label(self.root)
        self.right_panel.grid(row=0, column=1, padx=10, pady=10)

        # Destructor
        self.root.protocol('WM_DELETE_WINDOW', self.destructor)

    def parse_calibration(self, f):
        with open(f) as file:
            cal = yaml.load(file, Loader=yaml.FullLoader)
        self.K1 = np.array(cal["left"]["camera_matrix"]).reshape(3,3)
        self.K2 = np.array(cal["right"]["camera_matrix"]).reshape(3,3)
        self.D1 = np.array(cal["left"]["distortion_coefficients"]).reshape(1,5)
        self.D2 = np.array(cal["right"]["distortion_coefficients"]).reshape(1,5)
        self.R  = np.array(cal["R"]).reshape(3,3)
        self.T  = np.array(cal["T"]).reshape(3,1)
        self.E  = np.array(cal["E"]).reshape(3,3)
        self.F  = np.array(cal["F"]).reshape(3,3)

        self.left_cam_id = cal["left"]["gstreamer_id"]
        self.right_cam_id = cal["right"]["gstreamer_id"]

        self.image_width = cal["left"]["image_width"]
        self.image_height = cal["left"]["image_height"]
        self.image_size = (self.image_width, self.image_height)

        print("K1: ", self.K1)
        print("D1: ", self.D1)
        print("K2: ", self.K2)
        print("D2: ", self.D2)

    def run(self):
        self.video_loop()
        self.root.mainloop()

    def video_loop(self):
        # Helper to convert opencv images to tkinter format
        def cv2tk(img):
            if img is None:
                return None

            rgba_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)  # convert colors from BGR to RGBA
            pil_img = Image.fromarray(rgba_img)               # convert image to PIL
            imgtk = ImageTk.PhotoImage(image=pil_img)         # convert image to tkinter
            return imgtk

        l_ok, frame = self.left_cam.capture()
        if l_ok:
            left_frame = cv2.remap(frame, self.map1[0], self.map1[1], cv2.INTER_LINEAR)
            small_frame = cv2.resize(left_frame, IMAGE_DISPLAY_SHAPE)
            self.left_panel.frame = cv2tk(small_frame)
            self.left_panel.config(image=self.left_panel.frame)

        r_ok, frame = self.right_cam.capture()
        if r_ok:
            right_frame = cv2.remap(frame, self.map2[0], self.map2[1], cv2.INTER_LINEAR)
            small_frame = cv2.resize(right_frame, IMAGE_DISPLAY_SHAPE)
            self.right_panel.frame = cv2tk(small_frame)
            self.right_panel.config(image=self.right_panel.frame)

        self.root.after(30, self.video_loop)

    def destructor(self):
        self.left_cam.release()
        self.right_cam.release()
        self.root.quit()

    def init_rectification(self):
        R1, R2, P1, P2, Q, ROI_1, ROI_2 = cv2.stereoRectify(self.K1, self.D1, self.K2, self.D2, self.image_size, self.R, self.T, cv2.CALIB_ZERO_DISPARITY, self.image_size)
        print("R:\n", self.R)
        print("T:\n", self.T)
        print("Rl:\n", R1)
        print("Rr:\n", R2)
        print("Pl:\n", P1)
        print("Pr:\n", P2)
        print("Q:\n", Q)
        print("ROI_1:\n", ROI_1)
        print("ROI_2:\n", ROI_2)
        self.map1 = cv2.initUndistortRectifyMap(self.K1, self.D1, R1, P1, self.image_size, cv2.CV_16SC2)
        self.map2 = cv2.initUndistortRectifyMap(self.K2, self.D2, R2, P2, self.image_size, cv2.CV_16SC2)


if __name__=="__main__":
    gui = GUI(sys.argv[1])
    gui.run()
