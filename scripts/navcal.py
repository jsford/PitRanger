#! /usr/bin/env python3

import numpy as np
import cv2
import tempfile
import shutil
import os.path
import tkinter as tk
from PIL import Image, ImageTk

#IMAGE_DISPLAY_SHAPE = (960, 540)
IMAGE_DISPLAY_SHAPE = (1440, 810)

def gs_pipe(cam_id):
    return "nvarguscamerasrc sensor-id={} ! video/x-raw(memory:NVMM), framerate=60/1 ! nvvidconv ! video/x-raw, format=(string)I420 ! videoconvert ! video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True".format(cam_id)

def gs_cap(cam_id):
    cap = cv2.VideoCapture(gs_pipe(cam_id), cv2.CAP_GSTREAMER)

    if not cap.isOpened():
        print('Failed to open nav camera')
        exit(0)
    return cap

def cv2tk(img):
    if img is None:
        return None

    rgba_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)  # convert colors from BGR to RGBA
    pil_img = Image.fromarray(rgba_img)               # convert image to PIL
    imgtk = ImageTk.PhotoImage(image=pil_img)         # convert image to tkinter
    return imgtk

class Camera:
    def __init__(self, cam_id):
        self.id = cam_id;
        self.cap = gs_cap(self.id)
        self.current_image = None

    def capture(self):
        return self.cap.read()

    def release(self):
        self.cap.release()

class CalibrationMask:
    def __init__(self):
        self.mask = None;

    def _init_mask(self, shape):
        if self.mask is None:
            self.mask = np.zeros((shape[0], shape[1], 3), dtype=np.uint8)

    def update(self, frame):
        if self.mask is None:
            self._init_mask(frame.shape)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, (8,6), None)
        if ret:
            print("Found the chess board!")
            cv2.drawChessboardCorners(self.mask, (8,6), corners, ret)
        return ret

    def apply(self, frame):
        if self.mask is None:
            self._init_mask(frame.shape)
        masked_frame = cv2.addWeighted(frame, 1, self.mask, 1, 0)
        return masked_frame;

class CalibrationGUI:
    def __init__(self, left_cam_id=0, right_cam_id=1):
        self.tmpdir = tempfile.mkdtemp()
        self.saved_img_count = 0

        self.root = tk.Tk()
        self.root.title("Navigation Camera Calibration GUI")

        # Image panels
        self.left_cam = Camera(left_cam_id)
        self.left_mask = CalibrationMask()
        self.left_panel = tk.Label(self.root)
        self.left_panel.grid(row=0, column=0, padx=10, pady=10)

        self.right_cam = Camera(right_cam_id)
        self.right_mask = CalibrationMask()
        self.right_panel = tk.Label(self.root)
        self.right_panel.grid(row=0, column=1, padx=10, pady=10)

        # Buttons
        self.btn_panel = tk.Label(self.root)
        self.btn_panel.grid(row=0, column=2)

        self.pattern_width_spinbox = tk.Spinbox(self.btn_panel, from_=1, to=20)
        self.pattern_width_spinbox.pack(fill="both", expand=True, padx=10, pady=10)

        self.capture_btn = tk.Button(self.btn_panel, text="Capture", command=self.capture)
        self.capture_btn.pack(fill="both", expand=True, padx=10, pady=10)

        self.cal_btn = tk.Button(self.btn_panel, text="Calibrate", command=self.calibrate)
        self.cal_btn.pack(fill="both", expand=True, padx=10, pady=10)

        # Destructor
        self.root.protocol('WM_DELETE_WINDOW', self.destructor)

    def run(self):
        self.video_loop()
        self.root.mainloop()

    def capture(self):
        left_ok, left_frame = self.left_cam.capture()
        right_ok, right_frame = self.right_cam.capture()

        if left_ok and right_ok:
            small_left_frame = cv2.resize(left_frame, IMAGE_DISPLAY_SHAPE)
            found_corners_left = self.left_mask.update(small_left_frame)

            small_right_frame = cv2.resize(right_frame, IMAGE_DISPLAY_SHAPE)
            found_corners_right = self.right_mask.update(small_right_frame)

            if found_corners_left and found_corners_right:
                filename = os.path.join(self.tmpdir,'left_'+str(self.saved_img_count)+'.jpg')
                cv2.imwrite(filename, left_frame)

                filename = os.path.join(self.tmpdir, 'right_'+str(self.saved_img_count)+'.jpg')
                cv2.imwrite(filename, right_frame)

            self.saved_img_count += 1

    def calibrate(self):
        print("CALIBRATE")
        print(os.listdir(self.tmpdir))

    def video_loop(self):
        ok, frame = self.left_cam.capture()
        if ok:
            small_frame = cv2.resize(frame, IMAGE_DISPLAY_SHAPE)
            masked_frame = cv2tk( self.left_mask.apply(small_frame) )
            self.left_panel.masked_frame = masked_frame
            self.left_panel.config(image=masked_frame)

        ok, frame = self.right_cam.capture()
        if ok:
            small_frame = cv2.resize(frame, IMAGE_DISPLAY_SHAPE)
            masked_frame = cv2tk( self.right_mask.apply(small_frame) )
            self.right_panel.masked_frame = masked_frame
            self.right_panel.config(image=masked_frame)

        self.root.after(30, self.video_loop)

    def destructor(self):
        self.left_cam.release()
        self.right_cam.release()
        self.root.quit()
        shutil.rmtree(self.tmpdir)

if __name__=="__main__":
    gui = CalibrationGUI(2,3)
    gui.run()
