#! /usr/bin/env python3

import argparse
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

CameraCalibration = namedtuple('CameraCalibration',
        ['reprojection_error','intrinsics_matrix','distortion_coefficients','image_shape'])

StereoCalibration = namedtuple('StereoCalibration',
        ['reprojection_error','intrinsics_left','intrinsics_right','distortion_left','distortion_right', 'R', 'T', 'E', 'F','image_shape'])

def calibrate_mono(image_paths, pattern_shape, pattern_square_size):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # objp contains coordinates of the chessboard corners in meters
    objp = np.zeros((pattern_shape[0]*pattern_shape[1], 3), dtype=np.float32)
    objp[:,:2] = np.mgrid[0:pattern_shape[0], 0:pattern_shape[1]].T.reshape(-1,2)
    objp *= pattern_square_size

    objpoints = []  # 3d points in the world
    imgpoints = []  # 2d points in the image

    img_w = 0
    img_h = 0
    for p in image_paths:
        print("Processing: "+p)
        img = cv2.imread(p)
        img_w = img.shape[1]
        img_h = img.shape[0]
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_shape, None)

        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners2)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    print("RMS Error: "+str(ret))
    result = CameraCalibration(ret, mtx, dist, (img_w,img_h))
    return result

def calibrate_stereo(left_images, right_images, pattern_shape, pattern_square_size, left_calib, right_calib):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    objp = np.zeros((pattern_shape[0]*pattern_shape[1], 3), dtype=np.float32)
    objp[:,:2] = np.mgrid[0:pattern_shape[0], 0:pattern_shape[1]].T.reshape(-1,2)
    objp *= pattern_square_size

    objpoints = []
    left_imgpoints = []
    right_imgpoints = []

    for l,r in zip(left_images, right_images):
        print("Processing: "+l+", "+r)
        left_img = cv2.imread(l)
        right_img = cv2.imread(r)

        left_gray  = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        right_gray = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        left_ret, left_corners = cv2.findChessboardCorners(left_gray, pattern_shape, None)
        right_ret, right_corners = cv2.findChessboardCorners(right_gray, pattern_shape, None)

        if left_ret and right_ret:
            objpoints.append(objp)

            left_corners2 = cv2.cornerSubPix(left_gray, left_corners, (11,11), (-1,-1), criteria)
            left_imgpoints.append(left_corners2)

            right_corners2 = cv2.cornerSubPix(right_gray, right_corners, (11,11), (-1,-1), criteria)
            right_imgpoints.append(right_corners2)

    left_mtx   = left_calib.intrinsics_matrix
    left_dist  = left_calib.distortion_coefficients
    right_mtx  = right_calib.intrinsics_matrix
    right_dist = right_calib.distortion_coefficients

    err, m0, d0, m1, d1, R, T, E, F = cv2.stereoCalibrate(objpoints, left_imgpoints, right_imgpoints, left_mtx, left_dist, right_mtx, right_dist, left_gray.shape[::-1])
    print("RMS Error: "+str(err))
    result = StereoCalibration(err, m0, m1, d0, d1, R, T, E, F, left_calib.image_shape)
    return result

def save_stereo_calibration(filename, stereo_calib, left_cam_id, right_cam_id):
    left_dict = {}
    right_dict = {}
    stereo_dict = {}

    left_dict['camera_name']   = 'left';
    left_dict['gstreamer_id']   = left_cam_id;
    left_dict['camera_model']  = 'plumb_bob';
    left_dict['camera_matrix'] = stereo_calib.intrinsics_left.flatten().tolist()
    left_dict['distortion_coefficients'] = stereo_calib.distortion_left.flatten().tolist()
    left_dict['image_width'] = stereo_calib.image_shape[0]
    left_dict['image_height'] = stereo_calib.image_shape[1]

    right_dict['camera_name']   = 'right';
    right_dict['gstreamer_id']  = right_cam_id
    right_dict['camera_model']  = 'plumb_bob';
    right_dict['camera_matrix'] = stereo_calib.intrinsics_right.flatten().tolist()
    right_dict['distortion_coefficients'] = stereo_calib.distortion_right.flatten().tolist()
    right_dict['image_width'] = stereo_calib.image_shape[0]
    right_dict['image_height'] = stereo_calib.image_shape[1]

    stereo_dict['left']  = left_dict
    stereo_dict['right'] = right_dict
    stereo_dict['R'] = stereo_calib.R.flatten().tolist()
    stereo_dict['T'] = stereo_calib.T.flatten().tolist()
    #stereo_dict['E'] = stereo_calib.E.flatten().tolist()
    #stereo_dict['F'] = stereo_calib.F.flatten().tolist()

    with open(filename,'w') as file:
        file.write("# This file generated by nav_cal.py\n")
        file.write("# Jordan Ford\n")
        file.write("# Note: Matrices are in row-major order.\n")
        file.write("#\n\n")
        yaml.dump(stereo_dict, file, sort_keys=False)

class Camera:
    def __init__(self, cam_id, image_size):
        self.id = cam_id;
        self.cap = self._gs_cap(self.id)
        self.current_image = None
        self.image_size = image_size

    def capture(self):
        return self.cap.read()

    def release(self):
        self.cap.release()

    def _gs_pipe(self, cam_id):
        return "nvarguscamerasrc sensor-id={} ! "\
               "video/x-raw(memory:NVMM),width=(int){},height=(int){},framerate=60/1 ! "\
               "nvvidconv ! video/x-raw, format=(string)I420 ! videoconvert ! "\
               "video/x-raw, format=(string)BGR ! appsink max-buffers=1 drop=True".format(cam_id, image_size[0], image_size[1])

    def _gs_cap(self, cam_id):
        cap = cv2.VideoCapture(self._gs_pipe(cam_id), cv2.CAP_GSTREAMER)

        if not cap.isOpened():
            print('Failed to open nav camera')
            exit(0)
        return cap


class CalibrationMask:
    def __init__(self):
        self.mask = None;

    def _init_mask(self, shape):
        if self.mask is None:
            self.mask = np.zeros((shape[0], shape[1], 3), dtype=np.uint8)

    def update(self, frame, pattern_shape):
        if self.mask is None:
            self._init_mask(frame.shape)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_shape, None)
        if ret:
            cv2.drawChessboardCorners(self.mask, pattern_shape, corners, ret)
        return ret

    def apply(self, frame):
        if self.mask is None:
            self._init_mask(frame.shape)
        masked_frame = cv2.addWeighted(frame, 1, self.mask, 1, 0)
        return masked_frame;

class CalibrationGUI:
    def __init__(self, left_cam_id=0, right_cam_id=1, image_size=(0,0)):
        self.tmpdir = tempfile.mkdtemp()
        self.saved_img_count = 0

        self.image_size = image_size

        self.root = tk.Tk()
        self.root.title("Navigation Camera Calibration GUI")

        # Image panels
        self.left_cam_id = left_cam_id
        self.left_cam = Camera(left_cam_id, self.image_size)
        self.left_mask = CalibrationMask()
        self.left_panel = tk.Label(self.root)
        self.left_panel.grid(row=0, column=0, padx=10, pady=10)

        self.right_cam_id = right_cam_id
        self.right_cam = Camera(right_cam_id, self.image_size)
        self.right_mask = CalibrationMask()
        self.right_panel = tk.Label(self.root)
        self.right_panel.grid(row=0, column=1, padx=10, pady=10)

        # Buttons
        self.btn_panel = tk.Label(self.root)
        self.btn_panel.grid(row=0, column=2)

        self.pattern_width_label = tk.Label(self.btn_panel, text="Chessboard Width [Squares]")
        self.pattern_width_label.pack(fill="both", expand=True, padx=10, pady=0)
        self.pattern_width_spinbox = tk.Spinbox(self.btn_panel, from_=1, to=20, textvariable=tk.DoubleVar(value=7), justify=tk.RIGHT)
        self.pattern_width_spinbox.pack(fill="both", expand=True, padx=10, pady=10)

        self.pattern_height_label = tk.Label(self.btn_panel, text="Chessboard Height [Squares]")
        self.pattern_height_label.pack(fill="both", expand=True, padx=10, pady=0)
        self.pattern_height_spinbox = tk.Spinbox(self.btn_panel, from_=1, to=20, textvariable=tk.DoubleVar(value=9), justify=tk.RIGHT)
        self.pattern_height_spinbox.pack(fill="both", expand=True, padx=10, pady=10)

        self.pattern_square_size_label = tk.Label(self.btn_panel, text="Chessboard Square Size [mm]")
        self.pattern_square_size_label.pack(fill="both", expand=True, padx=10, pady=0)
        self.pattern_square_size_spinbox = tk.Spinbox(self.btn_panel, from_=1, to=1000, textvariable=tk.DoubleVar(value=40), justify=tk.RIGHT)
        self.pattern_square_size_spinbox.pack(fill="both", expand=True, padx=10, pady=10)

        self.capture_btn = tk.Button(self.btn_panel, text="Capture", command=self.capture)
        self.capture_btn.pack(fill="both", expand=True, padx=10, pady=10)

        self.cal_btn = tk.Button(self.btn_panel, text="Calibrate", command=self.calibrate)
        self.cal_btn.pack(fill="both", expand=True, padx=10, pady=10)

        # Capture when <Enter> is pressed.
        self.root.bind('<Return>', self.handle_enter)

        # Destructor
        self.root.protocol('WM_DELETE_WINDOW', self.destructor)

    def run(self):
        self.video_loop()
        self.root.mainloop()

    def handle_enter(self, event):
        self.capture()

    def capture(self):
        left_ok, left_frame = self.left_cam.capture()
        right_ok, right_frame = self.right_cam.capture()

        if left_ok and right_ok:
            pattern_shape = (int(self.pattern_width_spinbox.get())-1, int(self.pattern_height_spinbox.get())-1)

            small_left_frame = cv2.resize(left_frame, IMAGE_DISPLAY_SHAPE)
            found_corners_left = self.left_mask.update(small_left_frame, pattern_shape)

            small_right_frame = cv2.resize(right_frame, IMAGE_DISPLAY_SHAPE)
            found_corners_right = self.right_mask.update(small_right_frame, pattern_shape)

            if found_corners_left and found_corners_right:
                filename = os.path.join(self.tmpdir,'left_'+str(self.saved_img_count)+'.jpg')
                cv2.imwrite(filename, left_frame)

                filename = os.path.join(self.tmpdir, 'right_'+str(self.saved_img_count)+'.jpg')
                cv2.imwrite(filename, right_frame)

            self.saved_img_count += 1

    def calibrate(self):
        if self.saved_img_count < 4:
            print("Please capture more image pairs before attempting to calibrate.")
            return

        image_paths  = sorted([os.path.join(self.tmpdir,x) for x in os.listdir(self.tmpdir) if '.jpg' in x])
        left_images  = sorted([os.path.join(self.tmpdir,x) for x in image_paths if 'left_' in x])
        right_images = sorted([os.path.join(self.tmpdir,x) for x in image_paths if 'right_' in x])

        pattern_shape = (int(self.pattern_width_spinbox.get())-1, int(self.pattern_height_spinbox.get())-1)
        pattern_square_size = float(self.pattern_square_size_spinbox.get()) / 1000.0

        print("Calibrating left camera...")
        left_calib  = calibrate_mono(left_images, pattern_shape, pattern_square_size)

        print("Calibrating right camera...")
        right_calib = calibrate_mono(right_images, pattern_shape, pattern_square_size)

        print("Calibrating stereo pair...")
        stereo_calib = calibrate_stereo(left_images, right_images, pattern_shape, pattern_square_size, left_calib, right_calib)

        yaml_file = fd.asksaveasfilename(initialdir="./", title="Save Calibration As...", filetypes=[("yaml","*.yml"),("yaml","*.yaml")])
        save_stereo_calibration(yaml_file, stereo_calib, self.left_cam_id, self.right_cam_id)
        self.destructor()

    def video_loop(self):

        # Helper to convert opencv images to tkinter format
        def cv2tk(img):
            if img is None:
                return None

            rgba_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)  # convert colors from BGR to RGBA
            pil_img = Image.fromarray(rgba_img)               # convert image to PIL
            imgtk = ImageTk.PhotoImage(image=pil_img)         # convert image to tkinter
            return imgtk

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
    parser = argparse.ArgumentParser(description='Calibrate Rover Nav Cameras.')
    parser.add_argument('--left_id', nargs=1, default=[0], type=int, help='The cam_id for the left camera')
    parser.add_argument('--right_id', nargs=1, default=[1], type=int, help='The cam_id for the right camera')
    parser.add_argument('--scale', nargs=1, default=[4], type=int, help='Divide raw image width and height by scale')
    args = parser.parse_args()

    cam_ids = (args.left_id[0], args.right_id[0])
    print("Calibrating Camera {} and Camera {}.".format(cam_ids[0], cam_ids[1]))

    image_size = (int(3840/args.scale[0]), int(2160/args.scale[0]))
    gui = CalibrationGUI(cam_ids[0], cam_ids[1], image_size)
    gui.run()
