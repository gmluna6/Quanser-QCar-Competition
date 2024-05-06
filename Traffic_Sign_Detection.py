'''Traffic_Sign_Detection.py
This script holds the camera loop for object detection, when running if sign detected car affected
Update on the control_state.json file will happen when vehicle controlled from here
'''
#imports
import time
import cv2
import ultralytics.engine
import ultralytics.engine.results
from pal.products.qcar import QCarCameras

import ultralytics
from ultralytics import YOLO
import torch
from pal.products.qcar import QCar
import json
from filelock import Timeout, FileLock
import numpy as np

from hal.utilities.image_processing import ImageProcessing

#opening midel and initializing variables
model = YOLO('C:/Users/user/Downloads/George_Spring24_QCAR/Research_Resources/Research_Resources/src/SDCS/skills_activities/vehicle_control/best.pt')
# Initial Setup
runTime = 120.0  # Same runtime as the Vehicle Control Script
sampleRate = 200

#Time beforeit detects again:
det_counter_red = 0
stop_sign_det = False
red_sign_det = False
frames_since_last_stop_stop = 100  # Counter to track frames since last stop, initially set to allow stopping immediately
frames_since_last_stop_light = 100  # Counter to track frames since last stop, initially set to allow stopping immediately
            

cameras = QCarCameras(
    enableBack=False,
    enableFront=True,
    enableLeft=False,
    enableRight=False,
)

def update_control_state(allow_prediction):
    lock = FileLock("control_state.json.lock")
    with lock:
        with open('control_state.json', 'w') as file:
            json.dump({'allow_prediction': allow_prediction}, file)

#loop for camera interfacing and live detection
# with QCarRealSense(mode='RGB, Depth') as myCam:
with cameras:
    t0 = time.time()
    while time.time() - t0 < runTime:
        # start_time = time.time()
        # Read RGB frame
        cameras.readAll()
        for i, c in enumerate(cameras.csi):
            if c is not None:
                frame = c.imageData

        croppedRGB_White_Line = frame[300:800, 150:450] # original camera size [262:352, 0:410]
        croppedRGB_Traffic = frame[100:200, 200:700] # original camera size [262:352, 0:410]
        hsvBuf = cv2.cvtColor(croppedRGB_White_Line, cv2.COLOR_BGR2HSV)
        white_binary = ImageProcessing.binary_thresholding(frame=hsvBuf,
                                                     		lowerBounds=np.array([0,0,200]),
                                                       		upperBounds=np.array([180,50,255]))
        contours, _ = cv2.findContours(white_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        has_something = len(contours) >= 1
        print(has_something)
        #         cv2.imshow(('CSI '+str(i)+':'), c.imageData)        
        # cv2.waitKey(1)
        # Object detection
        results = model(frame)
        det_cls = results[0].boxes.cls
        print(results[0].boxes.xywh)
        # if torch.equal(det_cls, torch.tensor([0.0])) or torch.equal(det_cls, torch.tensor([2.0])):
        # Stop the car if stop sign (cls=0) or red sign (cls=2) detected and 1000 frames have passed
        tens = results[0].boxes.xywh
        nump = tens.numpy()
        # x_val = int(nump[0,0])
        # y_val = int(nump[0,3])
        if torch.equal(det_cls, torch.tensor([2.0])):
            if frames_since_last_stop_stop  >= 100:  # Ensure 200 frames have passed since last stop
                if nump[0,1] > 50:
                    with QCar(frequency=200) as myCar:
                        with QCar(frequency=sampleRate) as myCar:
                            t1 = time.time()
                            # update_control_state(False)
                            while time.time() - t1 < 3: #Stop 3 seconds
                                myCar.write(0,0,None)
                            frames_since_last_stop_stop  = 0  # Reset frame counter after stopping
                            # update_control_state(True)

                        # t2 = time.time()
                        # while time.time() - t2 < 0.3:
                        #     myCar.write(0.1,0, LEDs)

        frames_since_last_stop_stop += 1  # Increment frame counter regardless of detection
        
        # if torch.equal(det_cls, torch.tensor([0.0])) and has_something:
        if torch.equal(det_cls, torch.tensor([0.0])) and int(nump[0,3]) > 23:
            det_counter_red +=1
                # Ensure 200 frames have passed since last stop
            if nump[0,1] > 100 and nump[0,0] > 250 :#and det_counter_red >= 10:
                with QCar(frequency=200) as myCar:
                    with QCar(frequency=sampleRate) as myCar:
                        t1 = time.time()
                        det_counter_red = 0
                        # update_control_state(False)
                        while time.time() - t1 < 2.5: #Stop 3 seconds
                            myCar.write(0,0,None)
                        # update_control_state(True)

                        # t2 = time.time()
                        # while time.time() - t2 < 0.3:
                        #     myCar.write(0.1,0, LEDs)
