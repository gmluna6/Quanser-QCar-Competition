'''
This File utlizes the RGB IntelRealSense for object detection and response to traffic signals and signs 
'''
#imports
import time
import cv2
import ultralytics.engine
import ultralytics.engine.results
from pal.products.qcar import QCarRealSense
import ultralytics
from ultralytics import YOLO
import tensorflow as tf
import torch
from pal.products.qcar import QCar
#opening midel and initializing variables
model = YOLO('F:\\objdetqcarcomp\\best.pt')
# Initial Setup
runTime = 120.0  # Same runtime as the Vehicle Control Script
qcar = QCar(readMode=1, frequency=500)
#loop for camera interfacing and live detection
with QCarRealSense(mode='RGB, Depth') as myCam:
    t0 = time.time()
    while time.time() - t0 < runTime:
        start_time = time.time()
        # Read RGB frame
        myCam.read_RGB()
        frame = myCam.imageBufferRGB
        # Object detection
        # model.predict(frame, show = True, save_dir=False, conf=.2)  # Lowering confidence threshold 
        results = model.predict(frame, show=True, conf=.2)
        class_names = results[0].names
        boxes = results[0].boxes
        det_cls = results[0].boxes.cls
        if torch.equal(det_cls, torch.tensor([0.0])) or torch.equal(det_cls, torch.tensor([2.0])):
            qcar.write(0,0)    
        # print(det_cls) for debugging data type of det_cls
        cv2.waitKey(10)

# Optimized code - needs tweaking
# import time
# import cv2
# from pal.products.qcar import QCarRealSense, QCar
# from ultralytics import YOLO
# import torch
# # Load YOLO model with smaller size and lower confidence threshold
# model = YOLO('F:\\objdetqcarcomp\\best.pt')

# # Initial Setup
# runTime = 120.0  # Same runtime as the Vehicle Control Script
# qcar = QCar(readMode=1, frequency=500)

# with QCarRealSense(mode='RGB, Depth') as myCam:
#     t0 = time.time()
#     # Preload the model outside the loop
#     while time.time() - t0 < runTime:
#         start_time = time.time()
#         # Read RGB frame
#         myCam.read_RGB()
#         frame = myCam.imageBufferRGB
#         # Object detection
#         results = model.predict(frame, show=True, conf=0.2)
#         det_cls = results[0].boxes.cls
#         if torch.equal(det_cls, torch.tensor([0.0])) or torch.equal(det_cls, torch.tensor([2.0])):  # Check if the detected class is 0 or 2
#             qcar.write(0, 0)
#         else:
#             pass
#         cv2.waitKey(1)
