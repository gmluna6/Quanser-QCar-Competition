'''hardware_test_intelrealsense.py

This example demonstrates how to read and display depth & RGB image data
from the Intel Realsense camera.
'''
import time
import cv2
from pal.products.qcar import QCarRealSense
from ultralytics import YOLO
import numpy as np
# from QCAR_Competition_Controls import SpeedController
# Load YOLO model
model = YOLO('F:\\objdetqcarcomp\\best.pt')

# Initial Setup
runTime = 120.0 # same runtime as the Vehicle Control Script

with QCarRealSense(mode='RGB, Depth') as myCam:
    t0 = time.time()
    while time.time() - t0 < runTime:
        myCam.read_RGB()
        frame = myCam.imageBufferRGB

        # Object detection
        # results = model.predict(frame, show=True, conf=.25)
        results = model.predict(frame, conf=.25)
        # if model.predict(frame) == '1 Red_light' or '1 Stop_sign':
            # SpeedController.stop_car()
        # # Draw bounding boxes and labels
        # for result in results:
        #     xmin, ymin, xmax, ymax, conf, cls = result
        #     label = model.names[int(cls)]
        #     color = (0, 255, 0)  # green color for bounding boxes
        #     cv2.rectangle(frame, (int(xmin), int(ymin)), (int(xmax), int(ymax)), color, 2)
        #     cv2.putText(frame, label, (int(xmin), int(ymin) - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        # cv2.imshow('Object Detection', frame)
        cv2.waitKey(100)
# import time
# import cv2
# from pal.products.qcar import QCarRealSense
# from ultralytics import YOLO
# model = YOLO('F:\\objdetqcarcomp\\best.pt')
# # Initial Setup
# runTime = 120.0 # seconds

# with QCarRealSense(mode='RGB, Depth') as myCam:
#     t0 = time.time()
#     while time.time() - t0 < runTime:
#         myCam.read_RGB()
#         cv2.imshow('My RGB', myCam.imageBufferRGB)

#         # myCam.read_depth()
#         # cv2.imshow('My Depth', myCam.imageBufferDepthPX)

#         cv2.waitKey(100)