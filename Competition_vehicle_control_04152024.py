# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

"""
vehicle_control.py

Skills acivity code for vehicle control lab guide.
Students will implement a vehicle speed and steering controller.
Please review Lab Guide - vehicle control PDF
"""
import os
import signal
import numpy as np
from threading import Thread
import time
import cv2
import pyqtgraph as pg

from pal.products.qcar import QCar, QCarGPS, IS_PHYSICAL_QCAR
from pal.utilities.scope import MultiScope
from pal.utilities.math import wrap_to_pi
from hal.products.qcar import QCarEKF
from hal.products.mats import SDCSRoadMap
import pal.resources.images as images

# Setup competition imports 

import random
import sys
import time
import math
import struct
import cv2
import random

# environment objects

from qvl.qlabs import QuanserInteractiveLabs
from qvl.qcar import QLabsQCar
from qvl.free_camera import QLabsFreeCamera
from qvl.real_time import QLabsRealTime
from qvl.basic_shape import QLabsBasicShape
from qvl.system import QLabsSystem
from qvl.walls import QLabsWalls
from qvl.flooring import QLabsFlooring
from qvl.stop_sign import QLabsStopSign
from qvl.crosswalk import QLabsCrosswalk
import pal.resources.rtmodels as rtmodels

# traffic light imports 

from quanser.communications import Stream
from qvl.qlabs import QuanserInteractiveLabs
from qvl.traffic_light import QLabsTrafficLight

# qcar camera imports

from pal.products.qcar import QCar,QCarCameras
from pal.utilities.math import *
from pal.utilities.gamepad import LogitechF710
from hal.utilities.image_processing import ImageProcessing

# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 120
startDelay = 2
controllerUpdateRate = 500

# camera timing parameters 
sampleRate     = 30.0
sampleTime     = 1/sampleRate
simulationTime = tf

# Additional parameters
counter = 0
imageWidth = 640
imageHeight = 480
imageBuffer360 = np.zeros((imageHeight + 40, 4*imageWidth + 120, 3),
                          dtype=np.uint8) # 20 px padding between pieces

# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = 1.05
K_p = 0.3
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True #False
K_stanley = 1.5
nodeSequence = [2, 20, 10, 2]#[0, 20, 0]

#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
# Additional camera parameters
counter 	= 0
imageWidth  = 820
imageHeight = 410
cameraID 	= '3'

myCam = QCarCameras(frameWidth=imageWidth,
		    		frameHeight=imageHeight,
					frameRate=sampleRate,
					enableFront= True )

import threading

# def traffic_light_timing():
#     # os.system('cls')
#     qlabs = QuanserInteractiveLabs()
#     x_offset = 0.13
#     y_offset = 1.67
#     TrafficLight0 = QLabsTrafficLight(qlabs)
#     TrafficLight0.spawn_degrees([2.3 + x_offset, y_offset, 0], [0, 0, 0], scale=[.1, .1, .1], configuration=0, waitForConfirmation=True)
#     TrafficLight0.set_state(QLabsTrafficLight.STATE_GREEN)
#     TrafficLight1 = QLabsTrafficLight(qlabs)
#     TrafficLight1.spawn_degrees([-2.3 + x_offset, -1 + y_offset, 0], [0, 0, 180], scale=[.1, .1, .1], configuration=0, waitForConfirmation=True)
#     TrafficLight1.set_state(QLabsTrafficLight.STATE_RED)
#     i = 0
#     while True:
#         i += 1
#         print(i)

#         if i % 2 == 0:
#             TrafficLight0.set_state(QLabsTrafficLight.STATE_GREEN)
#             TrafficLight1.set_state(QLabsTrafficLight.STATE_RED)
#         else:
#             TrafficLight1.set_state(QLabsTrafficLight.STATE_GREEN)
#             TrafficLight0.set_state(QLabsTrafficLight.STATE_RED)

#         time.sleep(5) 
        
# Function to setup QLabs, Spawn in QCar, and run real time model
def setup(initialPosition, initialOrientation):
    # Try to connect to Qlabs

    os.system('cls')
    qlabs = QuanserInteractiveLabs()
    print("Connecting to QLabs...")
    try:
        qlabs.open("localhost")
        print("Connected to QLabs")
    except:
        print("Unable to connect to QLabs")
        quit()

    # Delete any previous QCar instances and stop any running spawn models
    qlabs.destroy_all_spawned_actors()
    QLabsRealTime().terminate_all_real_time_models()

    #Set the Workspace Title
    hSystem = QLabsSystem(qlabs)
    x = hSystem.set_title_string('ACC Self Driving Car Competition', waitForConfirmation=True)


    ### Flooring

    x_offset = 0.13
    y_offset = 1.67
    hFloor = QLabsFlooring(qlabs)
    #hFloor.spawn([0.199, -0.491, 0.005])
    hFloor.spawn_degrees([x_offset, y_offset, 0.001],rotation = [0, 0, -90])


    ### region: Walls
    hWall = QLabsWalls(qlabs)
    hWall.set_enable_dynamics(False)

    for y in range (5):
        hWall.spawn_degrees(location=[-2.4 + x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])

    for x in range (5):
        hWall.spawn_degrees(location=[-1.9+x + x_offset, 3.05+ y_offset, 0.001], rotation=[0, 0, 90])

    for y in range (6):
        hWall.spawn_degrees(location=[2.4+ x_offset, (-y*1.0)+2.55 + y_offset, 0.001], rotation=[0, 0, 0])

    for x in range (5):
        hWall.spawn_degrees(location=[-1.9+x+ x_offset, -3.05+ y_offset, 0.001], rotation=[0, 0, 90])

    hWall.spawn_degrees(location=[-2.03 + x_offset, -2.275+ y_offset, 0.001], rotation=[0, 0, 48])
    hWall.spawn_degrees(location=[-1.575+ x_offset, -2.7+ y_offset, 0.001], rotation=[0, 0, 48])


    # Spawn a QCar at the given initial pose
    car2 = QLabsQCar(qlabs)

    #-1.335+ x_offset, -2.5+ y_offset, 0.005
    #0, 0, -45
    car2.spawn_id(actorNumber=0, location=initialPosition, rotation=initialOrientation, scale=[.1, .1, .1], configuration=0, waitForConfirmation=True)
    basicshape2 = QLabsBasicShape(qlabs)
    basicshape2.spawn_id_and_parent_with_relative_transform(actorNumber=102, location=[1.15, 0, 1.8], rotation=[0, 0, 0], scale=[.65, .65, .1], configuration=basicshape2.SHAPE_SPHERE, parentClassID=car2.ID_QCAR, parentActorNumber=2, parentComponent=1,  waitForConfirmation=True)
    basicshape2.set_material_properties(color=[0.4,0,0], roughness=0.4, metallic=True, waitForConfirmation=True)

    camera1=QLabsFreeCamera(qlabs)
    camera1.spawn_degrees (location = [-0.426+ x_offset, -5.601+ y_offset, 4.823], rotation=[0, 41, 90])

    camera2=QLabsFreeCamera(qlabs)
    camera2.spawn_degrees (location = [-0.4+ x_offset, -4.562+ y_offset, 3.938], rotation=[0, 47, 90])

    camera3=QLabsFreeCamera(qlabs)
    camera3.spawn_degrees (location = [-0.36+ x_offset, -3.691+ y_offset, 2.652], rotation=[0, 47, 90])

    car2.possess()

    # stop signs
    myStopSign = QLabsStopSign(qlabs)
    myStopSign.spawn_degrees ([2.25 + x_offset, 1.5 + y_offset, 0.05], [0, 0, -90], [0.1, 0.1, 0.1], False)
    myStopSign.spawn_degrees ([-1.3 + x_offset, 2.9 + y_offset, 0.05], [0, 0, -15], [0.1, 0.1, 0.1], False)

    # traffic lights
    TrafficLight0 = QLabsTrafficLight(qlabs)
    TrafficLight0.spawn_degrees([2.3 + x_offset, y_offset, 0], [0, 0, 0], scale=[.1, .1, .1], configuration=0, waitForConfirmation=True)
    TrafficLight0.set_state(QLabsTrafficLight.STATE_GREEN)
    TrafficLight1 = QLabsTrafficLight(qlabs)
    TrafficLight1.spawn_degrees([-2.3 + x_offset, -1 + y_offset, 0], [0, 0, 180], scale=[.1, .1, .1], configuration=0, waitForConfirmation=True)
    TrafficLight1.set_state(QLabsTrafficLight.STATE_RED)
    
    
    # Spawning crosswalks
    myCrossWalk = QLabsCrosswalk(qlabs)
    myCrossWalk.spawn_degrees (location =[-2 + x_offset, -1.475 + y_offset, 0.01],
                rotation=[0,0,0], scale = [0.1,0.1,0.075],
                configuration = 0)

    mySpline = QLabsBasicShape(qlabs)
    mySpline.spawn_degrees ([2.05 + x_offset, -1.5 + y_offset, 0.01], [0, 0, 0], [0.27, 0.02, 0.001], False)
    mySpline.spawn_degrees ([-2.075 + x_offset, y_offset, 0.01], [0, 0, 0], [0.27, 0.02, 0.001], False)

    # Start spawn model
    QLabsRealTime().start_real_time_model(rtmodels.QCAR_STUDIO)
    return car2

#function to terminate the real time model running
def terminate():
    QLabsRealTime().terminate_real_time_model(rtmodels.QCAR_STUDIO)
    
#region : Initial setup

if enableSteeringControl:
    roadmap = SDCSRoadMap(leftHandTraffic=False)
    waypointSequence = roadmap.generate_path(nodeSequence)
    initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
else:
    initialPose = [0, 0, 0]

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    setup(
        initialPosition=[initialPose[0], initialPose[1], 0],
        initialOrientation=[0, 0, initialPose[2]]
    )

# Used to enable safe keyboard triggered shutdown
global KILL_THREAD
KILL_THREAD = False
def sig_handler(*args):
    global KILL_THREAD
    KILL_THREAD = True
signal.signal(signal.SIGINT, sig_handler)
#endregion

class SpeedController:

    def __init__(self, kp=0, ki=0):
        self.maxThrottle = .8

        self.kp = kp
        self.ki = ki

        self.ei = 0


    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
        '''
        e = v_ref - v
        self.ei += dt*e

        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
        '''
        return 0

# class SteeringController:

#     def __init__(self, waypoints, k=K_stanley, cyclic=True):
#         self.maxSteeringAngle = np.pi/5

#         self.wp = waypoints
#         self.N = len(waypoints[0, :])
#         self.wpi = 0

#         self.k = k
#         self.cyclic = cyclic

#         self.p_ref = (0, 0)
#         self.th_ref = 0

#         # Introduce a left bias for steering
#         self.left_bias = np.pi/45  # Adjust this value to control the bias

#     # ==============  SECTION B -  Steering Control  ====================
#     def update(self, p, th, speed):
#         wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
#         wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
#         v = wp_2 - wp_1
#         v_mag = np.linalg.norm(v)
#         try:
#             v_uv = v / v_mag
#         except ZeroDivisionError:
#             return 0

#         tangent = np.arctan2(v_uv[1], v_uv[0])

#         s = np.dot(p-wp_1, v_uv)

#         if s >= v_mag:
#             if  self.cyclic or self.wpi < self.N-2:
#                 self.wpi += 1

#         ep = wp_1 + v_uv*s
#         ct = ep - p
#         dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

#         ect = np.linalg.norm(ct) * np.sign(dir)
#         psi = wrap_to_pi(tangent-th)

#         self.p_ref = ep
#         self.th_ref = tangent

#         #return edited to contain left bias to fix issues with driving too far left 
#         return np.clip(
#             wrap_to_pi(psi + np.arctan2(self.k*ect, speed) + self.left_bias),
#             -self.maxSteeringAngle,
#             self.maxSteeringAngle)
#         '''
#         v = wp_2 - wp_1
#         v_mag = np.linalg.norm(v)
#         try:
#             v_uv = v / v_mag
#         except ZeroDivisionError:
#             return 0

#         tangent = np.arctan2(v_uv[1], v_uv[0])

#         s = np.dot(p-wp_1, v_uv)

#         if s >= v_mag:
#             if  self.cyclic or self.wpi < self.N-2:
#                 self.wpi += 1

#         ep = wp_1 + v_uv*s
#         ct = ep - p
#         dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

#         ect = np.linalg.norm(ct) * np.sign(dir)
#         psi = wrap_to_pi(tangent-th)

#         self.p_ref = ep
#         self.th_ref = tangent

#         return np.clip(
#             wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
#             -self.maxSteeringAngle,
#             self.maxSteeringAngle)
#         '''
#         return 0
# import numpy as np
# import skfuzzy as fuzz
# from skfuzzy import control as ctrl

# class SteeringController:
#     def __init__(self, waypoints, k=K_stanley, cyclic=True):
#         self.maxSteeringAngle = np.pi/6

#         self.wp = waypoints
#         self.N = len(waypoints[0, :])
#         self.wpi = 0

#         self.k = k
#         self.cyclic = cyclic

#         self.p_ref = (0, 0)
#         self.th_ref = 0

#         # Define fuzzy input variables
#         self.lateral_error = ctrl.Antecedent(np.arange(-5, 5, 0.1), 'lateral_error')
#         self.heading_error = ctrl.Antecedent(np.arange(-np.pi, np.pi, 0.1), 'heading_error')

#         # Define fuzzy output variable
#         self.steering_angle = ctrl.Consequent(np.arange(-self.maxSteeringAngle, self.maxSteeringAngle, 0.1), 'steering_angle')

#         # Define membership functions for input and output variables
#         self.lateral_error['negative'] = fuzz.trimf(self.lateral_error.universe, [-5, -5, 0])
#         self.lateral_error['zero'] = fuzz.trimf(self.lateral_error.universe, [-2, 0, 2])
#         self.lateral_error['positive'] = fuzz.trimf(self.lateral_error.universe, [0, 5, 5])

#         self.heading_error['negative'] = fuzz.trimf(self.heading_error.universe, [-np.pi, -np.pi, 0])
#         self.heading_error['zero'] = fuzz.trimf(self.heading_error.universe, [-np.pi/2, 0, np.pi/2])
#         self.heading_error['positive'] = fuzz.trimf(self.heading_error.universe, [0, np.pi, np.pi])

#         self.steering_angle['hard_left'] = fuzz.trimf(self.steering_angle.universe, [-self.maxSteeringAngle, -self.maxSteeringAngle, -0.5*self.maxSteeringAngle])
#         self.steering_angle['left'] = fuzz.trimf(self.steering_angle.universe, [-self.maxSteeringAngle, -0.5*self.maxSteeringAngle, 0])
#         self.steering_angle['straight'] = fuzz.trimf(self.steering_angle.universe, [-0.5*self.maxSteeringAngle, 0, 0.5*self.maxSteeringAngle])
#         self.steering_angle['right'] = fuzz.trimf(self.steering_angle.universe, [0, 0.5*self.maxSteeringAngle, self.maxSteeringAngle])
#         self.steering_angle['hard_right'] = fuzz.trimf(self.steering_angle.universe, [0.5*self.maxSteeringAngle, self.maxSteeringAngle, self.maxSteeringAngle])

#         # Define fuzzy rules
#         rule1 = ctrl.Rule(self.lateral_error['negative'] & self.heading_error['negative'], self.steering_angle['hard_left'])
#         rule2 = ctrl.Rule(self.lateral_error['negative'] & self.heading_error['zero'], self.steering_angle['left'])
#         rule3 = ctrl.Rule(self.lateral_error['negative'] & self.heading_error['positive'], self.steering_angle['straight'])
#         rule4 = ctrl.Rule(self.lateral_error['zero'] & self.heading_error['negative'], self.steering_angle['left'])
#         rule5 = ctrl.Rule(self.lateral_error['zero'] & self.heading_error['zero'], self.steering_angle['straight'])
#         rule6 = ctrl.Rule(self.lateral_error['zero'] & self.heading_error['positive'], self.steering_angle['right'])
#         rule7 = ctrl.Rule(self.lateral_error['positive'] & self.heading_error['negative'], self.steering_angle['straight'])
#         rule8 = ctrl.Rule(self.lateral_error['positive'] & self.heading_error['zero'], self.steering_angle['right'])
#         rule9 = ctrl.Rule(self.lateral_error['positive'] & self.heading_error['positive'], self.steering_angle['hard_right'])

#         # Create fuzzy control system
#         self.steering_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
#         self.steering_sim = ctrl.ControlSystemSimulation(self.steering_ctrl)

#     # ==============  SECTION B -  Steering Control  ====================
#     def update(self, p, th, speed):
#         wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
#         wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        
#         v = wp_2 - wp_1
#         v_mag = np.linalg.norm(v)
#         try:
#             v_uv = v / v_mag
#         except ZeroDivisionError:
#             return 0

#         tangent = np.arctan2(v_uv[1], v_uv[0])

#         s = np.dot(p-wp_1, v_uv)

#         if s >= v_mag:
#             if  self.cyclic or self.wpi < self.N-2:
#                 self.wpi += 1

#         ep = wp_1 + v_uv*s
#         ct = ep - p
#         dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)

#         ect = np.linalg.norm(ct) * np.sign(dir)
#         psi = wrap_to_pi(tangent-th)

#         self.p_ref = ep
#         self.th_ref = tangent

#         # Pass input values to the fuzzy controller
#         self.steering_sim.input['lateral_error'] = psi
#         self.steering_sim.input['heading_error'] = dir

#         # Compute the output
#         self.steering_sim.compute()

#         # Return the computed output
#         return self.steering_sim.output['steering_angle'] 
    
class SteeringController:

    def __init__(self, waypoints, k=K_stanley, cyclic=True, filter_coeff=0.25):
        self.maxSteeringAngle = np.pi / 5
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
        self.p_ref = (0, 0)
        self.th_ref = 0
        self.left_bias = np.pi / 45
        self.filter_coeff = filter_coeff
        self.filtered_steering_angle = 0

    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]
        wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        if v_mag == 0:
            return 0

        v_uv = v / v_mag
        tangent = np.arctan2(v_uv[1], v_uv[0])
        s = np.dot(p - wp_1, v_uv)

        if s >= v_mag:
            if self.cyclic or self.wpi < self.N - 2:
                self.wpi += 1

        ep = wp_1 + v_uv * s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent - th)

        steering_angle = psi + np.arctan2(self.k * ect, speed) + self.left_bias

        # Apply low-pass filter
        self.filtered_steering_angle += self.filter_coeff * (steering_angle - self.filtered_steering_angle)
        
        return np.clip(self.filtered_steering_angle, -self.maxSteeringAngle, self.maxSteeringAngle)   
    
def controlLoop():
    #region controlLoop setup
    global KILL_THREAD
    u = 0
    delta = 0
    # used to limit data sampling to 10hz
    countMax = controllerUpdateRate / 10
    count = 0
    #endregion

    #region Controller initialization
    speedController = SpeedController(
        kp=K_p,
        ki=K_i
    )
    if enableSteeringControl:
        steeringController = SteeringController(
            waypoints=waypointSequence,
            k=K_stanley
        )
    #endregion

    #region QCar interface setup
    qcar = QCar(readMode=1, frequency=controllerUpdateRate)
    if enableSteeringControl:
        ekf = QCarEKF(x_0=initialPose)
        gps = QCarGPS(initialPose=initialPose)
    else:
        gps = memoryview(b'')
    #endregion

    with qcar, gps:
        t0 = time.time()
        t=0
        while (t < tf+startDelay) and (not KILL_THREAD):
            #region : Loop timing update
            tp = t
            t = time.time() - t0
            dt = t-tp
            #endregion

            #region : Read from sensors and update state estimates
            qcar.read()
            if enableSteeringControl:
                if gps.readGPS():
                    y_gps = np.array([
                        gps.position[0],
                        gps.position[1],
                        gps.orientation[2]
                    ])
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        y_gps,
                        qcar.gyroscope[2],
                    )
                else:
                    ekf.update(
                        [qcar.motorTach, delta],
                        dt,
                        None,
                        qcar.gyroscope[2],
                    )

                x = ekf.x_hat[0,0]
                y = ekf.x_hat[1,0]
                th = ekf.x_hat[2,0]
                p = ( np.array([x, y])
                    + np.array([np.cos(th), np.sin(th)]) * 0.2)
            v = qcar.motorTach
            #endregion

            #region : Update controllers and write to car
            if t < startDelay:
                u = 0
                delta = 0
            else:
                #region : Speed controller update
                u = speedController.update(v, v_ref, dt)
                #endregion

                #region : Steering controller update
                if enableSteeringControl:
                    delta = steeringController.update(p, th, v)
                else:
                    delta = 0
                #endregion

            qcar.write(u, delta)
            #endregion

            #region : Update Scopes
            count += 1
            if count >= countMax and t > startDelay:
                t_plot = t - startDelay

                # Speed control scope
                speedScope.axes[0].sample(t_plot, [v, v_ref])
                speedScope.axes[1].sample(t_plot, [v_ref-v])
                speedScope.axes[2].sample(t_plot, [u])

                # Steering control scope
                if enableSteeringControl:
                    steeringScope.axes[4].sample(t_plot, [[p[0],p[1]]])

                    p[0] = ekf.x_hat[0,0]
                    p[1] = ekf.x_hat[1,0]

                    x_ref = steeringController.p_ref[0]
                    y_ref = steeringController.p_ref[1]
                    th_ref = steeringController.th_ref

                    x_ref = gps.position[0]
                    y_ref = gps.position[1]
                    th_ref = gps.orientation[2]

                    steeringScope.axes[0].sample(t_plot, [p[0], x_ref])
                    steeringScope.axes[1].sample(t_plot, [p[1], y_ref])
                    steeringScope.axes[2].sample(t_plot, [th, th_ref])
                    steeringScope.axes[3].sample(t_plot, [delta])


                    arrow.setPos(p[0], p[1])
                    arrow.setStyle(angle=180-th*180/np.pi)

                count = 0
            #endregion
            continue


# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Setup and run experiment
if __name__ == '__main__':
    #region : Setup scopes
    if IS_PHYSICAL_QCAR:
        fps = 10
    else:
        fps = 30
        
    # Scope for monitoring speed controller
    speedScope = MultiScope(
        rows=3,
        cols=1,
        title='Vehicle Speed Control',
        fps=fps
    )
    speedScope.addAxis(
        row=0,
        col=0,
        timeWindow=tf,
        yLabel='Vehicle Speed [m/s]',
        yLim=(0, 1)
    )
    speedScope.axes[0].attachSignal(name='v_meas', width=2)
    speedScope.axes[0].attachSignal(name='v_ref')

    speedScope.addAxis(
        row=1,
        col=0,
        timeWindow=tf,
        yLabel='Speed Error [m/s]',
        yLim=(-0.5, 0.5)
    )
    speedScope.axes[1].attachSignal()

    speedScope.addAxis(
        row=2,
        col=0,
        timeWindow=tf,
        xLabel='Time [s]',
        yLabel='Throttle Command [%]',
        yLim=(-0.3, 0.3)
    )
    speedScope.axes[2].attachSignal()

    # Scope for monitoring steering controller
    if enableSteeringControl:
        steeringScope = MultiScope(
            rows=4,
            cols=2,
            title='Vehicle Steering Control',
            fps=fps
        )

        steeringScope.addAxis(
            row=0,
            col=0,
            timeWindow=tf,
            yLabel='x Position [m]',
            yLim=(-2.5, 2.5)
        )
        steeringScope.axes[0].attachSignal(name='x_meas')
        steeringScope.axes[0].attachSignal(name='x_ref')

        steeringScope.addAxis(
            row=1,
            col=0,
            timeWindow=tf,
            yLabel='y Position [m]',
            yLim=(-1, 5)
        )
        steeringScope.axes[1].attachSignal(name='y_meas')
        steeringScope.axes[1].attachSignal(name='y_ref')

        steeringScope.addAxis(
            row=2,
            col=0,
            timeWindow=tf,
            yLabel='Heading Angle [rad]',
            yLim=(-3.5, 3.5)
        )
        steeringScope.axes[2].attachSignal(name='th_meas')
        steeringScope.axes[2].attachSignal(name='th_ref')

        steeringScope.addAxis(
            row=3,
            col=0,
            timeWindow=tf,
            yLabel='Steering Angle [rad]',
            yLim=(-0.6, 0.6)
        )
        steeringScope.axes[3].attachSignal()
        steeringScope.axes[3].xLabel = 'Time [s]'

        steeringScope.addXYAxis(
            row=0,
            col=1,
            rowSpan=4,
            xLabel='x Position [m]',
            yLabel='y Position [m]',
            xLim=(-2.5, 2.5),
            yLim=(-1, 5)
        )

        im = cv2.imread(
            images.SDCS_CITYSCAPE,
            cv2.IMREAD_GRAYSCALE
        )

        steeringScope.axes[4].attachImage(
            scale=(-0.002035, 0.002035),
            offset=(1125,2365),
            rotation=180,
            levels=(0, 255)
        )
        steeringScope.axes[4].images[0].setImage(image=im)

        referencePath = pg.PlotDataItem(
            pen={'color': (85,168,104), 'width': 2},
            name='Reference'
        )
        steeringScope.axes[4].plot.addItem(referencePath)
        referencePath.setData(waypointSequence[0, :],waypointSequence[1, :])

        steeringScope.axes[4].attachSignal(name='Estimated', width=2)

        arrow = pg.ArrowItem(
            angle=180,
            tipAngle=60,
            headLen=10,
            tailLen=10,
            tailWidth=5,
            pen={'color': 'w', 'fillColor': [196,78,82], 'width': 1},
            brush=[196,78,82]
        )
        arrow.setPos(initialPose[0], initialPose[1])
        steeringScope.axes[4].plot.addItem(arrow)
    #endregion

    #region : Setup control thread, then run experiment
    controlThread = Thread(target=controlLoop)
    controlThread.start()

    try:
        while controlThread.is_alive() and (not KILL_THREAD):
            MultiScope.refreshAll()
            time.sleep(0.01)
    finally:
        KILL_THREAD = True
    #endregion
    if not IS_PHYSICAL_QCAR:
        qlabs_setup.terminate()

    input('Experiment complete. Press any key to exit...')
#endregion