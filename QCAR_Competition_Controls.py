"""
Team QCardinals 
modified version of provided Vehicle_Control.py to work with traffic sign detection and competition setup
"""
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
import time
import cv2
# environment objects
from qvl.real_time import QLabsRealTime
import pal.resources.rtmodels as rtmodels 
# qcar camera imports
from pal.products.qcar import QCar
from pal.utilities.math import *

# Kept parameters for expirament setup
# ===== Timing Parameters
# - tf: experiment duration in seconds.
# - startDelay: delay to give filters time to settle in seconds.
# - controllerUpdateRate: control update rate in Hz. Shouldn't exceed 500
tf = 37
startDelay = 5
controllerUpdateRate = 500

# camera timing parameters 
sampleRate     = 30.0
sampleTime     = 1/sampleRate
simulationTime = tf

# ===== Speed Controller Parameters
# - v_ref: desired velocity in m/s
# - K_p: proportional gain for speed controller
# - K_i: integral gain for speed controller
v_ref = .75
K_p = 0.5
K_i = 1

# ===== Steering Controller Parameters
# - enableSteeringControl: whether or not to enable steering control
# - K_stanley: K gain for stanley controller
# - nodeSequence: list of nodes from roadmap. Used for trajectory generation.
enableSteeringControl = True #False
K_stanley = .75
nodeSequence = [10, 2, 4, 14, 20, 22, 10]   # [2, 20, 10, 2] # [0, 20, 0]


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
    from Setup_Competition import setup
    setup(
        # setup start position for the parallel park portion 
        initialPosition=[-1.23, -0.75, 0],
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

    def reset(self):
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

class SteeringController:
    def __init__(self, waypoints, k=K_stanley, cyclic=True, filter_coeff=1.2):
        self.maxSteeringAngle = np.pi / 7.5
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
        self.p_ref = (0, 0)
        self.th_ref = 0
        self.left_bias = np.pi/70
        self.filter_coeff = filter_coeff
        self.filtered_steering_angle = 0
        self.prev_steering_angle = 0  # Store previous steering angle for filtering

    def reset(self):
        self.prev_steering_angle = 0

    # def update(self, p, th, speed):
    #     wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)]
    #     wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)]
    #     v = wp_2 - wp_1
    #     v_mag = np.linalg.norm(v)
    #     if v_mag == 0:
    #         return 0

    #     v_uv = v / v_mag
    #     tangent = np.arctan2(v_uv[1], v_uv[0])
    #     s = np.dot(p - wp_1, v_uv)

    #     if s >= v_mag:
    #         if self.cyclic or self.wpi < self.N - 2:
    #             self.wpi += 1

    #     ep = wp_1 + v_uv * s
    #     ct = ep - p
    #     dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
    #     ect = np.linalg.norm(ct) * np.sign(dir)
    #     psi = wrap_to_pi(tangent - th)

    #     steering_angle = psi + np.arctan2(self.k * ect, speed) + self.left_bias
    #     # Apply low-pass filter
    #     self.filtered_steering_angle += self.filter_coeff * (steering_angle - self.filtered_steering_angle)
        
    #     return np.clip(self.filtered_steering_angle, -self.maxSteeringAngle, self.maxSteeringAngle)   
    def update(self, p, th, speed):
        # Offset the waypoint coordinates to the left by a certain distance
        offset_distance = 0.0125  # Adjust this value as needed
        left_offset = np.array([-offset_distance * np.sin(th), offset_distance * np.cos(th)])

        wp_1 = self.wp[:, np.mod(self.wpi, self.N - 1)] + left_offset
        wp_2 = self.wp[:, np.mod(self.wpi + 1, self.N - 1)] + left_offset
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
        
        # if steering_angle > 0:
        #     steering_angle = psi + np.arctan2(self.k * ect, speed) 
        # elif steering_angle < 0:
        #     pass
        # print(steering_angle)
        # Apply low-pass filter
        self.filtered_steering_angle += self.filter_coeff * (steering_angle - self.filtered_steering_angle)
        
        return np.clip(self.filtered_steering_angle, -self.maxSteeringAngle, self.maxSteeringAngle)

    
    
# Edited speed value 'u' to be a fixed value, this prevents the car from overwriting commands with detection on
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
                # removed line  - u = speedController.update(v, v_ref, dt) b/c fixed speed works better with the traffic
                # sign det
                u = 0.05
                #endregion
                print(delta)
                #  debugging delta value print(delta) created conitional for steering angle based on delta

                if delta < 0.04:
                    u = 0.14
                else: 
                    u = 0.0685                    
                
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
