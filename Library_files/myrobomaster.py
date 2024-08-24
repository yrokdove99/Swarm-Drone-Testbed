from math import *
import numpy as np
from .robomaster_pkg import robot

#from - import normalized_vector
#import ceil_from_digits

class MyRobomaster(robot.Robot):
    def __init__(self, node_idx:int, ip:str, sn:str=None, model:str=None, sensors_position:dict=None):
        global OBST_DETECT_MARGIN, MAX_DETECT_DISTANCE # 1.0
        super().__init__()
        try:
            print(f'[NODE{node_idx}] Try to initialize')
            self.initialize2(conn_type='sta', proto_type='udp', ip=ip, sn=sn)
            print(f'[NODE{node_idx}] Success initialize')
        except Exception as e:
            raise Exception(e)
        
        self.node_idx = node_idx
        self.model = model

        ##### mechanum #####
        self.BODY_WIDTH = 320
        self.BODY_DEPTH = 240
        self.BODY_HEIGHT = 270

        self.MIN_RPM = 100
        self.MAX_RPM = 1000 # or -1000
        self.MIN_ROTATION_RPM = 75

        self.MAX_SPEED_FORWARD = 3500 # 3500 mm / sec
        self.MAX_SPEED_SIDE =  2800 # 2800 mm / sec
        self.RPM_PER_METER = 285 # 285 rpm / 1 meter
        self.METER_PER_ROT_FORWARD = 210 # 0.21 meter / 1 rotation, forward
        self.METER_PER_ROT_SIDE = 168 # 0.168 meter / 1 rotation, side
        self.MAX_ANGLE_SPEED = None #1/self._wheel_base_length*(self.MAX_SPEED-(-self.MAX_SPEED))

        self.ROTATION_CCW = np.array((1, -1, -1, 1)) # counter clock wise
        self.ROTATION_CW = np.array((-1, 1, 1, -1)) # clock wise

    def set_wheel_speed(self, wheel_rpm_vector):
        w1, w2, w3, w4 = wheel_rpm_vector
        self._chassis.drive_wheels(w1, w2, w3, w4, timeout=0.2)

#* -------------- Plant Part  --------------