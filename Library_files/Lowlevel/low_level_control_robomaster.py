import os
import sys

parent_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
sys.path.append(parent_path)
if parent_path in sys.path:
    pass
else:
    raise Exception('Parent path is not appended sys.path')

# default 
from timeit import default_timer as timer
import time
import logging

import threading
from threading import Thread

import time
import numpy as np
import numpy.linalg as np_la
from math import *

# same dir
from .low_level_control_methods import LowLevelControlMethods

# upper dir
from normalize import *


########################################### low level control end ###########################################
lock = threading.Lock()
MyRobomaster = None
g_distance_matrix = None

class LowLevelControlRobomaster():
    def __init__(self, ESTM, robot:MyRobomaster, NUMOF_SENSORS, ACHIEVED_CONDITION_DISTANCE, BLENDING_STATE, OBST_DETECT_MARGIN=5):
        self.recv_cnt = 0
        self.send_cnt = 0
        
        self.ESTM = ESTM
        self.rob = robot
        self.NUMOF_SENSORS = NUMOF_SENSORS

        self.node_idx = robot.node_idx
        self._chassis = None
        
        self.task_vector = (0, 0)
        self.latest_task_vector = None
        self.is_task_achieved = False

        self.LLCMethods = LowLevelControlMethods(BLENDING_STATE)
        ##### go to goal #####

        self.ACHIEVED_CONDITION_DISTANCE = ACHIEVED_CONDITION_DISTANCE
        self.ROT_ANGLE_RESOLUTION = 10 # degree 

        ##### avoid obstacle #####
        self.MAX_DETECT_RANGE = 10.0
        self.MIN_DETECT_RANGE = 0.01
        self.OBST_DETECT_RANGE = MAX_DETECT_DISTANCE = BLENDING_STATE['detected']['d']

        self.now_state = 'safe' 
        self.state_desired = 'safe' 
        self.state_undesired = 'fw' 
        self.OBST_DETECT_MARGIN = OBST_DETECT_MARGIN #! 현재 감지되지 않았더라도, 계속 감지된 상태임을 유지하는 횟수
        self.obst_margin_cnt_down = 0 

        ##### consensus #####
        self.CONSENSUS_ACHIEVED_DIST = 0.80

    def gimbal_always_to_center(self, ep_gimbal, idx): #!gimbal
        cnt = 0
        delay = 1
        while cnt < 180*(1/(delay)): #!gimbal
            try:
                ep_gimbal.recenter(pitch_speed=360, yaw_speed=360).wait_for_completed()
            except:
                print(f'[NODE{idx}] gimbal break')
                break
            time.sleep(delay)

    def callback_receive_from_node(self, sensor_raw_data, node_idx):
        """
        receive sensor data from callback function,
        and send control input to robot
            Args:
                sensor_raw_data: distance raw data from TOF Infrared Distance Sensor
                node_idx: index of robot
        """
        global g_distance_matrix
        distance = sensor_raw_data
        # mylog.error("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))
        # print("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))

        M = self.NUMOF_SENSORS
        #! Update distance information
        with lock:
            g_distance_matrix[node_idx:] = distance[:M]
        
        # print(f'g_distance_matrix: {g_distance_matrix}')
        self.recv_cnt += 1

    def send_to_node(self, g_task_vector_matrix, g_pos_matrix, g_prox_uvm_fw, enableAO=True):
        robot = self.rob
        wheel_rpm_vector = self.vector_to_wheel_rpm_ratio(g_task_vector_matrix, g_pos_matrix, g_prox_uvm_fw, funcAO=enableAO)
        robot.set_wheel_speed(wheel_rpm_vector)
        self.send_cnt += 1

    #!#####################################################
    def vector_to_mecanum_wheel_ratio(self, vx, vy) -> tuple:
        w1 = vx+vy
        w2 = vx-vy
        w3 = vx+vy
        w4 = vx-vy
        wheel_speed = (w1, w2, w3, w4)
        return wheel_speed

    def calc_PID_wheel_speed(self, normal_wheel_speed, d, FIELD_DIAGONAL_DISTANCE:float = 3.66):
        vmax = max(abs(normal_wheel_speed))
        w = self.LLCMethods.get_PID_weight(d, FIELD_DIAGONAL_DISTANCE, self.rob.MAX_RPM)

        if vmax*w < self.rob.MIN_RPM:
            ratio = self.rob.MIN_RPM / vmax
            w = ceil_from_digits(ratio, 3)

        pid_wheel_speed = w * normal_wheel_speed
        return pid_wheel_speed
    
    def calc_pid_rotation_pid_wheel_speed(self, normal_wheel_speed, deg_error):
        vmax = max(abs(normal_wheel_speed))

        max_error = 180
        max_rot_speed = 360
        w = self.LLCMethods.get_PID_weight(abs(deg_error), max_error, max_rot_speed)

        if vmax*w < self.rob.MIN_ROTATION_RPM:
            ratio = self.rob.MIN_ROTATION_RPM / vmax
            w = ceil_from_digits(ratio, 3)

        pid_wheel_speed = w * normal_wheel_speed
        return pid_wheel_speed

    def calc_limit_rotation_speed_in_bound(self, w):
        if self.rob.MAX_RPM < w:
            w = self.rob.MAX_RPM
        elif w < self.rob.MIN_ROTATION_RPM:
            w = self.rob.MIN_ROTATION_RPM
        else:
            pass

        return w


    def calc_global_to_local_coordinate(self, heading, global_vector):
        #* ----- 1. calc desired vector to move -----
        #! goal position에 대한 task_vector 계산은 Middle level: SetEacGoal class 의 execute()에서 담당
        #! global coordinate system
        vgx, vgy = global_vector
        vx_, vy_ = vgx, vgy
        
        #* ----- 3. calc vector expression from global Coordinate System to Local(robot) C.S.  -----
        # calc difference: between robot heading and global starting point
        theta_g = atan2(vy_, vx_)
        theta_c = heading
        theta_gc = theta_g-theta_c # angle difference

        #! local:robot(=local) coordinate system
        d = sqrt(vx_**2 + vy_**2) # uclidian distance from robot to destination
        vx_local = d*cos(theta_gc)
        vy_local = d*sin(theta_gc)

        return vx_local, vy_local


    def get_task_vector(self, g_task_vector_matrix):
        return g_task_vector_matrix[self.node_idx,:]
    
    def get_task_vector_for_state(self, g_task_vector_matrix):
        if self.obst_margin_cnt_down > 0:
            if self.now_state == self.state_desired:
                # print(f'Maintain {self.obst_margin_cnt_down}')
                #! maintain latest vector for preventing collision
                #! this cycle-number called by 'margin'
                task_vector = self.latest_task_vector
                self.obst_margin_cnt_down -= 1
                
            else:
                # print(f'Dangerous {self.obst_margin_cnt_down}')
                task_vector = self.latest_task_vector
        else:
            # print(f'Safe {self.obst_margin_cnt_down}')
            # original task
            task_vector = (self.get_task_vector(g_task_vector_matrix)/1000).reshape(2,1)
            
        return task_vector

    
    def get_node_obst_distance(self):
        global g_distance_matrix
        return g_distance_matrix[self.node_idx,:] / 1000 #! estimation에서 단위변환까지 다 하게끔 변경.
    #! global 변수 하나 더 만들어서, raw data 전용, 단위 변환된 distance 전용으로 나눠서 하는게 어떨지?
    
    # TODO: go to low level control part
    def get_follow_wall_vector_weight_param(self, np_d):
        alpha = (1/10)
        d_max = self.MAX_DETECT_RANGE
        d_min = self.MIN_DETECT_RANGE
        d_bound = np.where(d_max < np_d, d_max, np_d)
        d_bound = np.where(d_bound < d_min, d_min, d_bound)

        d_weight = alpha*(1/d_bound)
        return d_weight
    
    def blend(self, w_fw):
        return self.LLCMethods.get_aggregated_weight('closest', w_fw)
    
    def get_heading_degree(self, g_pos_matrix):
        x, y, theta = g_pos_matrix[self.node_idx,:]
        return theta
    
    #!#####################################################
    def calc_pid_normal_wheel_ratio(self, wheel_ratio, deg):
        max_error = 180
        max_rot_speed = 1
        w = self.LLCMethods.get_PID_weight(abs(deg), max_error, max_rot_speed)
        pid_ = w * wheel_ratio
        return pid_
    
    def get_prox_v_fw(self, g_prox_uvm_fw):
        return g_prox_uvm_fw[self.node_idx,:]
    
    def get_u_fw_vector(self, dists, g_prox_uvm_fw):
        ESTM = self.ESTM
        #* get node_i`s aggregated follow wall vector
        u_fw_comp = self.get_prox_v_fw(g_prox_uvm_fw)
        d_w = self.get_follow_wall_vector_weight_param(dists)
        u_fw_comp = d_w * u_fw_comp

        u_fw_x, u_fw_y = ESTM.decompose_real_imag(u_fw_comp) # real:(3,1), imag:(3,1)
        u_fw_x, u_fw_y = u_fw_x.reshape(1,3), u_fw_y.reshape(1,3)
        u_fw = np.concatenate((u_fw_x,u_fw_y), axis=0) # (2,3)
        u_fw = u_fw.sum(axis=1).reshape(2,1)

        return u_fw

    def get_aggregated_weight(self, dists):
        LLCM = self.LLCMethods
        w_fw = LLCM.blending_function_np(np.append(0,dists)) # np.append(0,dist): np.vectorize 의 첫번째 요소 return 값이 False면 작동안된다.
        do = w_fw[1:]
        w_ag = self.blend(do)

        return w_ag
    
    def get_rot_wheel_ratio(self, u_ao, theta_h):
        LLCM = self.LLCMethods
        robot = self.rob

        vx_g, vy_g = u_ao

        # get how much should this node rotation CW(clockwise) or CCW
        deg_rotation = LLCM.calc_desired_rotation_angle(theta_h, vx_g, vy_g)

        # if angle error is not zero -> sum rotation ratio and task-vector`s wheel ratio
        if abs(deg_rotation) > self.ROT_ANGLE_RESOLUTION:
            if deg_rotation > 0: # rotation counter clock wise 
                rot_direction = robot.ROTATION_CCW
            else: # rotation clock wise 
                rot_direction = robot.ROTATION_CW   

            rotation_normal_wheel_ratio = normalized_vector(rot_direction)

            #!For PID Controlled rotation ratio:
            #! rotation_normal_wheel_ratio = self.calc_pid_normal_wheel_ratio(rotation_normal_wheel_ratio, deg_rotation)

        # if no angle error -> nothing to sum
        else:
            rotation_normal_wheel_ratio = np.array((0,0,0,0))

        return rotation_normal_wheel_ratio, deg_rotation
    
    #! [changed here: funcAO]
    def vector_to_wheel_rpm_ratio(self, g_task_vector_matrix, g_pos_matrix, g_prox_uvm_fw, funcAO=True):
        """
        Get wheel rpm values to achieve node task-vector

        Args:
            funcAO: option do Avoid obstacle or not
                True: ->Avoid obstacle enable  
                False: ->Avoid obstacle disable
        Return:
            limited_wheel_rpm: calculated wheel rpm value bounded in [50, 1000]
        """
        LLCM = self.LLCMethods
        robot = self.rob

        #* 1. get node_i`s task vector according to now state
        u_t = self.get_task_vector_for_state(g_task_vector_matrix)
        magnitude = np_la.norm(u_t)

        # when get to close to goal position -> stop
        #! self.now_state == self.state_desired 조건이 없는 경우,
        #! 회피 기동 중에 회피 벡터 u_ao 의 magnitude가 작아지게 된다면
        #! 마찬가지로 멈추기 때문에 해당 조건을 넣은 것임.
        if magnitude < self.ACHIEVED_CONDITION_DISTANCE and self.now_state == self.state_desired: 
            stop_ratio = (0,0,0,0)
            return stop_ratio

        #* 2. get node_i`s follow wall vector     
        dists = self.get_node_obst_distance()
        u_fw = self.get_u_fw_vector(dists, g_prox_uvm_fw)

        #* 3. get node_i`s aggregated weight of follow wall vector     
        w_ag = self.get_aggregated_weight(dists)

        # if obstacle is detected -> change self.now_state 
        if w_ag != 0:
            # robot.set_wheel_speed((0,0,0,0))
            self.now_state = self.state_undesired
            self.obst_margin_cnt_down = self.OBST_DETECT_MARGIN
            
        else:
            self.now_state = self.state_desired

        #* 4. calculation of blending vector sum of task vector and follow wall vector
        if funcAO == True:    
            u_ao = w_ag*u_fw + (1-w_ag)*u_t
        else: 
            u_ao = u_t
            self.now_state = self.state_desired
            self.obst_margin_cnt_down = 0

        self.latest_task_vector = u_ao 

        #* 5. global to local vector component conversion
        theta_h = self.get_heading_degree(g_pos_matrix)
        vx_l, vy_l = self.calc_global_to_local_coordinate(theta_h, u_ao)

        #* 6. get task-vector wheel ratio
        # calc ratio of each mecanum wheel
        task_wheel_ratio = self.vector_to_mecanum_wheel_ratio(vx_l, vy_l)
        # normalize ratio
        task_normal_wheel_ratio = normalized_vector(task_wheel_ratio)

        #* 7. get rotation ratio
        rotation_normal_wheel_ratio, deg_rotation = self.get_rot_wheel_ratio(u_ao, theta_h)

        #* 8. blending wheel ratio: wheel ratio about task-vector(vectorized) and rotation(heading)
        if rotation_normal_wheel_ratio.any(): # rotation_normal_wheel_ratio is not zero vector
            mag_tr = np_la.norm(task_normal_wheel_ratio)
            mag_rr = np_la.norm(rotation_normal_wheel_ratio)
            mag_sum = mag_tr+mag_rr

            beta = max(mag_tr/mag_sum, mag_rr/mag_sum)

        else:
            beta = 0

        combined_ratio = (1-beta)*task_normal_wheel_ratio + beta*(rotation_normal_wheel_ratio)
        combined_ratio = normalized_vector(combined_ratio)
        
        #* 9. PID Control: adjust wheel speed based on : "magnitude of u_ao"
        if rotation_normal_wheel_ratio.any():
            pid_wheel_rpm = self.calc_pid_rotation_pid_wheel_speed(combined_ratio, deg_rotation)
        else:
            pid_wheel_rpm = self.calc_PID_wheel_speed(combined_ratio, magnitude, FIELD_DIAGONAL_DISTANCE=3.66)

        #* 10. limit bounded value of max or min wheel rpm
        #* limit wheel speed to ensure speed is in max RPM or min RPM coverage
        limited_wheel_rpm = LLCM.calc_limit_val_in_bound(pid_wheel_rpm, robot.MAX_RPM, 25)
        
        #* return bounded rpm
        return limited_wheel_rpm

    def init_gimbal(self):
        robot = self.rob
        th_gimbal = None
        #* ----- 2. set callback to send command ----- 
        if robot.model == 's1':
            ep_gimbal = robot.gimbal #! if you use gimbal
            th_gimbal = Thread(target=self.gimbal_always_to_center, args=(ep_gimbal, robot.node_idx,), daemon=True) #!if you use gimbal
            th_gimbal.start() #!if you use gimbal

        return th_gimbal
    
    def init_sensor(self, g_distance_mat, UP_LINK_FREQ:int=50):
        global g_distance_matrix
        g_distance_matrix = g_distance_mat
        robot = self.rob
        #* ----- 1. set callback function to get sensor raw data ----- 
        ep_sensor = robot.sensor
        ep_sensor.sub_distance(freq=UP_LINK_FREQ, callback=self.callback_receive_from_node, node_idx=robot.node_idx)

    def init_send_cmd(self, task_vm):
        th_send = Thread(target=self.send_to_node, args=(task_vm, ), daemon=True) 
        th_send.start()
        return th_send

    # def low_level_control_routine(self, g_distance_matrix):
    #     robot = self.rob

    #     th_gimbal = self.init_gimbal()

    #     self.init_sensor(g_distance_matrix)

    #     th_send = self.init_send_cmd()

    #     #* ----- 3. wait for main flag on ----- 
    #     while not g_flag_program_run: pass

    #     #* ----- 4. while main flag on, just sleep -> routines are running by thread ----- 
    #     while g_flag_program_run: time.sleep(1)
                
    #     #* ----- 5. terminate thread ----- 
    #     robot._chassis.unsub_attitude() # stop receiving sensor raw data
    
    #     th_send.join()
    #     if robot.model == 's1': th_gimbal.join()

    #     robot.close() # TODO: Close timing 확인하기.   
    #     print(f'[NODE{robot.node_idx}] Closed')    