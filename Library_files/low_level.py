from timeit import default_timer as timer
from .myrobomaster import MyRobomaster
# from robomaster_imports import mylog

import time
import logging

import threading
from threading import Thread
import time
import numpy as np
########################################### low level control end ###########################################
lock = threading.Lock()

class LowLevelControl():
    def __init__(self,  robot:MyRobomaster, g_flag_program_run:list, g_flag_task_vector:list, g_prox_information:np.array):
        self.rob = robot

        self.g_flag_program_run = g_flag_program_run
        self.g_flag_task_vector = g_flag_task_vector
        self.g_prox_information = g_prox_information
        self.NUMOF_SENSORS = g_prox_information.shape[0]
    
    def gimbal_always_to_center(self, ep_gimbal, idx): #!gimbal
        while not self.g_flag_program_run[0]: pass
        while self.g_flag_program_run[0]: #!gimbal
            try:
                ep_gimbal.recenter(pitch_speed=360, yaw_speed=360).wait_for_completed()
            except:
                print(f'[NODE{idx}] gimbal break')
                break

    def callback_receive_from_node(self, sensor_raw_data, node_idx):
        """
        receive sensor data from callback function,
        and send control input to robot
            Args:
                sensor_raw_data: distance raw data from TOF Infrared Distance Sensor
                node_idx: index of robot
        """
        distance = sensor_raw_data
        # mylog.error("tof1:{0}  tof2:{1}  tof3:{2}  tof4:{3}".format(distance[0], distance[1], distance[2], distance[3]))

        N = self.NUMOF_SENSORS
        robot = self.rob
        robot._prox.update_sensor_raw_data(sensor_raw_data[:N]) # TODO: 추후 수정 -> update_prox(raw_data) 등으로 교체.
        
        with lock:
            self.g_prox_information[node_idx:] = sensor_raw_data[:N]

    def send_to_node(self):
        robot = self.rob
        while not self.g_flag_program_run[0]: pass

        start = timer()
        while self.g_flag_program_run[0]:
            if self.g_flag_task_vector[0]:
                robot.set_wheel_speed2()
            else:
                pass # no robot control
        end = timer() - start

        print(f'[NODE{robot.node_idx}] time:{end} send_cmd_cnt:{robot.cmd_cnt} rate:{robot.cmd_cnt/end:.3f}')

    def low_level_control_start(self, UP_LINK_FREQ:int=50):
        robot = self.rob
        # g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_flag_program_run_on'])

        #* ----- 1. set callback function to get sensor raw data ----- 
        ep_sensor = robot.sensor
        ep_sensor.sub_distance(freq=UP_LINK_FREQ, callback=self.callback_receive_from_node, node_idx=robot.node_idx)

        #* ----- 2. set callback to send command ----- 
        if robot.model == 's1':
            ep_gimbal = robot.gimbal #! if you use gimbal
            th_gimbal = Thread(target=self.gimbal_always_to_center, args=(ep_gimbal, robot.node_idx,), daemon=True) #!if you use gimbal
            th_gimbal.start() #!if you use gimbal

        th_send = Thread(target=self.send_to_node, args=(), daemon=True) 
        th_send.start()

        #* ----- 3. wait for main flag on ----- 
        while not self.g_flag_program_run[0]: pass
        # g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: free_wait_flag_program_run_on'])

        #* ----- 4. while main flag on, just sleep -> routines are running by thread ----- 
        while self.g_flag_program_run[0]: time.sleep(1)
                
        #* ----- 5. terminate thread ----- 
        robot._chassis.unsub_attitude() # stop receiving sensor raw data
    
        th_send.join()
        if robot.model == 's1': th_gimbal.join()

        robot.close() # TODO: Close timing 확인하기.   
        print(f'[NODE{robot.node_idx}] Closed')    