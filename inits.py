from Library_files.robomaster_inits import get_online_robot_info, mylog

from Library_files.estimation import Estimation
from Library_files.motion_track import create_motion_tracker, tracking_position

from Library_files.myrobomaster import MyRobomaster
from Library_files.Lowlevel.low_level_control_methods import LowLevelControlMethods as llcMethod
from Library_files.Lowlevel.low_level_control_robomaster import LowLevelControlRobomaster as llcRobo

from Library_files.middle_level import MiddleLevelControl
from Library_files.high_level import HighLevelControl

import time
import numpy as np
import threading
from threading import Thread
import logging

from timeit import default_timer as timer

import sys
from datetime import datetime as dt
g_debug_msg = []
sys._getframe(0).f_code.co_name

def enable_flag_tracking():
    global g_flag_tracking
    g_flag_tracking = True

def disable_flag_tracking():
    global g_flag_tracking
    g_flag_tracking = False

def wait_flag_tracking_on():
    global g_flag_tracking
    while not g_flag_tracking: pass

# -------------- estimation --------------
def _exist_tracking_information():
    global g_tracking_info
    if None in g_tracking_info.values():
        return False
    else:
        return True

def _exist_sensor_data():
    global g_robots
    global g_distance_matrix
    if None in g_robots.values(): return False
    
    for vector in g_distance_matrix:
        try:
            if np.any(vector==-1):
                return False
        except Exception as e:
            print(e)
            return False
    return True

def wait_until_tracking_information_init():
    while not _exist_tracking_information(): pass

def wait_until_sensor_data_init():
    while not _exist_sensor_data(): pass

# -------------- Low level control --------------
def _exist_task_vector():
    global g_flag_task_vector
    return g_flag_task_vector
def wait_until_task_vectors_init():
    while not _exist_task_vector(): pass

# -------------- Middle level control --------------
def _exist_estimation_data():
    global g_pos_matrix
    global g_prox_vm

    if np.any(g_pos_matrix==-1):
        return False
    if np.any(g_prox_vm==-1) :
        return False

    return True

def init_task_vector():
    global g_flag_task_vector
    g_flag_task_vector = 1

def wait_until_estimation_data_init():
    while not _exist_estimation_data(): pass

def wait_purpose_from_high_level():
    global g_objective
    while g_objective == None: pass

# -------------- Main control --------------
def enable_flag_program_run():
    global g_flag_program_run
    g_flag_program_run = True

def disable_flag_program_run():
    global g_flag_program_run
    g_flag_program_run = False

def wait_flag_program_run_on():
    global g_flag_program_run
    while not g_flag_program_run: pass

def loop_motion_track():
    global g_flag_program_run
    global g_tracking_info

    morion_tracker = create_motion_tracker('127.0.0.1', g_tracking_info)
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait'])
    while not g_flag_program_run: pass
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: start'])

    while g_flag_program_run:
        tracking_position(morion_tracker)
        
    print('[Motion Track End]')

def loop_estimation():
    global ESTM
    global g_tracking_info
    global g_distance_matrix

    global g_pos_matrix
    global g_prox_vm
    global g_prox_uvm_fw

    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_until_tracking_information_init'])
    wait_until_tracking_information_init()
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_until_sensor_data_init'])
    wait_until_sensor_data_init()
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: free_wait_until_sensor_data_init'])

    while not g_flag_program_run: pass
    while g_flag_program_run:
        g_pos_matrix, g_prox_vm, g_prox_uvm_fw = ESTM.estimation_routine(g_tracking_info, g_distance_matrix)
    print('[Estimation End]')

def loop_high_level_control(HLC):
    global g_flag_program_run
    global g_objective

    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_flag_program_run_on'])
    wait_flag_program_run_on()
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: free_wait_flag_program_run_on'])

    HLC_start_time = timer()
    HLC_elapsed_time = timer() - HLC_start_time
    while g_flag_program_run and not HLC._is_over_running_time(HLC_elapsed_time):
        g_objective = HLC.high_level_control_routine()
        HLC_elapsed_time = timer() - HLC_start_time

def loop_middle_level_control(MLC):
    global g_flag_program_run
    global g_objective
    global g_task_vector_matrix
    
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_until_robot_data_init'])
    wait_until_estimation_data_init()
    g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: free_wait_until_robot_data_init'])
    
    while g_flag_program_run:
        g_task_vector_matrix = MLC.middle_level_control_routine(g_objective)

def loop_low_level_control(LLC, enableAvoidObstacle):
    global g_flag_program_run

    global g_task_vector_matrix
    global g_distance_matrix
    global g_pos_matrix
    global g_prox_uvm_fw

    robot = LLC.rob

    th_gimbal = LLC.init_gimbal()
    LLC.init_sensor(g_distance_matrix)

    while np.any(g_task_vector_matrix==0): pass

    #* ----- 3. wait for main flag on ----- 
    while not g_flag_program_run: pass
    start = timer()
    #* ----- 4. while main flag on, just sleep -> routines are running by thread -----
    while g_flag_program_run:
        LLC.send_to_node(g_task_vector_matrix, g_pos_matrix, g_prox_uvm_fw, enableAvoidObstacle)
    end = timer() - start
            
    #* ----- 5. terminate thread ----- 
    robot._chassis.unsub_attitude()

    if robot.model == 's1':
        th_gimbal.join()
    
    robot.close()  
    print(f'[NODE{robot.node_idx}] Closed / recv_rate:{LLC.recv_cnt/end} send_rate:{LLC.send_cnt/end}')    

def example_running(task, robots_info, NODE_IDX_LIST, SENSOR_ANGLES, ACHIEVED_CONDITION_DISTANCE, BLENDING_STATE, OBST_DETECT_MARGIN, RUNNING_TIME_LIMIT, enableAvoidObstacle, ROTATION):
    global ESTM
    
    NUMOF_NODE = len(NODE_IDX_LIST)
    NUMOF_SENSOR = len(SENSOR_ANGLES)

    #* ------- 1. motion tracking thread enable, but no start by flag -------
    Motion_tracking_thread = Thread(target=loop_motion_track, args=(), daemon = True)
    Motion_tracking_thread.start()
    
    #* ------- 2. estimating thread enable, but no start by flag -------
    ESTM = Estimation(NUMOF_NODE, SENSOR_ANGLES)
    estimating_thread = Thread(target=loop_estimation, args=(), daemon=True)
    estimating_thread.start()

    #* ------- 3.1. Node Init -------
    for node_idx in NODE_IDX_LIST:
        try:
            robot = MyRobomaster(*robots_info[node_idx], SENSOR_ANGLES)
            robot.MAX_RPM = 300
            robot._chassis =  robot.chassis
            g_robots[node_idx] = robot
            
        except Exception as e:
            print(f'[NODE{node_idx}] INIT ERROR')
            raise Exception(e)

    #* ------- 3.2. low level control thread start -------
    low_level_control_thread_list = []
    for node_idx, rob in g_robots.items():
        LLC = llcRobo(ESTM, rob, NUMOF_SENSOR, ACHIEVED_CONDITION_DISTANCE, BLENDING_STATE, OBST_DETECT_MARGIN)
        llc_thread = Thread(target=loop_low_level_control, args=(LLC,enableAvoidObstacle), daemon=True)
        llc_thread.start()
        low_level_control_thread_list.append(llc_thread)

    #* ------- 4. Middle level control -------
    mlc_class = MiddleLevelControl(ESTM, g_robots, ACHIEVED_CONDITION_DISTANCE, ROTATION)
    mlc_thread = Thread(target=loop_middle_level_control, args=(mlc_class,), daemon=True)
    mlc_thread.start()

    #* ------- 5. high level control -------
    g_objective = task
    hlc_class = HighLevelControl(g_objective, RUNNING_TIME_LIMIT)
    hlc_thread = Thread(target=loop_high_level_control, args=(hlc_class,), daemon=True)
    hlc_thread.start()

    #* ------- program start flag enable -------
    enable_flag_program_run()

    time.sleep(RUNNING_TIME_LIMIT)

    #* ------- program start flag enable -------
    disable_flag_program_run()

    #* ------- terminate all thread -------
    Motion_tracking_thread.join()
    estimating_thread.join()
    for llcth in low_level_control_thread_list:
        llcth.join()
    mlc_thread.join()
    hlc_thread.join()


def init_(NUMOF_NODE, SENSOR_ANGLES):
    global g_flag_program_run
    global g_tracking_info
    global g_distance_matrix
    global g_pos_matrix
    global g_prox_vm
    global g_prox_uvm_fw
    global g_task_vector_matrix
    global g_robots
    global g_objective

    mylog.setLevel(logging.ERROR)
    robots_info = get_online_robot_info(3)
    numFindNode = len(robots_info.keys())
    print(f'#Of Nodes{NUMOF_NODE}\n#Of find nodes:{numFindNode}\n{robots_info}')
    if NUMOF_NODE != numFindNode:
        raise Exception('Node count mismatch')

    NODE_IDX_LIST = [i for i in range(NUMOF_NODE)] if NUMOF_NODE else [0]
    NUMOF_SENSOR = len(SENSOR_ANGLES)
    
    g_flag_program_run = False
    g_tracking_info = {node_idx:None for node_idx in NODE_IDX_LIST}
    g_distance_matrix = -1*np.ones((NUMOF_NODE, NUMOF_SENSOR))
    g_pos_matrix = -1*np.ones((NUMOF_NODE, 3))
    g_prox_vm = -1*np.ones((NUMOF_NODE, NUMOF_SENSOR))
    g_prox_uvm_fw = -1*np.ones((NUMOF_NODE, NUMOF_SENSOR))
    g_robots = {node_idx:None for node_idx in NODE_IDX_LIST}
    g_task_vector_matrix = np.zeros((NUMOF_NODE, 2))

    return robots_info, NODE_IDX_LIST