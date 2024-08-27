import numpy as np
from numpy import exp as nexp
import time
from timeit import default_timer as timer
from math import *

#* -------------- Estimation Part  --------------
class Estimation():
    def __init__(self, NUMOF_NODES:int, SENSOR_ANGLES:list):
        self.NUMOF_NODES = NUMOF_NODES
        self.SENSOR_ANGLES = np.array(SENSOR_ANGLES)
        self.NUMOF_SENSORS = len(SENSOR_ANGLES)

        self.node_position_matrix = None
        self.node_distance_matrix = None

        self.vector_obst = None
        self.prox_vector_matrix = None # prox_dist_vector_matrix

        self.vector_fw = None
        self.prox_vector_fw_matrix = None

        self.vector_op = None
        self.prox_vector_op_matrix = None

        self.MAX_DETECT_RANGE = 10 # meter
        self.MIN_DETECT_RANGE = 0.01 # meter

        self.ESTM_THREAD_SLEEP_TIME : float = 0.02

    def get_node_distance_matrix(self):
        return self.node_distance_matrix
    def get_node_position_matrix(self):
        return self.node_position_matrix
    def get_prox_v_fw(self):
        global g_prox_uvm_fw
        return g_prox_uvm_fw[self.node_idx,:]
    
    def get_node_pos(self, node_idx):
        x, y, theta = self.node_position_matrix[node_idx,:]
        return x, y, theta

    def get_node_obstacle_vector_matrix(self, node_idx):
        return self.prox_vector_matrix[node_idx,:].reshape(2,3)
    def get_node_follow_wall_vector_matrix(self, node_idx):
        return self.prox_vector_fw_matrix[node_idx,:].reshape(2,3)
    def get_node_obst_opposite_vector_matrix(self, node_idx):
        return self.prox_vector_op_matrix[node_idx,:].reshape(2,3)

    def get_node_obstacle_vector_comp(self, node_idx):
        return self.vector_obst[node_idx,:]#.reshape(2,1)
    def get_node_follow_wall_vector_comp(self, node_idx):
        return self.vector_fw[node_idx,:]#.reshape(2,1)
    def get_node_obst_opposite_vector_comp(self, node_idx):
        return self.vector_op[node_idx,:]#.reshape(2,1)
    #* ################################################

    @staticmethod
    def dict2numpy(g_tracking_info):
        dic = g_tracking_info
        num = len(dic.keys())
        np_pos = np.zeros((num, 3)) # #of position information is 3
        np_ori = np.zeros((num, 4)) # #of orientation information is 4

        for k, val in dic.items():
            np_pos[k:] = np.array(val.position).reshape(1,3) # shape (1,3)
            np_ori[k:] = np.array(val.orientation).reshape(1,4) # shape (1,4)
        
        return np_pos, np_ori

    def update_global_pose_matrix(self, np_pos, np_ori):
        # read position information of each rigid body, in mm unit
        # (x,y,z) : x, -y, z position in mm unit -> np_pos[:, :-1] -> (n,2):just deal with (x,y)
        np_coordinates =  1000 * np_pos[:, :-1] # shape(n,2)
        n = np_coordinates.shape[0]

        # shape(n,)
        np_theta = np.arctan2(+2.0 * (np_ori[:,3] * np_ori[:,2] + np_ori[:,0] * np_ori[:,1]), +1.0 - 2.0 * (np_ori[:,1] * np_ori[:,1] + np_ori[:,2] * np_ori[:,2]))
        np_theta = np_theta.reshape(n,1) # shape(n,) -> shape(n,1)
        
        # global var update
        g_pos_matrix = np.concatenate((np_coordinates,np_theta),axis=1)
        self.node_position_matrix  = g_pos_matrix
        return g_pos_matrix, np_theta

    def update_prox_distance_matrix(self, sensor_raw_data) -> None:
        # global g_distance_matrix
        distance_matrix = np.round(sensor_raw_data / 1000, 2) # (n,3)

        if self.NUMOF_SENSORS < distance_matrix[:,:].shape[1]:
            distance_matrix = distance_matrix[:,:self.NUMOF_SENSORS]

        # g_distance_matrix = distance_matrix
        self.node_distance_matrix = distance_matrix
    
    def get_vector_matrix(self, mat_x, mat_y):
        N = self.NUMOF_NODES
        M = self.NUMOF_SENSORS

        mat = np.zeros((N, 2*M))
        mat[:,:M] = mat_x
        mat[:,M:] = mat_y
        return mat

    def decompose_real_imag(self, vector_comp): # vector_comp : (N,M)
        M = self.NUMOF_SENSORS
        np_real = np.round(np.real(vector_comp), M) # (N,M)
        np_imag = np.round(np.imag(vector_comp), M) # (N,M)
        return (np_real, np_imag)
    
    # No Use ---------------------------------
    def update_prox_follow_wall_vector_matrix(self, np_d, vector_comp):
        d_w = self.get_avoid_vector_weight_param(np_d)
        self.vector_fw = vector_comp * (d_w * nexp(1j*(pi/2)))
        self.prox_vector_fw_matrix = self.get_vector_matrix(*self.decompose_real_imag(self.vector_fw))
    def update_prox_opposite_vector_matrix(self, np_d, vector_comp):
        d_w = self.get_avoid_vector_weight_param(np_d)
        self.vector_op = vector_comp * (d_w * nexp(1j*(pi)))
        self.prox_vector_op_matrix = self.get_vector_matrix(*self.decompose_real_imag(self.vector_op))
    # No Use ---------------------------------

    def update_prox_vector_matrixes(self, np_heading):
        N = self.NUMOF_NODES
        M = self.NUMOF_SENSORS
        j = 1j

        np_theta = np_heading
        np_sp = self.SENSOR_ANGLES # sp = np.array([0, 45, -45])
        np_d = self.node_distance_matrix # np.array([0, 1, 0])
        np_phase = np.radians(np_sp) # (1,3)

        # np_phase(1,M)+np_theta(1,N) -> (N,M)
        vector_comp = nexp(j*(np_phase+np_theta)).reshape(N,M) # (N,M)
        vector_comp = np.round(vector_comp,4)

        g_prox_vm = np_d * vector_comp
        g_prox_uvm_fw = vector_comp * nexp(j*radians(-90))
        self.vector_obst = g_prox_vm
        self.prox_vector_matrix = g_prox_vm

        return g_prox_vm, g_prox_uvm_fw
        
    def estimation_routine(self, g_tracking_info, g_distance_matrix):
        """
        - By processing the tracking info obtained through the Motion Tracking-Motive Program
            - 1.Create global position vector matrix information and update it to g_pos_matrix.
            - g_pos_matrix: (Nx3) -sized matrix, column represents x, y theta information.

        - By processing the distance info obtained through the IR sensor
            - 2.Proximity vector matrix is created and updated to g_prox_vm.
            - g_prox_vm: (NxM) represented in (x,y) coordinates in the global coordinate system the distance to the obstacle detected by each of the (NxM) size matrix, N nodes, and M sensors.
        """
        #! g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_until_tracking_information_init'])
        # Motion tracking operates and waits until tracking information is initiated
        #! wait_until_tracking_information_init()
        #! g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: wait_until_sensor_data_init'])

        # Wait until low level control is activated and sensed distance information is initiated
        #! wait_until_sensor_data_init()
        # g_debug_msg.append([dt.now(), f'[{sys._getframe(0).f_code.co_name}]: free_wait_until_sensor_data_init'])
        
        #! while g_flag_program_run:
        estimating_start_time =  timer()
        
        #* 1. ------- global position estimation -------
        #* get global position vector and heading
        np_pos, np_ori = self.dict2numpy(g_tracking_info)
        #* update global position vector matrix : g_pos_matrix
        g_pos_matrix, np_theta = self.update_global_pose_matrix(np_pos, np_ori)

        #* 2. ------- proximity vector matrix estimation -------
        #* update proximity vector matrix : g_prox_vm
        self.update_prox_distance_matrix(g_distance_matrix)
        g_prox_vm, g_prox_uvm_fw = self.update_prox_vector_matrixes(np_theta)

        estimating_elapsed_time = timer() - estimating_start_time
        #* add intensional sleep time for time-regular update
        #* estimating_elapsed_time: code running time
        #* if estimating_elapsed_time > 0.02 --> no sleep time
        #* else sleep (0.02 - estimating_elapsed_time)
        time.sleep(self.ESTM_THREAD_SLEEP_TIME - estimating_elapsed_time if estimating_elapsed_time < self.ESTM_THREAD_SLEEP_TIME else 0)

        return g_pos_matrix, g_prox_vm, g_prox_uvm_fw

#* -------------- Estimation Part  --------------